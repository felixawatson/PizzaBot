classdef SlicerBot < handle
    %% Class for pizza cutting robot
    % Cuts the pizza into 8 equal slices

    properties
        robot; % Base robot used for the slicer bot
        gripper; % Gripper on end effector of the robot
        cutter; % Pizza cutter object to be wielded
        cutteroffset = [0,0,0.1];
        base = [-1,-1,0.4]; % Approximate base position
        home = deg2rad([-120,-90,60,-60,-90,0]); % Home joint positions
        eStop = 0; % Estop flag
        cancelDemo = 0; % Demo cancelled flag   
        step = 50; % Number of steps for joint movements

    end

    methods
        % Constructor
        %create a self.robot with a pizza slicer attachment
        function self = SlicerBot()
            % Spawn in UR3 to desired location
            self.robot = UR3(transl(self.base(1), self.base(2), self.base(3)));
            % Rotate joints to home position
            self.robot.model.animate(self.home);
            % Determine end effector position of the UR3
            ee = self.robot.model.fkine(self.robot.model.getpos);
            
            % Create a gripper object
            self.gripper = ur3gripper;
            % Set gripper position to open
            self.gripper.base(ee,'open');
            self.gripper.open();

            % Create a cutter object
            self.cutter = cutter;
            % Place cutter into holder position
            self.cutter.model.base = transl([-1.35,-0.7,0.61])*troty(pi);
            self.cutter.model.animate(0);
        end

        % Move to home joint position
        function Home(self,grip,cutter)
            % Path plan
            steps = self.step;
            q1 = self.robot.model.getpos;
            q2 = self.home; 
            qMatrix = jtraj(q1,q2,steps);

            % Move to home
            for i = 1:steps
                % Estop flag check
                if self.EStopCheck()
                    return
                end
                self.robot.model.animate(qMatrix(i,:));
                % Animate gripper
                ee = self.robot.model.fkine(self.robot.model.getpos);  
                switch grip
                    case 'open'
                        self.gripper.base(ee,'open')
                    case 'closed'
                        self.gripper.base(ee,'closed')
                    otherwise
                        self.gripper.base(ee,'open')
                end                 
                if cutter == 0
                    drawnow()
                else
                    self.cutter.model.base = ee;
                    self.cutter.model.animate(0)
                end
                drawnow()
            end
        end

        % Get transform
        function [tf] = gettransform(self,q)
            if q == 0
                q1 = self.robot.model.getpos;            
                tf = self.robot.model.fkine(q1).T;
            else
                tf = self.robot.model.fkine(q).T;
            end
        end

        % RMRC 
        function [qMatrix] = RMRC(self,qMatrix)
            % Set parameters for the simulation
            t = 10;                             % Total time (s)
            steps = length(qMatrix);            % No. of steps for simulation
            deltaT = t/steps;                   % Control frequency
            delta = 2*pi/steps;                 % Small angle change
            epsilon = 0.1;                      % Threshold value for manipulability/Damped Least Squares
            W = diag([ 1 1 0.1 0.1 0.1]);      % Weighting matrix for the velocity vector
            
            % Allocate array data
            m = zeros(steps,1);                 % Array for Measure of Manipulability
            qdot = zeros(steps,6);              % Array for joint velocities
            theta = zeros(3,steps);             % Array for roll-pitch-yaw angles
            x = zeros(3,steps);                 % Array for x-y-z trajectory
            positionError = zeros(3,steps);     % For plotting trajectory error
            angleError = zeros(3,steps);        % For plotting trajectory error
            
            % Set up trajectory, initial pose
            for i=1:steps
                tr = self.robot.model.fkine(qMatrix(i,:)).T;
                rpy = tr2rpy(tr);
                x(1,i) = tr(1,4);       % Points in x
                x(2,i) = tr(2,4);       % Points in y
                x(3,i) = tr(3,4);       % Points in z
                theta(1,i) = rpy(1);    % Roll angle 
                theta(2,i) = rpy(2);    % Pitch angle
                theta(3,i) = rpy(3);    % Yaw angle
            end
             
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            q0 = qMatrix(1,:);                                                            % Initial guess for joint angles
            qMatrix(1,:) = self.robot.model.ikcon(T,q0);                                % Solve joint angles to achieve first waypoint
            
            % Track the trajectory with RMRC
            for i = 1:steps-1
                T = self.robot.model.fkine(qMatrix(i,:)).T;                             % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = self.robot.model.jacob0(qMatrix(i,:));                              % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.robot.model.qlim(j,1)     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.robot.model.qlim(j,2) % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                     	% Update next joint state based on joint velocities
            end
            for i = 1:length(qMatrix)
                self.robot.model.animate(qMatrix(i,:));
                % Animate gripper
                ee = self.robot.model.fkine(self.robot.model.getpos);  
                self.gripper.base(ee,'closed')
                drawnow()
            end
        end

        % Joint movement path
        function JointMove(self,transform,grip,cutter)
            % Plan path
            steps = self.step;
            q1 = self.robot.model.getpos;
            q2 = self.robot.model.ikcon(transform,q1); 
            qMatrix = jtraj(q1,q2,steps);  
        
            % Move joints
            for i = 1:steps
                % Check Estop flag
                if self.EStopCheck()
                    return
                end
                self.robot.model.animate(qMatrix(i,:));
                % Animate gripper
                ee = self.robot.model.fkine(self.robot.model.getpos);  
                switch grip
                    case 'open'
                        self.gripper.base(ee,'open')
                    case 'closed'
                        self.gripper.base(ee,'closed')
                    otherwise
                        self.gripper.base(ee,'open')
                end
                
                if cutter == 0
                    drawnow()
                    return
                else
                self.cutter.model.base = ee;
                self.cutter.model.animate(0)
                end
                drawnow()
            end
        end
        
        % Cartesian movement path
        function CartesianMove(self,transform)
            % Plan path
            steps = self.step;
            q1 = self.robot.model.getpos;            
            tf1 = self.robot.model.fkine(q1).T;
            tfMatrix = ctraj(tf1,transform,steps);    
            
            % Move in catersian plane
            for i = 1:steps
                % Check Estop flag
                if self.EStopCheck()
                    return
                end
                qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q1);
                q1 = qMatrix(i,:);
                self.robot.model.animate(qMatrix(i,:))
                ee = tfMatrix(:,:,i); 
                self.gripper.base(ee,'open');
                drawnow()
            end
        end

        % Jogs robot in cartesian frame 10cm given a certain direction
        function CartJogRobot(self,direction)
            % Plan path
            steps = self.step;
            distance = 0.1;
            q1 = self.robot.model.getpos();
            tf = self.robot.model.fkine(q1).T;
            jog = tf;
            
            % Consider which direction was inputted
            switch direction
                case '-x'
                    jog(1,4) = jog(1,4) - distance;
                case '+x'
                    jog(1,4) = jog(1,4) + distance;
                case '-y'
                    jog(2,4) = jog(2,4) - distance;
                case '+y'
                    jog(2,4) = jog(2,4) + distance;
                case '-z'
                    jog(3,4) = jog(3,4) - distance;
                case '+z'
                    jog(3,4) = jog(3,4) + distance;
                otherwise
                    return
            end

            qMatrix = zeros(steps,6);
            tfMatrix = ctraj(tf,jog,steps);  

            % Move in cartesian direction
            for i = 1:steps
                % Check Estop flag
                if self.EStopCheck()
                    return
                end
                qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q1);
                q1 = qMatrix(i,:);
                self.robot.model.animate(qMatrix(i,:))
                ee = self.robot.model.fkine(self.robot.model.getpos);  
                self.gripper.base(ee,'open');
                drawnow()
            end
        end

        % jogs robots joints using a slider
        function JointJogRobot(self,joint,sliderVal)
            
            % get current joint states
            q1 = self.robot.model.getpos();     
            
            % update to slider value
            q1(1,joint) = deg2rad(sliderVal);
            
            % animate
            self.robot.model.animate(q1);
            ee = self.robot.model.fkine(self.robot.model.getpos);  
            self.gripper.base(ee,'open');
            drawnow()
        end
        
        % Estop function
        function cancel = EStopCheck(self)
            % While the Estop is active, wait here
            while self.eStop
                % Pause allows other functionality to continue
                pause(0.5);
            end
            % If the demo is confirmed as cancelled then raise flag
            if self.cancelDemo
                cancel = true;
            % Otherwise program continues
            else
                cancel = false;
            end
        end

        % Calculate cut based on pizza position
        function [jointstates] = CalculateCut(self)
            steps = 50; % Change interpolation
            rad = 0.3/2; % Size of pizza
            center = [-0.8,-0.7,0.65]; % Pizza center
            diag = rad*sin(pi/4);
            tf = transl(center - [0,0,0]);

            % First cut
            p01 = transl(center - [rad,0,0]) * trotx(pi);
            p02 = transl(center + [rad,0,0]) * trotx(pi);
            p03 = p02 * transl([0,0,-0.1]); % way point
            % Second cut
            p04 = transl(center - [0,rad,0]) * trotx(pi) * trotz(pi/2); % 8
            p05 = transl(center + [0,rad,0]) * trotx(pi) * trotz(pi/2); % 7
            % Third cut
            p07 = transl(center - [diag,diag,0]) * trotx(pi) * trotz(-pi/4); % 
            p08 = transl(center + [diag,diag,0]) * trotx(pi) * trotz(-pi/4); % 
            p09 = p07 * transl([0,0,-0.1]); % Waypoint
            % Fourth cut
            p10 = transl(center + [diag,-diag,0]) * trotx(pi) * trotz(pi/4); % 4
            p11 = transl(center + [-diag,diag,0]) * trotx(pi) * trotz(pi/4); % 5
            p06 = p11 * transl([0,0,-0.1]); % Waypoint 6
            
            jointstates = zeros(13*steps,6);
                        
            q1 = self.robot.model.getpos;
            q2 = self.home; % Make it a TR
            qMatrix = jtraj(q1,q2,steps);  
            jointstates(1:steps,:) = qMatrix;
            
            q1 = q2;
            q2 = self.robot.model.ikcon(p01,q1); % Make it a TR
            qMatrix = jtraj(q1,q2,steps);   
            jointstates(1+steps:2*steps,:) = qMatrix;
            
            % Cut one
            tfMatrix = ctraj(p01,p02,steps);   
            for i = 1:steps
                if self.EStopCheck()
                    return
                end
                qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q2);
                q2 = qMatrix(i,:);
            end
            jointstates(1+2*steps:3*steps,:) = qMatrix;
            
            % Waypoint
            q1 = q2;
            q2 = self.robot.model.ikcon(p03,q2); % Make it a TR
            qMatrix = jtraj(q1,q2,steps);    
            jointstates(1+3*steps:4*steps,:) = qMatrix;
            
            % Waypoint
            q1 = q2;
            q2 = self.robot.model.ikcon(p10,q2); % Make it a TR
            qMatrix = jtraj(q1,q2,steps); 
            jointstates(1+4*steps:5*steps,:) = qMatrix;
            
            % Cut two
            tfMatrix = ctraj(p10,p11,steps);   
            for i = 1:steps
                if self.EStopCheck()
                    return
                end
                qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q2);
                q2 = qMatrix(i,:);
            end
            jointstates(1+5*steps:6*steps,:) = qMatrix;
            
            % Waypoint
            q1 = q2;
            q2 = self.robot.model.ikcon(p06,q1); % Make it a TR
            qMatrix = jtraj(q1,q2,steps);    
            jointstates(1+6*steps:7*steps,:) = qMatrix;
            
            q1 = q2;
            q2 = self.robot.model.ikcon(p05,q1); % Make it a TR
            qMatrix = jtraj(q1,q2,steps);  
            jointstates(1+7*steps:8*steps,:) = qMatrix;
            
            % Cut three
            tfMatrix = ctraj(p05,p04,steps);   
            for i = 1:steps
                if self.EStopCheck()
                    return
                end
                qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q2);
                q2 = qMatrix(i,:);
            end
            jointstates(1+8*steps:9*steps,:) = qMatrix;
            
            % Waypoint
            q1 = q2;
            q2 = self.robot.model.ikcon(p09,q1); % Make it a TR
            qMatrix = jtraj(q1,q2,steps);  
            jointstates(1+9*steps:10*steps,:) = qMatrix;
            
            % Waypoint
            q1 = q2;
            q2 = self.robot.model.ikcon(p07,q1); % Make it a TR
            qMatrix = jtraj(q1,q2,steps);  
            jointstates(1+10*steps:11*steps,:) = qMatrix;
            
            % Cut four
            tfMatrix = ctraj(p07,p08,steps);   
            for i = 1:steps
                if self.EStopCheck()
                    return
                end
                qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q2);
                q2 = qMatrix(i,:);
            end
            jointstates(1+11*steps:12*steps,:) = qMatrix;
            
            % Home
            q1 = q2;
            q2 = self.home; %make it a TR
            qMatrix = jtraj(q1,q2,steps);   
            jointstates(1+12*steps:13*steps,:) = qMatrix;
        end
    
        % Step through slicing procedure
        function stepSlicing(self,qMatrix,cutter)
            for i = 1:length(qMatrix)
                if self.EStopCheck()
                    return
                end
                self.robot.model.animate(qMatrix(i,:));
                ee = self.robot.model.fkine(self.robot.model.getpos);  
                self.gripper.base(ee,'closed')
                if cutter == 0
                    drawnow()
                    return
                else
                self.cutter.model.base = ee; 
                self.cutter.model.animate(0)
                end
                drawnow();
            end
        end
    end
end