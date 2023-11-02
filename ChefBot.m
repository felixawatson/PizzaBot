classdef ChefBot < handle
    %% Class for pizza preparation robot
    % Pizza bot puts ingredients on the pizza and then places the pizza in
    % the oven to be cooked. Once cooked the pizza is placed in front of
    % the slicer bot

    % movement through functions might make it difficult to animate grippers and
    % pizza staying on the ee, 

    properties
        robot; 
        gripper;
        eescoopoffset = transl([-0.05,0,0.35]);
        home = [-pi/2 -pi/4 3*pi/4 0 0 0];
        eStop = 0;
        cancelDemo = 0;
        step = 50;
    end

    methods
        % constructor
        function self = ChefBot()
            % create IRB1200 with gripper
            self.robot = IRB1200H();
            self.robot.model.animate(self.home);
            ee = self.robot.model.fkine(self.robot.model.getpos);
            self.gripper = scoop;
            self.gripper.model.base = ee;
            self.gripper.model.animate(0);
        end

        % collision avoidance
        function ObjectAvoidance(self)
        end

        % Move to home position
        function Home(self)
            steps = self.step;
            q1 = self.robot.model.getpos;
            q2 = self.home; 
            qMatrix = jtraj(q1,q2,steps);  

            for i = 1:steps
                if self.EStopCheck()
                    return
                end
                self.robot.model.animate(qMatrix(i,:));
                %animate gripper
                ee = self.robot.model.fkine(self.robot.model.getpos); 
                self.gripper.model.base = ee;
                self.gripper.model.animate(0);
                drawnow()
            end
        end

        % calculate motion
        function [qMatrix] = CalcMotion(self,transform)
            steps = self.step;
            q1 = self.robot.model.getpos;
            q2 = self.robot.model.ikcon(transform,q1); 
            qMatrix = jtraj(q1,q2,steps);  
        end

        % animate calculated motion
        function stepCalcMotion(self,qMatrix,pizza)
            for i = 1:length(qMatrix)
                self.robot.model.animate(qMatrix(i,:));
                %animate gripper
                ee = self.robot.model.fkine(self.robot.model.getpos); 
                self.gripper.model.base = ee;
                self.gripper.model.animate(0);
                try  
                    pizza.model.base = ee * transl(self.eescoopoffset); 
                    pizza.model.animate(0)  
                end
                drawnow()
            end
        end

        % Resolved Motion Rate Control
        function [qMatrix] = RMRC(self,qMatrix)
            % 1.1) Set parameters for the simulation
            t = 10;                             % Total time (s)
            steps = length(qMatrix);                        % No. of steps for simulation
            deltaT = t/steps;                   % Control frequency
            delta = 2*pi/steps;                 % Small angle change
            epsilon = 0.1;                      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 1 1 1]);      % Weighting matrix for the velocity vector
            
            % 1.2) Allocate array data
            m = zeros(steps,1);                 % Array for Measure of Manipulability
            qdot = zeros(steps,6);              % Array for joint velocities
            theta = zeros(3,steps);             % Array for roll-pitch-yaw angles
            x = zeros(3,steps);                 % Array for x-y-z trajectory
            positionError = zeros(3,steps);     % For plotting trajectory error
            angleError = zeros(3,steps);        % For plotting trajectory error
            
            % 1.3) Set up trajectory, initial pose
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
            
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                % UPDATE: fkine function now returns an SE3 object. To obtain the 
                % Transform Matrix, access the variable in the object 'T' with '.T'.
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
        end
        
        % get transform
        function [tf] = gettransform(self,q)
            if q == 0
                q1 = self.robot.model.getpos;            
                tf = self.robot.model.fkine(q1).T;
            else
                tf = self.robot.model.fkine(q).T;
            end
        end

        % joint movement path
        function JointMove(self,transform,pizza)
            steps = self.step;
            q1 = self.robot.model.getpos;
            q2 = self.robot.model.ikcon(transform,q1); 
            qMatrix = jtraj(q1,q2,steps);  
            % qMatrix = self.RMRC(qMatrix);
            for i = 1:steps
                if self.EStopCheck()
                    return
                end
                self.robot.model.animate(qMatrix(i,:));
                ee = self.robot.model.fkine(self.robot.model.getpos); 
                self.gripper.model.base = ee;
                self.gripper.model.animate(0);
                try  
                    pizza.model.base = ee * transl(self.eescoopoffset); 
                    pizza.model.animate(0)  
                end
                drawnow()
            end
        end

        % cartesian movement path 
        function CartesianMove(self,transform,pizza)
            steps = self.step;
            q1 = self.robot.model.getpos;            
            tf1 = self.robot.model.fkine(q1).T;
            tfMatrix = ctraj(tf1,transform,steps);    
            
            for i = 1:steps
                if self.EStopCheck()
                    return
                end
                qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q1);
                q1 = qMatrix(i,:);
                self.robot.model.animate(qMatrix(i,:))
                ee = self.robot.model.fkine(self.robot.model.getpos);  
                self.gripper.model.base = ee;
                self.gripper.model.animate(0);
                try 
                    pizza.model.base = ee * transl(self.eescoopoffset);
                    pizza.model.animate(0)  
                catch
                end
                drawnow()
            end
        end

        % jogs robot in cartesian frame
        function CartJogRobot(self,direction)
            steps = 50;
            distance = 0.1;
            q1 = self.robot.model.getpos();
            tf = self.robot.model.fkine(q1).T;
            jog = tf;
                        
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

            for i = 1:steps
                if self.EStopCheck()
                    return
                end
                qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q1);
                q1 = qMatrix(i,:);
                self.robot.model.animate(qMatrix(i,:))
                ee = self.robot.model.fkine(self.robot.model.getpos);  
                self.gripper.model.base = ee;
                self.gripper.model.animate(0);
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
            self.gripper.model.base = ee;
            self.gripper.model.animate(0);
            drawnow()
        end

        function cancel = EStopCheck(self)
            while self.eStop
                pause(0.5);
            end
            
            if self.cancelDemo
                cancel = true;
            else
                cancel = false;
            end
            self.cancelDemo = 0;
        end
    end
end