classdef ChefBot < handle
    %% Class for pizza preparation robot
    % Pizza bot puts ingredients on the pizza and then places the pizza in
    % the oven to be cooked. Once cooked the pizza is placed in front of
    % the slicer bot

    % movement through functions might make it difficult to animate grippers and
    % pizza staying on the ee, 

    properties (Constant)
        robot = IRB1200H; 
        home = [0 0 pi/2 0 0 0];
    end

    methods
        % constructor
        function self = ChefBot()
            % create IRB1200 with gripper
            home = self.home;
            IRB = IRB1200h;
            IRB.model.animate(home);
        end

        % collision avoidance
        function ObjectAvoidance(self)
        end

        % place ingredients
        function AddToppings(self)
        end

        % puts pizza in the oven to be cooked
        function CookPizza(self)
            % set local after plys are made. Change in xy to then cart move in
            % self.JointMove(ovenlocation - [0 0.5 0]); 
            % self.CartesianMove(ovenlocation);

        end

        % opens the gripper
        function OpenGripper(self)
        end

        % close gripper
        function CloseGripper(self)
        end
    end
    methods (Static)
        % joint movement path
        function JointMove(robot, gripper, transform)
            steps = 100;
            q1 = robot.model.getpos;
            q2 = robot.model.ikcon(transform); 
            qMatrix = jtraj(q1,q2,steps);    
        
            for i = 1:self.step
                robot.model.animate(qMatrix(i,:));
                %animate gripper
                ee = r.model.fkine(r.model.getpos).T;  
                gripper.model.base = ee;
                gripper.model.animate();
                drawnow()
            end
        end
        
        % cartesian movement path (not tested)
        function CartesianMove(self,gripper, transform)
            steps = 100;
            q1 = self.robot.model.getpos;            
            tf1 = self.robot.model.fkine(q1).T;
            tfMatrix = ctraj(tf1,transform,steps);    
            
            for i = 1:steps
                if i == 1
                    qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q1);
                    lastpos = qMatrix(i,:);
                    self.robot.model.animate(qMatrix(i,:))
                    ee = r.model.fkine(r.model.getpos).T;  
                    gripper.model.base = ee;
                    gripper.model.animate();
                    drawnow()
                else 
                    qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),lastpos);
                    lastpos = qMatrix(i,:);
                    self.robot.model.animate(qMatrix(i,:))
                    ee = r.model.fkine(r.model.getpos).T;  
                    gripper.model.base = ee;
                    gripper.model.animate();
                    drawnow()
                end
            end
        end

        % jogs robot in cartesian frame
        function CartJogRobot(self,direction)
            steps = self.step;
            distance = 0.1;
            q1 = self.robot.model.getpos;
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
                if i == 1
                    qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q1);
                    lastpos = qMatrix(i,:);
                    self.robot.model.animate(qMatrix(i,:))
                    drawnow()
                else 
                    qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),lastpos);
                    lastpos = qMatrix(i,:);
                    self.robot.model.animate(qMatrix(i,:))
                    drawnow()
                end
            end
        end
        
        % jogs robots joints using a slider
        function JointJogRobot(self,joint,sliderVal)
            
            % get current joint states
            q1 = self.robot.model.getpos;     
            
            % update to slider value
            q1(1,joint) = deg2rad(sliderVal);
            
            % animate
            self.robot.model.animate(q1);
        end
    end

    methods (Static)
    end
end