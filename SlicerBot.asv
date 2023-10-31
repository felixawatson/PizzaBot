classdef SlicerBot < handle
    %% Class for pizza cutting robot
    % uses visual servoing to detect when a pizza is in front of it and
    % cut it into 8 equal slices

    properties
        robot;
        gripper;
        base = [-1,-1,0.4]; % approx
        home = deg2rad([-120,-90,60,-60,-90,0]);
    end

    methods
        % constructor
        function self = SlicerBot()
            %create a UR3 with a pizza slicer attachment
            self.robot = UR3;
            self.gripper = ur3gripper;
            self.robot.model.base = self.base;
            self.robot.model.animate(self.home);
            ee = self.robot.model.fkine(self.robot.model.getpos);
            self.gripper.base(ee,'open');
            self.gripper.open();
        end

        % joint movement path
        function JointMove(self,transform)
            steps = 100;
            q1 = self.robot.model.getpos;
            q2 = self.robot.model.ikcon(transform); 
            qMatrix = jtraj(q1,q2,steps);    
        
            for i = 1:self.step
                self.robot.model.animate(qMatrix(i,:));
                %animate gripper
                ee = self.robot.model.fkine(self.robot.model.getpos);  
                self.gripper.base(ee)
                drawnow()
            end
        end
        
        % cartesian movement path (not tested)
        function CartesianMove(self,transform)
            steps = 100;
            q1 = self.robot.model.getpos;            
            tf1 = self.robot.model.fkine(q1).T;
            tfMatrix = ctraj(tf1,transform,steps);    
            
            for i = 1:steps
                qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q1);
                q1 = qMatrix(i,:);
                self.robot.model.animate(qMatrix(i,:))
                ee = tfMatrix(:,:,i); 
                self.gripper.model.base = ee;
                self.gripper.base(ee);
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
    end

    methods (Static)
        % use RGB-D camera to detect the pizza [BONUS]
        function DetectPizza()
        end

        % calculate cut based on pizza position
        function CalculateCut()
        end

        % step through slicing procedure
        function stepSlicing()
        end
    end
end