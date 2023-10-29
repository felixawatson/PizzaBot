classdef SlicerBot < handle
    %% Class for pizza cutting robot
    % uses visual servoing to detect when a pizza is in front of it and
    % cut it into 8 equal slices

    properties
        robot;
        step = 50;
    end

    methods
        % constructor
        function self = SlicerBot()
            %create a UR3 with a pizza slicer attachment
            self.robot = UR3;
        end

            % joint movement path
        function JointMove(robot, transform)
            steps = self.step;
            q1 = robot.model.getpos;
            q2 = robot.model.ikcon(transform); 
            qMatrix = jtraj(q1,q2,steps);    
        
            for i = 1:self.step
                robot.model.animate(qMatrix(i,:));
                %animate gripper
                % ee = r.model.fkine(r.model.getpos).T;  
                % gripper.model.base = ee
                drawnow()
            end
        end
        
        % cartesian movement path (not tested)
        function CartesianMove(self,transform)
            steps = self.step;
            tf1 = self.robot.model.getpos.T;
            q1 = self.robot.model.getpos;
            tfMatrix = ctraj(tf1,transform,steps);    
            
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
            q1 = self.robot.model.getpos();     
            
            % update to slider value
            q1(1,joint) = deg2rad(sliderVal);
            
            % animate
            self.robot.model.animate(q1);
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