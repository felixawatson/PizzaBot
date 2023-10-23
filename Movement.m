classdef Movement < handle
    %% Class for pizza cutting robot
    % uses visual servoing to detect when a pizza is in front of it and
    % cut it into 8 equal slices

    properties (Constant)
        
    end

    methods
        % constructor
        function self = Movement()
        end
    end

    methods (Static)
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
        function CartesianMove(robot,transform)
            steps = self.step;
            tf1 = robot.model.getpos.T;
            q1 = robot.model.getpos;
            tfMatrix = ctraj(tf1,transform,steps);    
            
            for i = 1:step
                if i == 1
                    qMatrix(i,:) = robot.model.ikcon(tfMatrix(:,:,i),q1);
                    lastpos = qMatrix(i,:);
                    robot.model.animate(qMatrix(i,:))
                    drawnow()
                else 
                    qMatrix(i,:) = robot.model.ikcon(tfMatrix(:,:,i),lastpos);
                    lastpos = qMatrix(i,:);
                    robot.model.animate(qMatrix(i,:))
                    drawnow()
                end
            end
        end

        % jogs robot in cartesian frame
        function CartJogRobot(robot,direction)
            step = 50;
            distance = 0.1;
            q1 = robot.model.getpos;
            tf = robot.model.fkine(q1).T;
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

            qMatrix = zeros(step,6);
            tfMatrix = ctraj(tf,jog,step);    

            for i = 1:step
                if i == 1
                    qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),q1);
                    lastpos = qMatrix(i,:);
                    robot.model.animate(qMatrix(i,:))
                    drawnow()
                else 
                    qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),lastpos);
                    lastpos = qMatrix(i,:);
                    robot.model.animate(qMatrix(i,:))
                    drawnow()
                end
            end
        end
        
        % jogs robots joints using a slider
        function JointJogRobot(robot,joint,sliderVal)
            
            % get current joint states
            q1 = robot.model.getpos;     
            
            % update to slider value
            q1(1,joint) = sliderVal;
            
            % animate
            robot.model.animate(q1);
        end
    end
end