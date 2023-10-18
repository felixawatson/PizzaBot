classdef ChefBot < handle
    %% Class for pizza preparation robot
    % Pizza bot puts ingredients on the pizza and then places the pizza in
    % the oven to be cooked. Once cooked the pizza is placed in front of
    % the slicer bot

    % movement through functions might make it difficult to animate grippers and
    % pizza staying on the ee, 

    properties (Constant)
        robot = IRB1200H; 
        step = 50;
    end

    methods
        % constructor
        function self = ChefBot()
            % create IRB1200 with gripper
            
            % just for testing, can remove
            IRB = IRB1200H;
            home = [0 0 pi/2 0 0 0];
            IRB.model.animate(home);
            
            self.JogRobot(IRB,'+x');
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
            self.JointMove(ovenlocation - [0 0.5 0]); 
            self.CartesianMove(ovenlocation);

        end

        % opens the gripper
        function OpenGripper(self)
        end

        % close gripper
        function CloseGripper(self)
        end
    end

    methods (Static)
        % joint movement path (not tested)
        function JointMove(xyz_point)
            r = ChefBot.robot;
            steps = self.step;
            q1 = r.model.getpos;
            q2 = r.model.ikcon(xyz_point); 
            qMatrix = jtraj(q1,q2,steps);    
        
            for i = 1:self.step
                r.model.animate(qMatrix(i,:));
                %animate gripper
                % ee = r.model.fkine(r.model.getpos).T;  
                % gripper.model.base = ee
                drawnow()
            end
        end

        % cartesian movement path (not tested)
        function CartesianMove(robot,transform)
            IRB = robot;
            steps = self.step;
            tf1 = IRB.model.getpos.T;
            q1 = IRB.model.getpos;
            tfMatrix = ctraj(tf1,transform,steps);    
            
            for i = 1:step
                if i == 1
                    qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),q1);
                    lastpos = qMatrix(i,:);
                    IRB.model.animate(qMatrix(i,:))
                    drawnow()
                else 
                    qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),lastpos);
                    lastpos = qMatrix(i,:);
                    IRB.model.animate(qMatrix(i,:))
                    drawnow()
                end
            end
        end
       
        % jogs robot in cartesian frame
        function JogRobot(robot,direction)
            IRB = robot;
            step = 50;
            distance = 0.1;
            q1 = IRB.model.getpos;
            tf = IRB.model.fkine(q1).T;
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
                    IRB.model.animate(qMatrix(i,:))
                    drawnow()
                else 
                    qMatrix(i,:) = IRB.model.ikcon(tfMatrix(:,:,i),lastpos);
                    lastpos = qMatrix(i,:);
                    IRB.model.animate(qMatrix(i,:))
                    drawnow()
                end
            end

        end
    end
end