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
            steps = 100;
            q1 = self.robot.model.getpos;
            q2 = self.robot.model.ikcon(transform,q1); 
            qMatrix = jtraj(q1,q2,steps);    
        
            for i = 1:steps
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

        % cartesian movement path 
        function CartesianMove(self,transform,pizza)
            steps = 100;
            q1 = self.robot.model.getpos;            
            tf1 = self.robot.model.fkine(q1).T;
            tfMatrix = ctraj(tf1,transform,steps);    
            
            for i = 1:steps
                if i == 1
                    qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),q1);
                    lastpos = qMatrix(i,:);
                    self.robot.model.animate(qMatrix(i,:))
                    ee = self.robot.model.fkine(self.robot.model.getpos);  
                    self.gripper.model.base = ee;
                    self.gripper.model.animate(0);
                   
                    try 
                        pizza.model.base = ee * transl(self.eescoopoffset);
                        pizza.model.animate(0)  
                    end
                    drawnow()
                else 
                    qMatrix(i,:) = self.robot.model.ikcon(tfMatrix(:,:,i),lastpos);
                    lastpos = qMatrix(i,:);
                    self.robot.model.animate(qMatrix(i,:))
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
end