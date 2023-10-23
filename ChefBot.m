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

    end
end