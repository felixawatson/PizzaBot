classdef Pizza < RobotBaseClass
    %ROBOTPIZZAS A class that creates a herd of robot PIZZAS

    properties(Access = public)
        plyFileNameStem = 'Base';
        
    end
    
    methods
        % Constructor
        function self = Pizza(baseTr, toppings)
            if nargin < 2
                if nargin == 0
                    baseTr = transl(0,0,0);                
                end
            else
            % Graphical model changes based on selected pizza
            self.plyFileNameStem = toppings;
            end 
            self.CreateModel();
            self.model.base = self.model.base.T * baseTr;
            self.PlotAndColourRobot();
        end

        function CreateModel(self)
            % Single link 'robot'
            link(1) = Link([0  0  0  0  0]);
            self.model = SerialLink(link, 'name', self.name);
        end
    end          
end