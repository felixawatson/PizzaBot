classdef Pizza < RobotBaseClass
    %ROBOTPIZZAS A class that creates a herd of robot PIZZAS

    properties(Access = public)
        plyFileNameStem = 'Base';
        
    end
    
    methods
        function self = Pizza(baseTr, toppings)
            if nargin < 2
                if nargin == 0
                    baseTr = transl(0,0,0);                
                end
            else
            self.plyFileNameStem = toppings;
            end 
            self.CreateModel();
            self.model.base = self.model.base.T * baseTr;
            self.PlotAndColourRobot();
        end

        function CreateModel(self)
            % single link 'robot'
            link(1) = Link([0  0  0  0  0]);
            self.model = SerialLink(link, 'name', self.name);
        end
    end          
end