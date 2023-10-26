classdef PizzaBot < handle
    %ROBOTPIZZAS A class that creates a herd of robot PIZZAS
	%   The pizzas can be moved around randomly. It is then possible to query
    %   the current location (base) of the pizza.    
    
    %#ok<*TRYNC>    

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end
    
    properties
        %> Number of cows
        pizzaCount = 1;
        
        %> A cell structure of \c cowCount cow models
        pizzaModel;
        
        %> paddockSize in meters
        paddockSize = [2,2];        
        
        %> Dimensions of the workspace in regard to the padoc size
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = PizzaBot(name,pizzaCount,basePose)
            if 0 < nargin
                self.pizzaCount = pizzaCount;
            end
            
            self.workspaceDimensions = [-self.paddockSize(1)/2, self.paddockSize(1)/2 ...
                                       ,-self.paddockSize(2)/2, self.paddockSize(2)/2 ...
                                       ,0,self.maxHeight];

            % Create the required number of cows
            for i = 1:self.pizzaCount
                self.pizzaModel{i} = self.GetpizzaModel([name,num2str(i)]);
                self.pizzaModel{i}.base = transl(basePose(i,:)) * trotx(-pi/2);
                
                % Plot 3D model
                plot3d(self.pizzaModel{i},0,'workspace',self.workspaceDimensions,'view',[20,20],'delay',0,'noarrow','nowrist');
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end
        end
        
        function delete(self)
            for index = 1:self.pizzaCount
                %handles = findobj('Tag', self.brickModel{index}.name);
                %h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end         
    end
    
    methods (Static)
        %% GetCowModel
        function model = GetpizzaModel(name)
            if nargin < 1
                name = 'base';
            end
            % switch name
            %     case 'base'
            %         [faceData,vertexData] = plyread('base.ply','tri');
            %     case 'margherita'
            %         [faceData,vertexData] = plyread('margherita.ply','tri'); 
            %     case 'hawaiian'
            %         [faceData,vertexData] = plyread('pepperoni.ply','tri'); 
            %     case 'pepperoni'
            %         [faceData,vertexData] = plyread('hawaiian.ply','tri'); 
            %     case 'four cheeses'
                    [faceData,vertexData] = plyread('fourcheeses.ply','tri'); 
            % end

            link1 = Link('alpha',pi/2,'a',0,'d',0,'offset',0);
            model = SerialLink(link1,'name',name);
            
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end    
end