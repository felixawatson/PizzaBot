classdef margherita < handle
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
        function self = margherita (pizzaCount,basePose)
            if 0 < nargin
                self.pizzaCount = pizzaCount;
            end
            
            self.workspaceDimensions = [-self.paddockSize(1)/2, self.paddockSize(1)/2 ...
                                       ,-self.paddockSize(2)/2, self.paddockSize(2)/2 ...
                                       ,0,self.maxHeight];

            % Create the required number of cows
            for i = 1:self.pizzaCount
                self.pizzaModel{i} = self.GetpizzaModel(['brick',num2str(i)]);
                self.pizzaModel{i}.base = transl(basePose(i,:)) * trotx(-pi/2);
                
                % Plot 3D model
                plot3d(self.pizzaModel{i},0,'workspace',self.workspaceDimensions,'view',[20,20],'delay',0,'noarrow','nowrist');
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

            % axis equal
            % if isempty(findobj(get(gca,'Children'),'Type','Light'))
            %     camlight
            % end 
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
        
        %% PlotSingleRandomStep
        % Move each of the cows forward and rotate some rotate value around
        % the z axis
        function PlotSingleRandomStep(self)
            for cowIndex = 1:self.pizzaCount
                % Move Forward
                self.pizzaCount{cowIndex}.base = self.pizzaCount{cowIndex}.base * SE3(SE2(0.2, 0, 0));
                animate(self.pizzaCount{cowIndex},0);
                
                % Turn randomly
                % Save base as a temp variable
                tempBase = self.pizzaModel{cowIndex}.base.T;
                rotBase = tempBase(1:3, 1:3);
                posBase = tempBase(1:3, 4);
                newRotBase = rotBase * rotz((rand-0.5) * 30 * pi/180);
                newBase = [newRotBase posBase ; zeros(1,3) 1];
                           
                % Update base pose
                self.pizzaModel{cowIndex}.base = newBase;
                animate(self.pizzaModel{cowIndex},0);                

                % If outside workspace rotate back around
                % Get base as temp
                tempBase = self.pizzaModel{cowIndex}.base.T;
                
                if tempBase(1,4) < self.workspaceDimensions(1) ...
                || self.workspaceDimensions(2) < tempBase(1,4) ...
                || tempBase(2,4) < self.workspaceDimensions(3) ...
                || self.workspaceDimensions(4) < tempBase(2,4)
                    self.pizzaModel{cowIndex}.base = self.pizzaModel{cowIndex}.base * SE3(SE2(-0.2, 0, 0)) * SE3(SE2(0, 0, pi));
                end
            end
            % Do the drawing once for each interation for speed
            drawnow();
        end    
        
        %% TestPlotManyStep
        % Go through and plot many random walk steps
        function TestPlotManyStep(self,numSteps,delay)
            if nargin < 3
                delay = 0;
                if nargin < 2
                    numSteps = 200;
                end
            end
            for i = 1:numSteps
                self.PlotSingleRandomStep();
                pause(delay);
            end
        end
    end
    
    methods (Static)
        %% GetCowModel
        function model = GetpizzaModel(name)
            if nargin < 1
                name = 'Pizza';
            end
            [faceData,vertexData] = plyread('BigSausagePizza.ply','tri'); %change ply once files are made
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