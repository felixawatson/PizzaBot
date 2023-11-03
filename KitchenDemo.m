classdef KitchenDemo < handle
    %% Kitchen Environment
    %Sets up the static environment that the robots will be operating in
    events
        HumanDetected
    end

    properties
        
        dimensions = [3, 3.4, 1]; % Length, Width, Height
        X; % X limits for light curtain cube
        Y; % Y limits for light curtain cube
        Z; % Z limits for light curtain cube
        vertices;
        faces = [
            1, 2, 6, 5; % front
            2, 3, 7, 6; % right
            3, 4, 8, 7; % back
            4, 1, 5, 8; % left
            1, 2, 3, 4; % bottom
            5, 6, 7, 8  % top
        ];
        cube; % Light curtain object
        timer; % Callback timer
        babyLoc = [10,10,10]; % Location of baby.ply model
        baby; % Baby model for light curtain demo
    end

    methods
        % Constructor
        function self = KitchenDemo()
            axis = [-1.6 1.4 -1.4 2 0 1.5];

            hold on;

            % Place down the floor with the graphic
            surf([axis(1),axis(1);axis(2),axis(2)], [axis(3),axis(4);axis(3),axis(4)], repmat(axis(5),2,2), ...
                'CData', imread("floor.png"),'FaceColor','texturemap');
        
            % Create a wall along the y axis
            surf([axis(1),axis(2);axis(1),axis(2)], [axis(3),axis(3);axis(3),axis(3)], ...
                [axis(6),axis(6);axis(5),axis(5)], ...
            'CData', imread("wall.jpg"),'FaceColor','texturemap');
        
            % Create a wall along the y axis
            surf([axis(2),axis(2);axis(2),axis(2)], [axis(3),axis(4);axis(3),axis(4)], ...
                [axis(6),axis(6);axis(5),axis(5)], ...
            'CData', imread("wall.jpg"),'FaceColor','texturemap');

            % Place in kitchen object
            PlaceObject('environment.ply');
            % Add security camera
            PlaceObject('cctv.ply', [(axis(1)+0.2), axis(3), 1.1]);
            % Add estop button (non-functioning)
            PlaceObject('estop.ply', [axis(2), (axis(4)-0.5), 0.6]);
            % Add fire extinguisher
            PlaceObject('fireex.ply', [(axis(2)-0.1), (axis(4)-0.7), 0]);
            % Add safety indicator light 1
            PlaceObject('light.ply', [(axis(2)-0.125), (axis(4)-0.1), 0.95]);
            % Add safety indicator light 2
            PlaceObject('light.ply', [(axis(1)+0.15), (axis(4)-0.4), 0.4]);
            % Add pizza cutter holder
            PlaceObject('cutter holder.ply',[-1.35,-0.7,0.4]);

            self.X = [axis(1), axis(2), axis(2), axis(1), axis(1), axis(2), axis(2), axis(1)];
            self.Y = [axis(3), axis(3), axis(4), axis(4), axis(3), axis(3), axis(4), axis(4)];
            self.Z = [axis(5), axis(5), axis(5), axis(5), axis(6), axis(6), axis(6), axis(6)];

            % Rearrange the coordinates for vertices
            self.vertices = [self.X', self.Y', self.Z'];
            
            % Create an invisible cube around the working area
            self.cube = patch('Faces', self.faces, 'Vertices', self.vertices, 'FaceColor', 'r', 'FaceAlpha', 0.2);
            set(self.cube, 'Visible', 'off');
            
            % Create a timer to check periodically check the lightcurtain
            % Space for breaches
            self.timer = timer('ExecutionMode', 'fixedRate', 'Period', 0.5, 'TimerFcn', @self.checkCube);
        end

        % Start monitoring the light curtain space
        function startMonitoring(self)
            start(self.timer);
        end

        % Stop monitoring the light curtain space
        function stopMonitoring(self)
            stop(self.timer);
        end

        % Create a ply model of the baby and place it in the kitchen
        function WHSIncident(self)
            self.baby = PlaceObject('baby.ply', [0, 1, 0]);
            self.babyLoc = [0, 1, 0];
        end

        % Function that is periodically checked every 0.5 seconds as to
        % Fhether a human or other object has entered the workspace
        function checkCube(self, ~, ~)
            % Check if the object is inside the cube
            inside_curtain = all(self.babyLoc >= [-self.dimensions(1)/2, -self.dimensions(2)/2, 0]) && all(self.babyLoc <= self.dimensions/2);
            % If it is then warn the user and notify the listener in the
            % GUI class
            if inside_curtain
                disp('A person is within the hazard area');
                % Send a signal to the listener that an event has occured
                notify(self, 'HumanDetected');
                % Stop monitoring or the callback will keep getting
                % triggered
                self.stopMonitoring();
                % Flash the light curtain space to visually show the breach
                for i = 1:5
                    set(self.cube, 'Visible', 'on');
                    pause(0.5);
                    set(self.cube, 'Visible', 'off');
                    pause(0.5);
                end
                % Remove the baby from the scene
                delete(self.baby);
                % Change its location to be outside of the workspace again
                self.babyLoc = [10, 10, 10];
                % Start up monitoring again now that the area is clear
                self.startMonitoring();
            end
        end
    end
end