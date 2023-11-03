classdef KitchenDemo < handle
    %% Kitchen Environment
    %sets up the static environment that the robots will be operating in
    events
        HumanDetected
    end

    properties
        dimensions = [3, 3.4, 1]; % Length, Width, Height
        X;
        Y;
        Z;
        vertices;
        faces = [
            1, 2, 6, 5; % front
            2, 3, 7, 6; % right
            3, 4, 8, 7; % back
            4, 1, 5, 8; % left
            1, 2, 3, 4; % bottom
            5, 6, 7, 8  % top
        ];
        cube;
        timer;
        babyLoc = [10,10,10];
        baby;
    end

    methods
        %constructor
        function self = KitchenDemo()
            axis = [-1.6 1.4 -1.4 2 0 1.5];

            hold on;

            % place down the floor with the graphic
            surf([axis(1),axis(1);axis(2),axis(2)], [axis(3),axis(4);axis(3),axis(4)], repmat(axis(5),2,2), ...
                'CData', imread("floor.png"),'FaceColor','texturemap');
        
            % create a wall along the y axis
            surf([axis(1),axis(2);axis(1),axis(2)], [axis(3),axis(3);axis(3),axis(3)], ...
                [axis(6),axis(6);axis(5),axis(5)], ...
            'CData', imread("wall.jpg"),'FaceColor','texturemap');
        
            % create a wall along the y axis
            surf([axis(2),axis(2);axis(2),axis(2)], [axis(3),axis(4);axis(3),axis(4)], ...
                [axis(6),axis(6);axis(5),axis(5)], ...
            'CData', imread("wall.jpg"),'FaceColor','texturemap');

            PlaceObject('environment.ply');
            PlaceObject('cctv.ply', [(axis(1)+0.2), axis(3), 1.1]);
            PlaceObject('estop.ply', [axis(2), (axis(4)-0.5), 0.6]);
            PlaceObject('fireex.ply', [(axis(2)-0.1), (axis(4)-0.7), 0]);    
            PlaceObject('light.ply', [(axis(2)-0.125), (axis(4)-0.1), 0.95]);
            PlaceObject('light.ply', [(axis(1)+0.15), (axis(4)-0.4), 0.4]);
            PlaceObject('cutter holder.ply',[-1.35,-0.7,0.4]);

            self.X = [axis(1), axis(2), axis(2), axis(1), axis(1), axis(2), axis(2), axis(1)];
            self.Y = [axis(3), axis(3), axis(4), axis(4), axis(3), axis(3), axis(4), axis(4)];
            self.Z = [axis(5), axis(5), axis(5), axis(5), axis(6), axis(6), axis(6), axis(6)];

            % Rearrange the coordinates for vertices
            self.vertices = [self.X', self.Y', self.Z'];

            self.cube = patch('Faces', self.faces, 'Vertices', self.vertices, 'FaceColor', 'r', 'FaceAlpha', 0.2);
            set(self.cube, 'Visible', 'off');

            self.timer = timer('ExecutionMode', 'fixedRate', 'Period', 0.5, 'TimerFcn', @self.checkCube);
        end

        function startMonitoring(self)
            start(self.timer);
        end

        function stopMonitoring(self)
            stop(self.timer);
        end

        function WHSIncident(self)
            self.baby = PlaceObject('baby.ply', [0, 1, 0]);
            self.babyLoc = [0, 1, 0];
        end

        function checkCube(self, ~, ~)
            % Check if the object is inside the cube
            inside_curtain = all(self.babyLoc >= [-self.dimensions(1)/2, -self.dimensions(2)/2, 0]) && all(self.babyLoc <= self.dimensions/2);
            if inside_curtain
                disp('A person is within the hazard area');
                set(self.cube, 'Visible', 'off');
                notify(self, 'HumanDetected');
                self.stopMonitoring();
                for i = 1:5
                    set(self.cube, 'Visible', 'on');
                    pause(0.5);
                    set(self.cube, 'Visible', 'off');
                    pause(0.5);
                end
                delete(self.baby);
                self.babyLoc = [10, 10, 10];
                self.startMonitoring();
            end
        end
    end
end