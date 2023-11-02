classdef KitchenDemo < handle
    %% Kitchen Environment
    %sets up the static environment that the robots will be operating in

    methods
        %constructor
        function self = KitchenDemo()
            axis = [-1.6 1.4 -1.4 2 0 1.5];

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

        end
    end
end