classdef KitchenDemo < handle
    %% Kitchen Environment
    %sets up the static environment that the robots will be operating in

    methods
        %constructor
        function self = KitchenDemo()
            axis = [-3 2 -2 2 0 2];

            hold on;
            
            % place down the floor with the graphic
            surf([axis(1),axis(1);axis(2),axis(2)], [axis(3),axis(4);axis(3),axis(4)], repmat(axis(5),2,2), ...
                'CData', imread("floor.png"),'FaceColor','texturemap');
        
            % create a wall along the y axis
            surf([axis(1),axis(2);axis(1),axis(2)], [axis(4),axis(4);axis(4),axis(4)], ...
                [axis(6),axis(6);axis(5),axis(5)], ...
            'CData', imread("wall.jpg"),'FaceColor','texturemap');
        
            % create a wall along the y axis
            surf([axis(2),axis(2);axis(2),axis(2)], [axis(3),axis(4);axis(3),axis(4)], ...
                [axis(6),axis(6);axis(5),axis(5)], ...
            'CData', imread("wall.jpg"),'FaceColor','texturemap');
        
            counter = PlaceObject('bigcounterwithwall.ply', [0,0,0]);
            %counter2 = PlaceObject('bigcounter(broken).ply', [0,-1.1,0]);
            %counter3 = PlaceObject('bigcounter(broken).ply', [0,-2.2,0]);
            
            dispenser1 = PlaceObject('dispenser.PLY', [-1.5,1.25,1.5]);
            dispenser2 = PlaceObject('dispenser.PLY', [-1.1,1.25,1.5]);
            dispenser3 = PlaceObject('dispenser.PLY', [-0.7,1.25,1.5]);
            dispenser4 = PlaceObject('dispenser.PLY', [-0.3,1.25,1.5]);
            
            conveyer_enter = PlaceObject('conveyer_belt.PLY', [-1.65,0,0.92]);
            conveyer_exit = PlaceObject('conveyer_belt.PLY', [-1.65,-1.5,0.92]);
            
            oven = PlaceObject('pizzaoven.PLY', [0,0,0.91]);
        end
    end
end