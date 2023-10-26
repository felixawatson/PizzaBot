%function [] = loadenvi(baseloc) %loadenvi function will load the environment surrounding the robot
close all
clc

hold on
axis equal

%loading the ground texture
surf([-5,-5;5,5]...
,[-4,4;-4,4]...
,[0,0;0,0]...
,'CData',imread('floor.png') ...
,'FaceColor','texturemap');

%loading walls

%loading kitchen
counter = PlaceObject('bigcounterwithwall.ply', [0,0,0]);
counter2 = PlaceObject('bigcounter.ply', [0,-1.1,0]);
counter3 = PlaceObject('bigcounter.ply', [0,-2.2,0]);

dispenser1 = PlaceObject('dispenser.PLY', [-1.5,1.25,1.5]);
dispenser2 = PlaceObject('dispenser.PLY', [-1.1,1.25,1.5]);
dispenser3 = PlaceObject('dispenser.PLY', [-0.7,1.25,1.5]);
dispenser4 = PlaceObject('dispenser.PLY', [-0.3,1.25,1.5]);

conveyer_enter = PlaceObject('conveyer_belt.PLY', [-1.65,0,0.92]);
conveyer_exit = PlaceObject('conveyer_belt.PLY', [-1.65,-1.5,0.92]);

oven = PlaceObject('pizzaoven.PLY', [0,0,0.91]);