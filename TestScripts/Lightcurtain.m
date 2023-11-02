%% Light Curtain
clc 
close all
% Define the dimensions of the cube
dimensions = [3.2, 4, 1]; % Length, Width, Height

% Define vertices using combinations of dimensions
X = [0, dimensions(1), dimensions(1), 0, 0, dimensions(1), dimensions(1), 0];
Y = [0, 0, dimensions(2), dimensions(2), 0, 0, dimensions(2), dimensions(2)];
Z = [0, 0, 0, 0, dimensions(3), dimensions(3), dimensions(3), dimensions(3)];

X = X - dimensions(1)/2;
Y = Y - dimensions(2)/2;

% Rearrange the coordinates for vertices
vertices = [X', Y', Z'];

% Define the faces of the cube
faces = [
    1, 2, 6, 5; % front
    2, 3, 7, 6; % right
    3, 4, 8, 7; % back
    4, 1, 5, 8; % left
    1, 2, 3, 4; % bottom
    5, 6, 7, 8  % top
];

% Create the patch object for the cube
cube = patch('Faces', faces, 'Vertices', vertices, 'FaceColor', 'g', 'FaceAlpha', 0.2);

hold on

% Set the cube's visibility to off
set(cube, 'Visible', 'off');

% Define the coordinates of the object
baby_loc = [1.6, 1, 0];  % Change these values for the object's coordinates

% baby = PlaceObject('baby.ply', baby_loc);
% environment = PlaceObject('environment.ply', [0, 0 , 0]);

% Check if the object is inside the cube
inside_curtain = all(baby_loc >= [-dimensions(1)/2, -dimensions(2)/2, 0]) && all(baby_loc <= dimensions/2);

if inside_curtain
    curtain_detect = 1;
    disp('A person is within the hazard area')
else
    curtain_detect = 0;
    disp('The hazard area is clear')
end