
% Define the dimensions of the cube
dimensions = [3, 2, 4]; % Length, Width, Height

% Define vertices using combinations of dimensions
X = [0, dimensions(1), dimensions(1), 0, 0, dimensions(1), dimensions(1), 0];
Y = [0, 0, dimensions(2), dimensions(2), 0, 0, dimensions(2), dimensions(2)];
Z = [0, 0, 0, 0, dimensions(3), dimensions(3), dimensions(3), dimensions(3)];

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
cube = patch('Faces', faces, 'Vertices', vertices);

% Set the cube's visibility to off
set(cube, 'Visible', 'off');

% Define the coordinates of the object
baby_loc = [1.5, 2, 3];  % Change these values for the object's coordinates

baby = PlaceObject('baby.ply', baby_loc);

% Check if the object is inside the cube
is_inside = (baby_loc(:,1) >= 0 && baby_loc(:,1) <= dimensions(1) && ...
             baby_loc(:,2) >= 0 && baby_loc(:,2) <= dimensions(2) && ...
             baby_loc(:,3) >= 0 && baby_loc(:,3) <= dimensions(3));

if is_inside
    disp('The object is inside the area.');
else
    disp('The object is outside the area.');
end

% r = UR3;
% q = [0,0,0];
% 
% for i = 1:length(links)
%     L = links(1,i);
% 
%     current_transform = linkTransforms(:,:, i);
% 
%     current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
%     transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
%     transforms(:,:,i + 1) = current_transform;
% 
%     linkCentres(i,1) = centreOffset(i,1) + centreTr(3,1);
%     linkCentres(i,2) = centreOffset(i,2) + centreTr(3,2);
%     linkCentres(i,3) = centreOffset(i,3) + centreTr(3,3);
% end

