clear all
close all
clc

robot = UR3;
q = [0, 0, 0, 0, 0, 0];

% get transforms
tr = zeros(4,4,robot.model.n+1);
tr(:,:,1) = robot.model.base;
L = robot.model.links;
for i = 1 : robot.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha)
end

% Calculate midpoints
midpoints = zeros(3, robot.model.n);
for i = 1:robot.model.n
    midpoint_transform = (tr(:, :, i) + tr(:, :, i + 1)) / 2;
    midpoints(:, i) = midpoint_transform(1:3, 4);
end

% Display midpoints
disp("Midpoints of each link:");
disp(midpoints);

hold on;

for i = 1:size(tr, 3)
    % Get the midpoint of the segment
    midpoint = tr(1:3, 4, i);

    % Extract scaling factors from the transformation matrices
    scaling_factors = vecnorm(tr(1:3, 1:3, i));

    % Define radii for the ellipsoid based on scaling factors
    radius_x = 0.2 * scaling_factors(1); 
    radius_y = 0.15 * scaling_factors(2); 
    radius_z = 0.1 * scaling_factors(3); 

    % Generate points on the unit ellipsoid
    [x, y, z] = ellipsoid(0, 0, 0, 1, 1, 1); % Unit ellipsoid

    % Scale the ellipsoid according to the extracted radii
    x = x * radius_x;
    y = y * radius_y;
    z = z * radius_z;

    % Apply the transformation to the ellipsoid points
    ellipsoid_points = tr(1:3, 1:3, i) * [x(:)'; y(:)'; z(:)'];
    ellipsoid_points = reshape(ellipsoid_points', size(x, 1), size(x, 2), 3);

    % Translate and reposition the ellipsoid to the midpoint
    x = ellipsoid_points(:, :, 1) + midpoint(1);
    y = ellipsoid_points(:, :, 2) + midpoint(2);
    z = ellipsoid_points(:, :, 3) + midpoint(3);

    % Plot the ellipsoid
    surf(x, y, z, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

%% another version
% hold on;
% 
% % Plot ellipsoids representing the links
% for i = 1:size(midpoints, 2) - 1 % Iterate through each link segment
%     % Calculate the distance and direction between consecutive midpoints
%     segment_vector = midpoints(:, i + 1) - midpoints(:, i);
%     segment_length = norm(segment_vector);
%     segment_direction = segment_vector / segment_length;
% 
%     % Extract the scaling factors (dimensions) from the transformation matrices
%     scaling_factors = tr(1:3, 1:3, i);
% 
%     % Use the scaling factors to determine the size of the ellipsoid
%     scaling_factor = norm(scaling_factors * [1; 1; 1]);
% 
%     midpoint = (midpoints(:, i) + midpoints(:, i + 1)) / 2; % Midpoint of the segment
% 
%     % Generate a unit sphere
%     [x, y, z] = sphere(20);
% 
%     % Scale and transform the sphere to form an ellipsoid
%     x = x * (scaling_factor / 2) * segment_length + midpoint(1);
%     y = y * (scaling_factor / 2) * segment_length + midpoint(2);
%     z = z * (scaling_factor / 2) * segment_length + midpoint(3);
% 
%     % Plot the ellipsoid
%     surf(x, y, z, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
% end

%% another version
% radiusX = 0.15; % Change the radius value according to your robot's dimensions (X-axis)
% radiusY = 0.1; % Change the radius value according to your robot's dimensions (Y-axis)
% radiusZ = 0.4; % Change the radius value according to your robot's dimensions (Z-axis)
% 
% hold on;
% 
% % Plot ellipsoids representing the links
% for i = 1:size(midpoints, 2)
%     midpoint = midpoints(:, i);
% 
%     % Generate a sphere
%     [x, y, z] = sphere(20);
% 
%     % Scale and transform the sphere to form an ellipsoid
%     x = x * radiusX + midpoint(1);
%     y = y * radiusY + midpoint(2);
%     z = z * radiusZ + midpoint(3);
% 
%     % Plot the ellipsoid
%     surf(x, y, z, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
% end
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');