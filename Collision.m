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
% Provided transformation matrices 'tr'

hold on;

%% Placing prisms

for i = 1:size(tr, 3)-1
    % Calculate the midpoint as the position for the ellipsoid
    midpoint = (tr(1:3, 4, i) + tr(1:3, 4, i+1)) / 2;

    % Calculate the size of the ellipsoid using the start and end transforms
    start_point = tr(1:3, 4, i);
    end_point = tr(1:3, 4, i+1);
    a = 1.05 * norm(end_point - start_point) / 2; % Increase semi-major axis by 5%
    b = 0.05; % Semi-minor axis
    c = 0.05; % Another semi-minor axis (ellipsoid can be oriented differently)

    % Create a mesh grid to represent an ellipsoid
    [x, y, z] = ellipsoid(0, 0, 0, a, b, c, 20); % Use ellipsoid function

    % Transform the ellipsoid vertices to the midpoint
    ellipsoid_vertices = [x(:) y(:) z(:)]; % Vertices as rows
    transformed_vertices = (tr(1:3, 1:3, i) * ellipsoid_vertices')' + midpoint';

    % Reshape the vertices back into the ellipsoid structure
    x_new = reshape(transformed_vertices(:, 1), size(x));
    y_new = reshape(transformed_vertices(:, 2), size(y));
    z_new = reshape(transformed_vertices(:, 3), size(z));

    % Plot the ellipsoid at the midpoint
    surf(x_new, y_new, z_new, 'FaceColor', 'g', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

xlabel('X');
ylabel('Y');
zlabel('Z');

%% detecting collision

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