clc;

qlim = [(-170/180)*pi, (170/180)*pi;
    (-100/180)*pi, (135/180)*pi;
    (-200/180)*pi, (70/180)*pi;
    (-270/180)*pi, (270/180)*pi;
    (-128/180)*pi, (128/180)*pi;
    (-360/180)*pi, (360/180)*pi];

n = 10000;
threshold =10;

improvedMonteCarlo1(qlim, n, threshold);

% Main function
function [reachable_workspace, max_positions, min_positions] = improvedMonteCarlo1(qlim, n, threshold)
    % Initialize max and min positions
    max_positions = zeros(1, 3);
    min_positions = zeros(1, 3);

    % Generate seed workspace
    [seed_workspace, max_positions, min_positions] = generateSeedWorkspace(qlim, n);
    seed_points = getAllExpandedPoints(seed_workspace);

    % Expand seed workspace
    expanded_workspace = expandSeedWorkspace(seed_workspace, threshold, qlim, max_positions, min_positions);
    expanded_points = getAllExpandedPoints(expanded_workspace);
    disp(length(expanded_points))
    % Plot workspace
    plotWorkspace(seed_points, expanded_points);
end

% Generate seed workspace
function [seed_workspace, max_positions, min_positions] = generateSeedWorkspace(qlim, n)
    % qlim: Joint angle limits matrix, each row is the limit for a joint
    % n: Number of seed points needed for each subspace

    num_subspaces = 20;  % Divide into 20 subspaces in each dimension

    % Generate random joint angles
    theta = zeros(n, 6);
    for i = 1:6
        theta(:, i) = qlim(i, 1) + (qlim(i, 2) - qlim(i, 1)).* rand(n, 1);
    end

    % Forward kinematics to get positions
    positions = zeros(n, 3);
    for i = 1:n
        Tws = MODtransmatrix(theta(i, :));  % Calculate using forward kinematics function
        positions(i, :) = Tws(1:3, 4)';     % Extract position information
    end

    % Enclose all workspace points in a rectangular prism
    max_positions = max(positions);
    min_positions = min(positions);

    % Calculate range for each dimension
    range = max_positions - min_positions;

    % Calculate subspace size for each dimension
    subspaces_range = range / num_subspaces;

    % Divide into multiple equal subspaces and record seed points in each subspace
    num_points_per_subspace = zeros(num_subspaces, num_subspaces, num_subspaces);
    points_per_subspace = cell(num_subspaces, num_subspaces, num_subspaces);
    joints_per_subspace = cell(num_subspaces, num_subspaces, num_subspaces);

    for i = 1:num_subspaces
        for j = 1:num_subspaces
            for k = 1:num_subspaces
                % Calculate boundaries for current subspace
                sub_min = min_positions + [i-1, j-1, k-1] .* subspaces_range;
                sub_max = min_positions + [i, j, k] .* subspaces_range;

                % Filter seed points within current subspace
                indices = all(positions >= sub_min & positions <= sub_max, 2);
                num_points_per_subspace(i, j, k) = sum(indices);
                points_per_subspace{i, j, k} = positions(indices, :);
                joints_per_subspace{i, j, k} = theta(indices, :);
            end
        end
    end

    seed_workspace.num_points_per_subspace = num_points_per_subspace;
    seed_workspace.points_per_subspace = points_per_subspace;
    seed_workspace.joints_per_subspace = joints_per_subspace;
end

% Expand seed workspace
function expanded_workspace = expandSeedWorkspace(seed_workspace, threshold, qlim, max_positions, min_positions)
    num_subspaces = size(seed_workspace.num_points_per_subspace);
    expanded_workspace = seed_workspace;

    % Initialize counters
    nf = 0; % Joint limit exceed count
    nl = 0; % Workspace limit exceed count

    % Select subspaces to expand
    [sub_i, sub_j, sub_k] = ind2sub(size(seed_workspace.num_points_per_subspace), ...
        find(seed_workspace.num_points_per_subspace < threshold & seed_workspace.num_points_per_subspace > 0));

    % Combine subspace indices into a 3D index array
    subspaces_to_expand = [sub_i, sub_j, sub_k];

    % Calculate subspace range
    range = max_positions - min_positions;
    subspaces_range = range / 20;

    delta = 1.5; % Adjustment factor
    
    i = 1;

    while i <= length(subspaces_to_expand)
        subspace_idx = subspaces_to_expand(i,:);
        sigma = pi / 3; % Initial variance
        while expanded_workspace.num_points_per_subspace(subspace_idx(1), subspace_idx(2), subspace_idx(3)) < threshold
            % Reselect a point for expansion
            num_points_in_subspace = expanded_workspace.num_points_per_subspace(subspace_idx(1), subspace_idx(2), subspace_idx(3));
            point_idx = randi(num_points_in_subspace);

            sub_min = min_positions + [subspace_idx(1)-1, subspace_idx(2)-1, subspace_idx(3)-1] .* subspaces_range;
            sub_max = min_positions + [subspace_idx(1), subspace_idx(2), subspace_idx(3)] .* subspaces_range;

            joint = expanded_workspace.joints_per_subspace{subspace_idx(1), subspace_idx(2), subspace_idx(3)}(point_idx,:);
            new_theta = joint + randn(1, 6) * sigma;
            Tws = MODtransmatrix(new_theta);
            new_position = Tws(1:3, 4)';

            % Determine the subspace index of the new point
            if all(new_theta >= qlim(:, 1)') && all(new_theta <= qlim(:, 2)')
                subspace_idx_new = ceil((new_position - min_positions) ./ subspaces_range);
                 subspace_idx_new = min(subspace_idx_new, [20 20 20]);
                  subspace_idx_new = max(subspace_idx_new, [1 1 1]);
                % Update expanded workspace
                expanded_workspace.joints_per_subspace{subspace_idx_new(1), subspace_idx_new(2), subspace_idx_new(3)} = ...
                    [expanded_workspace.joints_per_subspace{subspace_idx_new(1), subspace_idx_new(2), subspace_idx_new(3)}; new_theta];
                expanded_workspace.points_per_subspace{subspace_idx_new(1), subspace_idx_new(2), subspace_idx_new(3)} = ...
                    [expanded_workspace.points_per_subspace{subspace_idx_new(1), subspace_idx_new(2), subspace_idx_new(3)}; new_position];
                expanded_workspace.num_points_per_subspace(subspace_idx_new(1), subspace_idx_new(2), subspace_idx_new(3)) = ...
                    expanded_workspace.num_points_per_subspace(subspace_idx_new(1), subspace_idx_new(2), subspace_idx_new(3)) + 1;

                % If the subspace originally had no points, adding a new point makes it 1, then add its index to the expansion list
                if expanded_workspace.num_points_per_subspace(subspace_idx_new(1), subspace_idx_new(2), subspace_idx_new(3)) == 1
                    subspaces_to_expand = [subspaces_to_expand; subspace_idx_new];
                end
            end

            % Update counters
            if any(new_theta < qlim(:, 1)') || any(new_theta > qlim(:, 2)')
                nf = nf + 1;
            end
            if any(new_position < sub_min) || any(new_position > sub_max)
                nl = nl + 1;
            end

            % Adjust variance based on counters
            if nf > 5 || nl > 300
                sigma = sigma / delta;
                nf = 0; % Reset counters
                nl = 0;
            end
        end
        i = i + 1;
        disp(i);
    end
end

% Get all expanded points
function all_expanded_points = getAllExpandedPoints(expanded_workspace)
    % Initialize an empty array to store all xyz coordinates
    all_expanded_points = [];

    % Get number of subspaces
    num_subspaces = length(expanded_workspace.points_per_subspace);

    % Traverse each subspace
    for i = 1:size(expanded_workspace.points_per_subspace, 1)
        for j = 1:size(expanded_workspace.points_per_subspace, 2)
            for k = 1:size(expanded_workspace.points_per_subspace, 3)
                % Get all points in the current subspace
                points = expanded_workspace.points_per_subspace{i, j, k};

                % Append the points in the current subspace to the all_expanded_points array
                all_expanded_points = [all_expanded_points; points];
            end
        end
    end
end

% Forward kinematics function
function [Tws] = MODtransmatrix(theta)
    % Link offsets
    d1 = 380;
    d2 = 0;
    d3 = 0;
    d4 = 340;
    d5 = 0;
    d6 = 270;

    % Link lengths
    a1 = 0;
    a2 = 0;
    a3 = 350;
    a4 = 0;
    a5 = 0;
    a6 = 0;

    % Link twists
    alpha1 = 0;
    alpha2 = -pi/2;
    alpha3 = 0;
    alpha4 = -pi/2;
    alpha5 = pi/2;
    alpha6 = -pi/2;

    DH_params = [theta(1) d1 a1 alpha1;
                 theta(2) d2 a2 alpha2;
                 theta(3)-pi/2 d3 a3 alpha3;
                 theta(4) d4 a4 alpha4;
                 theta(5) d5 a5 alpha5;
                 theta(6) d6 a6 alpha6];

    % Compute transformation matrix based on DH parameters
    T01 = dhTransform(DH_params(1, :));
    T12 = dhTransform(DH_params(2, :));
    T23 = dhTransform(DH_params(3, :));
    T34 = dhTransform(DH_params(4, :));
    T45 = dhTransform(DH_params(5, :));
    T56 = dhTransform(DH_params(6, :));

    T06 = T01 * T12 * T23 * T34 * T45 * T56;
    Tws = T06;
end

% DH parameters to transformation matrix function
function T = dhTransform(params)
    d = params(2);
    a = params(3);
    alpha = params(4);
    theta = params(1);

    T = [cos(theta), -sin(theta), 0, a;
         sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d;
         sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d;
         0, 0, 0, 1];
end

% Plot workspace
function plotWorkspace(seed_points, expanded_points)
    figure('color', [1 1 1]);
    hold on;

    % % 绘制种子工作空间点
    % if ~isempty(seed_points)
    %     scatter3(seed_points(:, 1), seed_points(:, 2), seed_points(:, 3), 1.5, 'r', 'filled');
    % end

    % 绘制扩展工作空间点
    if ~isempty(expanded_points)
        scatter3(expanded_points(:, 1), expanded_points(:, 2), expanded_points(:, 3), 1.5, 'b', 'filled');
    end

    xlabel('X (mm)', 'color', 'k', 'fontsize', 15);
    ylabel('Y (mm)', 'color', 'k', 'fontsize', 15);
    zlabel('Z (mm)', 'color', 'k', 'fontsize', 15);
   % 计算凸包
    [K, V] = convhull(expanded_points(:, 1), expanded_points(:, 2), expanded_points(:, 3));

    % 显示凸包的体积
    disp(['Convex Hull Volume: ', num2str(V)]);
    grid on;
    title('3D Workspace');
    view(3);
    axis equal;

    % 设置 x, y, z 轴的范围
    xlim([-1200 1200]);
    ylim([-1200 1200]);
    zlim([-600 1400]);

    hold off;
end

