clear;
clc;
addpath(genpath('.'));

% Example homogeneous transformation matrix
T = [
    0.866, -0.5, 0, 1;
    0.5, 0.866, 0, 2;
    0, 0, 1, 3;
    0, 0, 0, 1
];

% Self-Solution
T_inv_custom = inverse_homogeneous_transform(T);

% MATLAB-inv-function
T_inv_builtin = inv(T);

% 比较结果
disp('Custom Inverse:');
disp(T_inv_custom);
disp('Built-in Inverse:');
disp(T_inv_builtin);



% 比较运行时间
num_iterations = 10000;
tic;
for i = 1:num_iterations
    T_inv_custom = inverse_homogeneous_transform(T);
end
custom_time = toc;

tic;
for i = 1:num_iterations
    T_inv_builtin = inv(T);
end
builtin_time = toc;

disp('Custom Function Time:');
disp(custom_time);
disp('Built-in Function Time:');
disp(builtin_time);

rmpath(genpath('.'));
function T_inv = inverse_homogeneous_transform(T)

    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    % Rotation matrix inv
    R_inv = R';
    
    % inverse of translation part
    p_inv = -R_inv * p;
    
    % Remake a new homogeneous transformation matrix
    T_inv = eye(4);
    T_inv(1:3, 1:3) = R_inv;
    T_inv(1:3, 4) = p_inv;
end
