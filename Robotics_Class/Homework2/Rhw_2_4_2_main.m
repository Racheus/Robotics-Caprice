clear;
clc;
addpath(genpath('.'));

angles = deg2rad([10,20,30]);
R = eul2rotm(angles,'ZYX');

% 提取旋转矩阵中的元素
r11 = R(1,1);
r12 = R(1,2);
r13 = R(1,3);
r21 = R(2,1);
r22 = R(2,2);
r23 = R(2,3);
r31 = R(3,1);
r32 = R(3,2);
r33 = R(3,3);

% 计算绕 Z 轴的欧拉角
theta_z = atan2(r21, r11);

% 计算绕 Y 轴的欧拉角
phi_y = atan2(-r31,sqrt(r11^2 + r21^2));

% 计算绕 X 轴的欧拉角
psi_x = atan2(r32, r33);

% 将弧度转换为角度
theta_z_deg = rad2deg(theta_z);
phi_y_deg = rad2deg(phi_y);
psi_x_deg = rad2deg(psi_x);

fprintf('ZYX 欧拉角为：[%.4f, %.4f, %.4f]\n', theta_z_deg, phi_y_deg, psi_x_deg);

rmpath(genpath('.'));