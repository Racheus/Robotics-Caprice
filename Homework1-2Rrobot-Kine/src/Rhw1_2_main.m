% ME3403-01 , Robotics Homework 1-2
% 赵四维 , 521021910696
% Copyright 2023-2024-2 Spring Racheus Zhao

clear;
clc;
addpath(genpath('.'))

% Create a 2R robot model
L1 = Link('d', 0, 'a', 2, 'alpha', 0);
L2 = Link('d', 0, 'a', 1, 'alpha', 0);
robot = SerialLink([L1 L2], 'name', 'ME3403-2Rrobot');

theta1 = input("Please input theta1 (rad) (eg:pi/6):");
theta2 = input("Please input theta2 (rad) (eg:pi/6):");

x = 2*cos(theta1) + cos(theta1+theta2);
y = 2*sin(theta1) + sin(theta1+theta2);
phi = theta1 + theta2;

disp("末端位姿矩阵：")
disp([x,y,phi]);

J = [-2*sin(theta1)-sin(theta1+theta2),-sin(theta1+theta2);
    2*cos(theta1)+cos(theta1+theta2),cos(theta1+theta2);
    1,1];

disp("Jacobian 矩阵：")
disp(J);

robot.plot([theta1,theta2]);
view(0, 90); 
rmpath(genpath('.'));