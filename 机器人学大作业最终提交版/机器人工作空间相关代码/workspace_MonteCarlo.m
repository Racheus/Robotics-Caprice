clear all;
% Define joint limits
L1.qlim = [(-170/180)*pi, (170/180)*pi];
L2.qlim = [(-100/180)*pi, (135/180)*pi];
L3.qlim = [(-200/180)*pi, (70/180)*pi];
L4.qlim = [(-270/180)*pi, (270/180)*pi];
L5.qlim = [(-128/180)*pi, (128/180)*pi];
L6.qlim = [(-360/180)*pi, (360/180)*pi];

% Define theta min and max values
theta1min = -170; theta1max = 170;
theta2min = -100; theta2max = 135;
theta3min = -200; theta3max = 70;
theta4min = -270; theta4max = 270;
theta5min = -128; theta5max = 128;
theta6min = -360; theta6max = 360;

% Number of samples
n = 400000;
x = zeros(1, n);
y = zeros(1, n);
z = zeros(1, n);

% Generate random theta values and compute end effector positions
for i = 1:n
    theta1 = theta1min * (pi/180) + (theta1max - theta1min) * (pi/180) * rand;
    theta2 = theta2min * (pi/180) + (theta2max - theta2min) * (pi/180) * rand;
    theta3 = theta3min * (pi/180) + (theta3max - theta3min) * (pi/180) * rand;
    theta4 = theta4min * (pi/180) + (theta4max - theta4min) * (pi/180) * rand;
    theta5 = theta5min * (pi/180) + (theta5max - theta5min) * (pi/180) * rand;
    theta6 = theta6min * (pi/180) + (theta6max - theta6min) * (pi/180) * rand;

    Tws = MODtransmatrix(theta1, theta2, theta3, theta4, theta5, theta6);
    x(i) = Tws(1, 4);
    y(i) = Tws(2, 4);
    z(i) = Tws(3, 4);
end

% Plot the results
figure('color', [1 1 1]);
plot3(x, y, z, 'b.', 'MarkerSize', 1.5);
hold on;

xlabel('x轴(millimeter)', 'color', 'k', 'fontsize', 15);
ylabel('y轴(millimeter)', 'color', 'k', 'fontsize', 15);
zlabel('z轴(millimeter)', 'color', 'k', 'fontsize', 15);
% 计算凸包
[K, V] = convhull(x, y, z);

% 显示凸包的体积
disp(['Convex Hull Volume: ', num2str(V)]);
title('3D Workspace');
axis equal;
grid on;
% 设置 x, y, z 轴的范围
xlim([-1200 1200]);
ylim([-1200 1200]);
zlim([-600 1400]);
hold off;

% Function definition
function [T06] = MODtransmatrix(theta1, theta2, theta3, theta4, theta5, theta6)
    % 连杆偏移
    d1 = 380;
    d2 = 0;
    d3 = 0;
    d4 = 340;
    d5 = 0;
    d6 = 270;
    % 连杆长度
    a1 = 0;
    a2 = 0;
    a3 = 350;
    a4 = 0;
    a5 = 0;
    a6 = 0;
    % 连杆扭角
    alpha1 = 0;
    alpha2 = -pi/2;
    alpha3 = 0;
    alpha4 = -pi/2;
    alpha5 = pi/2;
    alpha6 = -pi/2;

    MDH = [theta1 d1 a1 alpha1;
           theta2 d2 a2 alpha2;
           theta3-pi/2 d3 a3 alpha3;
           theta4 d4 a4 alpha4;
           theta5 d5 a5 alpha5;
           theta6 d6 a6 alpha6];
  
    T01 = [cos(MDH(1, 1)), -sin(MDH(1, 1)), 0, MDH(1, 3);
           sin(MDH(1, 1)) * cos(MDH(1, 4)), cos(MDH(1, 1)) * cos(MDH(1, 4)), -sin(MDH(1, 4)), -sin(MDH(1, 4)) * MDH(1, 2);
           sin(MDH(1, 1)) * sin(MDH(1, 4)), cos(MDH(1, 1)) * sin(MDH(1, 4)), cos(MDH(1, 4)), cos(MDH(1, 4)) * MDH(1, 2);
           0, 0, 0, 1];
    T12 = [cos(MDH(2, 1)), -sin(MDH(2, 1)), 0, MDH(2, 3);
           sin(MDH(2, 1)) * cos(MDH(2, 4)), cos(MDH(2, 1)) * cos(MDH(2, 4)), -sin(MDH(2, 4)), -sin(MDH(2, 4)) * MDH(2, 2);
           sin(MDH(2, 1)) * sin(MDH(2, 4)), cos(MDH(2, 1)) * sin(MDH(2, 4)), cos(MDH(2, 4)), cos(MDH(2, 4)) * MDH(2, 2);
           0, 0, 0, 1];
    T23 = [cos(MDH(3, 1)), -sin(MDH(3, 1)), 0, MDH(3, 3);
           sin(MDH(3, 1)) * cos(MDH(3, 4)), cos(MDH(3, 1)) * cos(MDH(3, 4)), -sin(MDH(3, 4)), -sin(MDH(3, 4)) * MDH(3, 2);
           sin(MDH(3, 1)) * sin(MDH(3, 4)), cos(MDH(3, 1)) * sin(MDH(3, 4)), cos(MDH(3, 4)), cos(MDH(3, 4)) * MDH(3, 2);
           0, 0, 0, 1];
    T34 = [cos(MDH(4, 1)), -sin(MDH(4, 1)), 0, MDH(4, 3);
           sin(MDH(4, 1)) * cos(MDH(4, 4)), cos(MDH(4, 1)) * cos(MDH(4, 4)), -sin(MDH(4, 4)), -sin(MDH(4, 4)) * MDH(4, 2);
           sin(MDH(4, 1)) * sin(MDH(4, 4)), cos(MDH(4, 1)) * sin(MDH(4, 4)), cos(MDH(4, 4)), cos(MDH(4, 4)) * MDH(4, 2);
           0, 0, 0, 1];
    T45 = [cos(MDH(5, 1)), -sin(MDH(5, 1)), 0, MDH(5, 3);
           sin(MDH(5, 1)) * cos(MDH(5, 4)), cos(MDH(5, 1)) * cos(MDH(5, 4)), -sin(MDH(5, 4)), -sin(MDH(5, 4)) * MDH(5, 2);
           sin(MDH(5, 1)) * sin(MDH(5, 4)), cos(MDH(5, 1)) * sin(MDH(5, 4)), cos(MDH(5, 4)), cos(MDH(5, 4)) * MDH(5, 2);
           0, 0, 0, 1];
    T56 = [cos(MDH(6, 1)), -sin(MDH(6, 1)), 0, MDH(6, 3);
           sin(MDH(6, 1)) * cos(MDH(6, 4)), cos(MDH(6, 1)) * cos(MDH(6, 4)), -sin(MDH(6, 4)), -sin(MDH(6, 4)) * MDH(6, 2);
           sin(MDH(6, 1)) * sin(MDH(6, 4)), cos(MDH(6, 1)) * sin(MDH(6, 4)), cos(MDH(6, 4)), cos(MDH(6, 4)) * MDH(6, 2);
           0, 0, 0, 1];
    T06 = T01 * T12 * T23 * T34 * T45 * T56;
end
