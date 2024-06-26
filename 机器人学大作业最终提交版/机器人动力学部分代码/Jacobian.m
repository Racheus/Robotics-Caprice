clear all; 
clc; 
% 定义机器人的六个关节的DH参数表
L(1)= Link([0 0.38 0 0],'modified');
L(2)= Link([0 0 0 -pi/2],'modified');
L(3)= Link([0 0 0.35 0],'modified'); L(3).offset = -pi/2;
L(4)= Link([0 0.34 0 -pi/2],'modified');
L(5)= Link([0 0 0 pi/2],'modified');
L(6)= Link([0 0.27 0 -pi/2],'modified');

% 创建机器人对象
robot = SerialLink(L, 'name', 'six_link');

%%
% 初始关节角状态下，使用robotics toolbox自带函数求解雅各比矩阵
Q = [0 0 0 0 0 0];
jacobin = robot.jacob0(Q);   %jacob0函数代表相对于基坐标系的雅各比矩阵
jacobin

%%
% 初始关节角状态下，使用矢量积法公式计算推导雅各比矩阵
syms theta1 theta2 theta3 theta4 theta5 theta6
theta1 = 0;theta2 = 0;theta3 = -pi/2; theta4 = 0; theta5 = 0; theta6 = 0;  %theta是角度制
T01 = trans(theta1, 0.38, 0, 0); 
T12 = trans(theta2, 0, 0, -pi/2);
T23 = trans(theta3, 0, 0.35, 0);
T34 = trans(theta4, 0.34, 0, -pi/2);
T45 = trans(theta5, 0, 0, pi/2);
T56 = trans(theta6, 0.27, 0, -pi/2);
T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;
T06 = T01*T12*T23*T34*T45*T56;
P01 = [T01(1,4);T01(2,4);T01(3,4)];
P02 = [T02(1,4);T02(2,4);T02(3,4)];
P03 = [T03(1,4);T03(2,4);T03(3,4)];
P04 = [T04(1,4);T04(2,4);T04(3,4)];
P05 = [T05(1,4);T05(2,4);T05(3,4)];
P06 = [T06(1,4);T06(2,4);T06(3,4)];
z1 = T01(1:3,3);
z2 = T02(1:3,3);
z3 = T03(1:3,3);
z4 = T04(1:3,3);
z5 = T05(1:3,3);
z6 = T06(1:3,3);
j1 = [cross(z1,P06-P01);z1];
j2 = [cross(z2,P06-P02);z2];
j3 = [cross(z3,P06-P03);z3];
j4 = [cross(z4,P06-P04);z4];
j5 = [cross(z5,P06-P05);z5];
j6 = [cross(z6,P06-P06);z6];
jacobin1 = [j1,j2,j3,j4,j5,j6];  %公式计算得到相对于基坐标系的雅各比矩阵
jacobin1

function T = trans(theta, d, a, alpha)
T = [cos(theta),-sin(theta),0,a;
sin(theta)*cos(alpha), cos(alpha)*cos(theta),-sin(alpha),-d*sin(alpha);
sin(theta)*sin(alpha), sin(alpha)*cos(theta), cos(alpha), d*cos(alpha);
0, 0, 0, 1];
end

%对比jacobin和jacobin1的结果，几乎相同
