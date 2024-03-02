% ME3403-01 , Robotics Homework 1-3
% 赵四维 , 521021910696
% Copyright 2023-2024-2 Spring Racheus Zhao

clear;
clc;
addpath(genpath('.'))

disp("======(3)Inverse Solution of 2R arms========");
x1=input("please input position x :");
y1=input("please input position y :");
phi1=input("please input position phi (rad):");
p1=[x1,y1,phi1];
D = (x1*x1+y1*y1-2*2-1*1)/(2*2*1);
theta2_1 = atan(sqrt(1-D*D)/D);
theta2_2 = atan(-sqrt(1-D*D)/D);
theta1_1 = atan(y1/x1)-atan(sin(theta2_1)/(2+cos(theta2_1)));
theta1_2 = atan(y1/x1)-atan(sin(theta2_2)/(2+cos(theta2_2)));
q1=[theta1_1,theta2_1];
q2=[theta1_2,theta2_2];
if (theta1_1+theta1_2)==phi1
    q_ans=[theta1_1;theta1_2];
    disp("Solution of q:");
    disp(q_ans);

else if (theta1_2+theta2_2)==phi1
    q_ans=[theta1-2;theta2_2];
    disp("Solution of q:");
    disp(q1);
else
    disp("Error : The q matrix does not exist!");
end
end
rmpath(genpath('.'));