clear;
clc;
addpath(genpath('.'));

R = [0 0 1; -1 0 0; 0 -1 0]; 
axang = rotm2axang(R);
disp("Axis:")
disp(axang(1:3));
disp("Angle：")
disp(rad2deg(axang(4)));

R_prime = [0 0 1 1;-1 0 0 2;0 -1 0 3;0 0 0 1];
p_A = [1 3 5 1]';
disp('p_B的表达为')
disp(R_prime * p_A);
rmpath(genpath('.'));