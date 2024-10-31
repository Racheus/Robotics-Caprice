clear;
clc;
addpath(genpath('.'));

R1 = rotx(deg2rad(60));
disp("R1 is ");
disp(R1);

R2 = rotz(deg2rad(60));
disp("R2 is ");
disp(R2);

R3 = rotz(deg2rad(90));
disp("R3 is ");
disp(R3);

R_final = R3 * R1 * R2;
disp("R_fianl is ");
disp(R_final);

rmpath(genpath('.'));