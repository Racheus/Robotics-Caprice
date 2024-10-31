clear;
clc;
addpath(genpath('.'));

angles = deg2rad([10,20,30]);
R = eul2rotm(angles,'ZYX');
disp("Rotation Matrix : ")
disp(R);

R1 = rotx(angles(3));
R2 = roty(angles(2));
R3 = rotz(angles(1));

R_prime = R3*R2*R1 ;
disp("右乘得到的结果：")
disp(R_prime);

angle_new = rotm2eul(R_prime,"XYZ");
ans = rad2deg(angle_new);
disp("按照XYZ分解的结果 ： ")
disp(ans);

rmpath(genpath('.'));