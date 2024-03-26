clear;
clc;
addpath(genpath('.'));

T=[-0.2357, -0.912, -0.3358, 0.4218; 
    0.8894 ,-0.3417 ,0.3037 ,-0.007691;
    -0.3918, -0.2271, 0.8916, 18.92;
    0, 0 ,0, 1];

J_ans = MODikine(T);
disp("解析解：")
disp(J_ans);


L1 = Link([0,5,0,0,0],'modified');
L2 = Link([0,0,0,-pi/2,0],'modified');
L3 = Link([pi/2, 0, 5, 0],'modified');
L4 = Link([0,6,0,-pi/2,0],'modified');
L5 = Link([0,0,0,pi/2,0],'modified');
L6 = Link([0,5,0,-pi/2,0],'modified');

L3.offset = -pi/2;

SixRrobot = SerialLink([L1,L2,L3,L4,L5,L6],'name','ME3403-6Rrobot');
q=SixRrobot.ikine(T);
disp("数值解：")
disp(q);
disp("必须承认，二者不一样，助教老师对不起。")

rmpath(genpath('.'));
function [J] = MODikine(a)
d1 = 5;
d2 = 0;
d3 = 0;
d4 = 6;
d5 = 0;
d6 = 5;
a1 = 0;
a2 = 0;
a3 = 5;
a4 = 0;
a5 = 0;
a6 = 0;
alpha1 = 0;
alpha2 = -pi/2;
alpha3 = 0;
alpha4 = -pi/2;
alpha5 = pi/2;
alpha6 = -pi/2;
    
%%
nx=a(1,1);ox=a(1,2);ax=a(1,3);px=a(1,4);
ny=a(2,1);oy=a(2,2);ay=a(2,3);py=a(2,4);
nz=a(3,1);oz=a(3,2);az=a(3,3);pz=a(3,4);
    
% 求解关节角1
	theta1_1 = atan2(py,px)-atan2(d2,sqrt(abs(px^2+py^2-d2^2)));
	theta1_2 = atan2(py,px)-atan2(d2,-sqrt(abs(px^2+py^2-d2^2)));
	
% 求解关节角3
	m3_1 = (px^2+py^2+pz^2-a2^2-a3^2-d2^2-d4^2)/(2*a2);
	theta3_1 = atan2(a3,d4)-atan2(m3_1,sqrt(abs(a3^2+d4^2-m3_1^2)));
	theta3_2 = atan2(a3,d4)-atan2(m3_1,-sqrt(abs(a3^2+d4^2-m3_1^2)));
	
% 求解关节角2
    ms2_1 = -((a3+a2*cos(theta3_1))*pz)+(cos(theta1_1)*px+sin(theta1_1)*py)*(a2*sin(theta3_1)-d4);
    mc2_1 = (-d4+a2*sin(theta3_1))*pz+(cos(theta1_1)*px+sin(theta1_1)*py)*(a2*cos(theta3_1)+a3);
    theta23_1 = atan2(ms2_1,mc2_1);
    theta2_1 = theta23_1 - theta3_1;
    
    ms2_2 = -((a3+a2*cos(theta3_1))*pz)+(cos(theta1_2)*px+sin(theta1_2)*py)*(a2*sin(theta3_1)-d4);
    mc2_2 = (-d4+a2*sin(theta3_1))*pz+(cos(theta1_2)*px+sin(theta1_2)*py)*(a2*cos(theta3_1)+a3);
    theta23_2 = atan2(ms2_2,mc2_2);
    theta2_2 = theta23_2 - theta3_1;
    
    ms2_3 = -((a3+a2*cos(theta3_2))*pz)+(cos(theta1_1)*px+sin(theta1_1)*py)*(a2*sin(theta3_2)-d4);
    mc2_3 = (-d4+a2*sin(theta3_2))*pz+(cos(theta1_1)*px+sin(theta1_1)*py)*(a2*cos(theta3_2)+a3);
    theta23_3 = atan2(ms2_3,mc2_3);
    theta2_3 = theta23_3 - theta3_2;

    ms2_4 = -((a3+a2*cos(theta3_2))*pz)+(cos(theta1_2)*px+sin(theta1_2)*py)*(a2*sin(theta3_2)-d4);
    mc2_4 = (-d4+a2*sin(theta3_2))*pz+(cos(theta1_2)*px+sin(theta1_2)*py)*(a2*cos(theta3_2)+a3);
    theta23_4 = atan2(ms2_4,mc2_4);
    theta2_4 = theta23_4 - theta3_2;
        
% 求解关节角4
    ms4_1=-ax*sin(theta1_1)+ay*cos(theta1_1);
    mc4_1=-ax*cos(theta1_1)*cos(theta23_1)-ay*sin(theta1_1)*cos(theta23_1)+az*sin(theta23_1);
	theta4_1=atan2(ms4_1,mc4_1);
	
	ms4_2=-ax*sin(theta1_2)+ay*cos(theta1_2);
    mc4_2=-ax*cos(theta1_2)*cos(theta23_2)-ay*sin(theta1_2)*cos(theta23_2)+az*sin(theta23_2);
	theta4_2=atan2(ms4_2,mc4_2);
	
	ms4_3=-ax*sin(theta1_1)+ay*cos(theta1_1);
    mc4_3=-ax*cos(theta1_1)*cos(theta23_3)-ay*sin(theta1_1)*cos(theta23_3)+az*sin(theta23_3);
	theta4_3=atan2(ms4_3,mc4_3);
	
	ms4_4=-ax*sin(theta1_2)+ay*cos(theta1_2);
    mc4_4=-ax*cos(theta1_2)*cos(theta23_4)-ay*sin(theta1_2)*cos(theta23_4)+az*sin(theta23_4);
	theta4_4=atan2(ms4_4,mc4_4);
	
% 求解关节角5
	ms5_1=-ax*(cos(theta1_1)*cos(theta23_1)*cos(theta4_1)+sin(theta1_1)*sin(theta4_1))-ay*(sin(theta1_1)*cos(theta23_1)*cos(theta4_1)-cos(theta1_1)*sin(theta4_1))+az*(sin(theta23_1)*cos(theta4_1));
    mc5_1= ax*(-cos(theta1_1)*sin(theta23_1))+ay*(-sin(theta1_1)*sin(theta23_1))+az*(-cos(theta23_1));
    theta5_1=atan2(ms5_1,mc5_1);
    
	ms5_2=-ax*(cos(theta1_2)*cos(theta23_2)*cos(theta4_2)+sin(theta1_2)*sin(theta4_2))-ay*(sin(theta1_2)*cos(theta23_2)*cos(theta4_2)-cos(theta1_2)*sin(theta4_2))+az*(sin(theta23_2)*cos(theta4_2));
    mc5_2= ax*(-cos(theta1_2)*sin(theta23_2))+ay*(-sin(theta1_2)*sin(theta23_2))+az*(-cos(theta23_2));
    theta5_2=atan2(ms5_2,mc5_2);
  
	ms5_3=-ax*(cos(theta1_1)*cos(theta23_3)*cos(theta4_3)+sin(theta1_1)*sin(theta4_3))-ay*(sin(theta1_1)*cos(theta23_3)*cos(theta4_3)-cos(theta1_1)*sin(theta4_3))+az*(sin(theta23_3)*cos(theta4_3));
    mc5_3= ax*(-cos(theta1_1)*sin(theta23_3))+ay*(-sin(theta1_1)*sin(theta23_3))+az*(-cos(theta23_3));
    theta5_3=atan2(ms5_3,mc5_3);
    
	ms5_4=-ax*(cos(theta1_2)*cos(theta23_4)*cos(theta4_4)+sin(theta1_2)*sin(theta4_4))-ay*(sin(theta1_2)*cos(theta23_4)*cos(theta4_4)-cos(theta1_2)*sin(theta4_4))+az*(sin(theta23_4)*cos(theta4_4));
    mc5_4= ax*(-cos(theta1_2)*sin(theta23_4))+ay*(-sin(theta1_2)*sin(theta23_4))+az*(-cos(theta23_4));
    theta5_4=atan2(ms5_4,mc5_4);
    
% 求解关节角6
	ms6_1=-nx*(cos(theta1_1)*cos(theta23_1)*sin(theta4_1)-sin(theta1_1)*cos(theta4_1))-ny*(sin(theta1_1)*cos(theta23_1)*sin(theta4_1)+cos(theta1_1)*cos(theta4_1))+nz*(sin(theta23_1)*sin(theta4_1));
    mc6_1= nx*(cos(theta1_1)*cos(theta23_1)*cos(theta4_1)+sin(theta1_1)*sin(theta4_1))*cos(theta5_1)-nx*cos(theta1_1)*sin(theta23_1)*sin(theta4_1)+ny*(sin(theta1_1)*cos(theta23_1)*cos(theta4_1)+cos(theta1_1)*sin(theta4_1))*cos(theta5_1)-ny*sin(theta1_1)*sin(theta23_1)*sin(theta5_1)-nz*(sin(theta23_1)*cos(theta4_1)*cos(theta5_1)+cos(theta23_1)*sin(theta5_1));
	theta6_1=atan2(ms6_1,mc6_1);
	
	ms6_2=-nx*(cos(theta1_2)*cos(theta23_2)*sin(theta4_2)-sin(theta1_2)*cos(theta4_2))-ny*(sin(theta1_2)*cos(theta23_2)*sin(theta4_2)+cos(theta1_2)*cos(theta4_2))+nz*(sin(theta23_2)*sin(theta4_2));
    mc6_2= nx*(cos(theta1_2)*cos(theta23_2)*cos(theta4_2)+sin(theta1_2)*sin(theta4_2))*cos(theta5_2)-nx*cos(theta1_2)*sin(theta23_2)*sin(theta4_2)+ny*(sin(theta1_2)*cos(theta23_2)*cos(theta4_2)+cos(theta1_2)*sin(theta4_2))*cos(theta5_2)-ny*sin(theta1_2)*sin(theta23_2)*sin(theta5_2)-nz*(sin(theta23_2)*cos(theta4_2)*cos(theta5_2)+cos(theta23_2)*sin(theta5_2));
	theta6_2=atan2(ms6_2,mc6_2);
	
	ms6_3=-nx*(cos(theta1_1)*cos(theta23_3)*sin(theta4_3)-sin(theta1_1)*cos(theta4_3))-ny*(sin(theta1_1)*cos(theta23_3)*sin(theta4_3)+cos(theta1_1)*cos(theta4_3))+nz*(sin(theta23_3)*sin(theta4_3));
    mc6_3= nx*(cos(theta1_1)*cos(theta23_3)*cos(theta4_3)+sin(theta1_1)*sin(theta4_3))*cos(theta5_3)-nx*cos(theta1_1)*sin(theta23_3)*sin(theta4_3)+ny*(sin(theta1_1)*cos(theta23_3)*cos(theta4_3)+cos(theta1_1)*sin(theta4_3))*cos(theta5_3)-ny*sin(theta1_1)*sin(theta23_3)*sin(theta5_3)-nz*(sin(theta23_3)*cos(theta4_3)*cos(theta5_3)+cos(theta23_3)*sin(theta5_3));
	theta6_3=atan2(ms6_3,mc6_3);
	
	ms6_4=-nx*(cos(theta1_2)*cos(theta23_4)*sin(theta4_4)-sin(theta1_2)*cos(theta4_4))-ny*(sin(theta1_1)*cos(theta23_4)*sin(theta4_4)+cos(theta1_2)*cos(theta4_4))+nz*(sin(theta23_4)*sin(theta4_4));
    mc6_4= nx*(cos(theta1_2)*cos(theta23_4)*cos(theta4_4)+sin(theta1_2)*sin(theta4_4))*cos(theta5_4)-nx*cos(theta1_2)*sin(theta23_4)*sin(theta4_4)+ny*(sin(theta1_2)*cos(theta23_4)*cos(theta4_4)+cos(theta1_2)*sin(theta4_4))*cos(theta5_1)-ny*sin(theta1_2)*sin(theta23_4)*sin(theta5_4)-nz*(sin(theta23_4)*cos(theta4_4)*cos(theta5_4)+cos(theta23_4)*sin(theta5_4));
	theta6_4=atan2(ms6_4,mc6_4);
	
	
% 4组运动学非奇异逆解
theta_MOD1 = [ 
               theta1_1,theta2_1,theta3_1,theta4_1,theta5_1,theta6_1;
 			   theta1_2,theta2_2,theta3_1,theta4_2,theta5_2,theta6_2;
 			   theta1_1,theta2_3,theta3_2,theta4_3,theta5_3,theta6_3;
 			   theta1_2,theta2_4,theta3_2,theta4_4,theta5_4,theta6_4;
              ];
% 翻转
theta_MOD2 = ...
    [ 
      theta1_1,theta2_1,theta3_1,theta4_1+pi,-theta5_1,theta6_1+pi;
 	  theta1_2,theta2_2,theta3_1,theta4_2+pi,-theta5_2,theta6_2+pi;
 	  theta1_1,theta2_3,theta3_2,theta4_3+pi,-theta5_3,theta6_3+pi;
 	  theta1_2,theta2_4,theta3_2,theta4_4+pi,-theta5_4,theta6_4+pi;
    ];
    
  J = [theta_MOD1;theta_MOD2];         
end



