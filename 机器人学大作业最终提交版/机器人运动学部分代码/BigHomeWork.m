clc
clear all;
addpath(genpath('.'));
%% 定义机器人
L(1)=Link([0,0.38,0,0],'modified'); 
L(2)=Link([0,0,0,-pi/2],'modified');  
L(3)=Link('d',0,'a',0.35,'alpha',0,'offset',-pi/2,'modified');  
L(4)=Link([0,0.34,0,-pi/2],'modified'); 
L(5)=Link([0,0,0,pi/2],'modified'); 
L(6)=Link([0,0.27,0,-pi/2],'modified'); 
L=SerialLink(L,'name','Robot');
L.display();
view(3);
 L.plot([0.52,0.52,0.52,1,0.52,1],'delay',3);
axis equal;
drawnow;
%% 逆运动学求解
T=[    0.9193   -0.3287   -0.2163   0.05409
   -0.3678   -0.9131   -0.1760   0.06376
   -0.1396    0.2413   -0.9603   -0.4655
         0         0         0         1];
q=L.ikine(T,'mask',[1 1 1 1 1 1])
% q=L.fkine([ -2.3616    0.8200    0.9900   -3.0873    0.5212    1.0924])
% T=[ 0.9193   -0.3288   -0.2163   -0.1709
%    -0.3679   -0.9130   -0.1760   -0.1588
%    -0.1397    0.2414   -0.9603   -0.4655
%          0         0         0         1];
% q2=ikine8(T) %ikine8为逆运动学求解函数，输出为8行6列矩阵，对应8组解
%% 给定圆心、半径、圆的法向，实现圆形路径规划
axis([-0.96 0.96 -0.96 0.96 -0.5 2]);
w=[-0.96 0.96 -0.96 0.96 -0.5 2];
R=0.2;
C=[0;-0.5;0.8];
F=[cos(1) sin(1) 0 
    -sin(1)*cos(0.3) cos(1)*cos(0.3) -sin(0.3) 
    sin(1)*sin(0.3) sin(1)*cos(0.3) cos(0.3) ];
q=drawCircle(C,R,F);
for i=1:2:200
  S=L.fkine(q(i,:));
  axis([-0.96 0.96 -0.96 0.96 -0.5 2]);
  scatter3(S.t(1),S.t(2),S.t(3),'red');
  hold on;
  L.plot(q(i,:),'delay',0.01); 
  drawnow;
end
%% 空中接物 
% 符合工作空间的输入数据输入数据太难凑了orz
T0=[0 0 1 0.1699
   0 -1 0 0.1783
   1 0 0 0.4856
   0 0 0 1];
ikine8(T0);
P1=[0.5;0.1;0.5];
P2=[-0.66;-0.08;0.69];
V1=[-0.095;0;0.32];
V2=[0.1;0.01;0.5];
tm=1000;
q=catchBall(T0,P1,P2,V1,V2,tm);
tm=q(1,1);
P1d=zeros(3,q(1,2));
P2d=zeros(3,tm+1);
if q(1,5)==1
    for t=1:tm+1
        P2d(:,t)=getPoint(P2,V2,t);
    end
    for t=1:q(1,2)
        P1d(:,t)=getPoint(P1,V1,t);
    end
else
    for t=1:tm+1
        P2d(:,t)=getPoint(P1,V1,t);
    end
    for t=1:q(1,2)
        P1d(:,t)=getPoint(P2,V2,t);
    end 
end

for i=1:7:tm+1
   L.plot(q(i+1,:),'delay',0.001); 
   axis([-0.96 0.96 -0.96 0.96 -0.5 2])
   scatter3(P2d(1,i),P2d(2,i),P2d(3,i),'g','*')
   hold on;
   if i<q(1,2)+1
   scatter3(P1d(1,i),P1d(2,i),P1d(3,i),'b')
   end
end