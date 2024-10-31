clear;
clc;
% %机器人建模
th(1) = 0; d(1) = 0.2;   a(1) = -0.220;  alp(1) = pi/2;
th(2) = 0; d(2) = 0;     a(2) = -1.069; alp(2) = 0;   
th(3) = 0; d(3) = 0.1;   a(3) = 0;      alp(3) = -pi/2;
th(4) = 0; d(4) = 1.04;  a(4) = 0;      alp(4) = pi/2;
th(5) = 0; d(5) = 0;     a(5) = 0;      alp(5) = pi/2;
th(6) = 0; d(6) = 0.495; a(6) = 0;      alp(6) = pi;
% DH parameters  th     d    a    alpha  sigma
L(1) = Link([th(1), d(1), a(1), alp(1), 0], 'revolute');
L(2) = Link([th(2), d(2), a(2), alp(2), 0], 'revolute');L(2).offset=-pi/2;
L(3) = Link([th(3), d(3), a(3), alp(3), 0], 'revolute');
L(4) = Link([th(4), d(4), a(4), alp(4), 0], 'revolute');L(4).offset=pi;
L(5) = Link([th(5), d(5), a(5), alp(5), 0], 'revolute');L(5).offset=-pi;
L(6) = Link([th(6), d(6), a(6), alp(6), 0], 'revolute');
robot = SerialLink(L); %SerialLink 类函数
robot.name='B09';

 L(1).qlim=[-180,180]/180*pi;  %工作空间，3关节和5关节的空间空间到不了[-180,180]
 L(2).qlim=[-135,135]/180*pi;
 L(3).qlim=[-90,90]/180*pi;
 L(4).qlim=[-135,135]/180*pi;
 L(5).qlim=[-90,90]/180*pi;
 L(6).qlim=[-180,180]/180*pi;

%机械臂的工作空间,关节3，5设置工作角度为[-90,90]
A=unifrnd(-pi,pi,[1,3000]);
B=unifrnd(-pi*3/4,pi*3/4,[1,3000]);
C=unifrnd(-pi/2,pi*3/4,[1,3000]);
D=unifrnd(-pi*3/4,3*pi/4,[1,3000]);
E=unifrnd(-pi/2,pi/2,[1,3000]);
F=unifrnd(-pi,pi,[1,3000]);

G=cell(3000,3);%建立元胞数组
for n=1:3000
    G{n}=[A(n) B(n) C(n) D(n) E(n) F(n)];
end                          %生成3000组随机点
H1=cell2mat(G);              %将元胞数组转化为矩阵         
T=double(robot.fkine(H1));      %机械臂正解 
%figure(1)
%scatter3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:))) %工作空间绘图
 
%% 运动学正解
theta2 = [0.1,0,0,0,0,0];   			%关节角
p1=robot.fkine(theta2)  ;     			%fkine正解函数，根据关节角theta，求解出末端位姿p
q1=ikine(robot,p1) ;           			%ikine逆解函数，根据末端位姿p，求解出关节角q

%已知初始点和终止点的位姿，五次多项式规划路径
init_ang=robot.ikine(T(:,:,10));				%根据起始点位姿，得到起始点关节角
targ_ang=robot.ikine(T(:,:,20));				%根据终止点位姿，得到终止点关节角

%轨迹规划方法并生成动画
step = 20;
f=3;
figure(f);
[q ,qd, qdd]=jtraj(init_ang,targ_ang,step); %五次多项式轨迹，得到关节角度，角速度，角加速度，20为采样点个数
grid on
T=robot.fkine(q(:,:,:));						%根据插值，得到末端执行器位姿
nT=T.T;
plot3(squeeze(nT(1,4,:)),squeeze(nT(2,4,:)),squeeze(nT(3,4,:)));%输出末端轨迹
title('输出末端轨迹');

w=3*[-1 1 -1 1 -1 2];
v=[35 20];
robot.plot3d(q,'tilesize',0.1,'workspace',w,'path','.\stl','nowrist','view',v);				%动画演示 

%% 求解位置、速度、加速度变化曲线
f = 4;
figure(f)
subplot(3,2,[1,3]); 					%subplot 对画面分区 三行两列 占用1到3的位置
plot3(squeeze(nT(1,4,:)),squeeze(nT(2,4,:)),squeeze(nT(3,4,:)));%输出末端轨迹
robot.plot3d(q,'tilesize',0.1,'workspace',w,'path','.\stl','nowrist','view',v);							%动画演示

figure(f)
subplot(3, 2, 2);
for i = 1:6
    plot(q(:,i));
    hold on;
end
title('位置');
grid on;

figure(f)
subplot(3, 2, 3);
for i = 1:6
    plot(qd(:,i));
    hold on;
end
title('速度');
grid on;

figure(f)
subplot(3, 2, 4);
for i = 1:6
    plot(qdd(:,i));
    hold on;
end
title('加速度');
grid on;

t = robot.fkine(q);					%运动学正解
rpy=tr2rpy(t);   						%t中提取位置（xyz）
figure(f)
subplot(3,2,5);
plot2(rpy);

%% ctraj规划轨迹 考虑末端执行器在两个笛卡尔位姿之间移动  
f = 5
T0 = robot.fkine(init_ang);			%运动学正解
T1 = robot.fkine(targ_ang);			%运动学正解

Tc = ctraj(T0,T1,step);  				%得到每一步的T阵

tt = transl(Tc);
figure(f)
plot2(tt,'r');
title('直线轨迹');







