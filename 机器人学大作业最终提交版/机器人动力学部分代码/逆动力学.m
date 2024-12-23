clear all; 
clc; 
% 定义机器人的六个关节的改进-DH参数表
L(1)= Link([0 0.38 0 0],'modified');
L(2)= Link([0 0 0 -pi/2],'modified');
L(3)= Link([0 0 0.35 0],'modified'); L(3).offset = -pi/2;
L(4)= Link([0 0.34 0 -pi/2],'modified');
L(5)= Link([0 0 0 pi/2],'modified');
L(6)= Link([0 0.27 0 -pi/2],'modified');

% 设计机器人各关节动力学参数（质量(kg)、质心位置(m)、电机惯性、惯量矩阵(kg*m^2）
L(1).m=5.235;
L(1).r=[0.115,0,0.192];
L(1).Jm=0.0002;
L(1).I=10^(-6)*[233372.299 12.446 115929.582;
	            12.446 314433.958 -1424.735;
	            115929.582 -1424.735 104166.070];

L(2).m=7.873;
L(2).r=[0.169,0.191,0.208];
L(2).Jm=0.0002;
L(2).I=10^(-6)*[813892.378 250920.627 277029.704;
	            250920.627 614464.982 322817.779;
	            277029.704 322817.779 674792.813];


L(3).m=3.134;
L(3).r=[0.186,0.389,0.224];
L(3).Jm=0.0002;
L(3).I=10^(-6)*[649467.616 227018.391 131078.955;
	            227018.391 274135.221 274686.973;
	            131078.955 274686.973 603049.531];

L(4).m=1.870;
L(4).r=[0.171,0.593,0.241];
L(4).Jm=0.0002;
L(4).I=10^(-6)*[774568.299 189247.218 76952.202;
	            189247.218 166909.189 267523.097;
	            76952.202 267523.097 720406.439];

L(5).m=0.492;
L(5).r=[0.163,0.697,0.250];
L(5).Jm=0.0002;
L(5).I=10^(-6)*[270700.434 55908.336 20055.668;
	            55908.336 44195.340 85791.755;
	            20055.668 85791.755 253206.649];

L(6).m=0.627;
L(6).r=[0.146,0.864,0.263];
L(6).Jm=0.0002;
L(6).I=10^(-6)*[513522.841 78780.688 24023.978;
	            78780.688 58186.565 142442.647;
	            24023.978 142442.647 482683.267];

% 创建机器人对象
robot = SerialLink(L, 'name', 'six_link');


%%
%正向动力学，给定力矩求关节角、角速度、角加速度


[t1,q1,qd1] = robot.nofriction().fdyn(2, @my_torque_function);   %求解关节角和角速度
tau1 = zeros(size(t1,1), 6);
for i=1:size(t1,1)
    tau1(i,:)=[120 200 150 100 50 20];
end
qdd1 = robot.accel(q1,qd1,tau1);   %求解关节角加速度

subplot(3,2,[1,3]); 					%subplot 对画面分区 三行两列 占用1到3的位置
figure(1);
grid on

figure(1);
subplot(3,2,1);
plot(t1',tau1,'LineWidth',2);grid on;title('关节力矩');   %给定各关节的关节力矩随时间变化曲线
xlabel('t/s');
ylabel('tau/Nm');
legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','bestoutside');
grid on;

figure(1)
subplot(3, 2, 2);
for i=1:6
    plot(t1',q1(:,i));    %动力学正解得到关节角随时间变化曲线
    hold on;
end
legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','bestoutside');
title('关节角');
xlabel('t/s');
ylabel('q/rad');
grid on;

figure(1)
subplot(3, 2, 3);
for i=1:6
    plot(t1',qd1(:,i));     %动力学正解得到关节角速度随时间变化曲线
    hold on;
end
legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','bestoutside');
title('速度');
xlabel('t/s');
ylabel('w/rad/s');
grid on;

figure(1)
subplot(3, 2, 4);
for i=1:6
    plot(t1',qdd1(:,i));       %动力学正解得到关节角加速度随时间变化曲线
    hold on;
end
legend('关节1','关节2','关节3','关节4','关节5','关节6','Location','bestoutside');
title('加速度');
xlabel('t/s');
ylabel('a/(rad/s/s)');
grid on;

T1=robot.fkine(q1(:,:,:));						%根据插值，得到末端执行器位姿
nT1=T1.T;
subplot(3,2,5);
plot3(squeeze(nT1(1,4,:)),squeeze(nT1(2,4,:)),squeeze(nT1(3,4,:)));   %输出给定关节力矩对应的末端轨迹
title('输出末端轨迹');

figure(2)
% 遍历关节角度轨迹数据，并绘制机器人的动画
for i = 1:size(t1,1)
    robot.plot(q1);
    pause(0.001);
end

function tau1 = my_torque_function(robot, t1, q1, qd1)
tau1 = [120 200 150 100 50 20];   %假设各关节力矩为常量
end