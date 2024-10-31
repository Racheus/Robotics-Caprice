clear all;
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
global robot
robot = SerialLink(L); %SerialLink 类函数
robot.name='B09';


%% 设定球的参数
g=1/3;
global time_step;
time_step=0.05;
%球的初速度、位置值
ball_initial_pos=[6,0,1];
ball_initial_pos2=[6,0,2];
radius=0.1;
velx=-1;
vely=1/6;
velz=1;

velx2=-1;
vely2=-1/6;
velz2=1;

endtime=(velz+sqrt(velz*velz+2*g*(1+ball_initial_pos(3))))/g;%到地板（z=-1）停止
time_vec=0:time_step:endtime;
time_vec=transpose(time_vec);
ball_x=velx*time_vec+ball_initial_pos(1);
ball_y=vely*time_vec+ball_initial_pos(2);
ball_z=velz*time_vec-g/2*time_vec.^2+ball_initial_pos(3);

endtime2=(velz2+sqrt(velz2*velz2+2*g*(1+ball_initial_pos2(3))))/g;%到地板（z=-1）停止

time_vec2=0:time_step:endtime2;
time_vec2=transpose(time_vec2);
ball_x2=velx2*time_vec2+ball_initial_pos2(1);
ball_y2=vely2*time_vec2+ball_initial_pos2(2);
ball_z2=velz2*time_vec2-g/2*time_vec2.^2+ball_initial_pos2(3);
%% 求解机械臂接球轨迹
global i_stop;
i_stop=0;
global ball_pos_fir ball_pos_sec;
global iscatch2;
data=catchball2(ball_x,ball_y,ball_z,time_vec,ball_x2,ball_y2,ball_z2,time_vec2);
iscatch2
if isempty(data)
   q_targ_t=zeros(length(time_vec),6,1);
   "Catch Ball Fail"
else
    "Catch Ball Success"
   q_targ_t=data(:,2:7);
   time_vec=data(:,1);
   
end

ball_x=ball_pos_fir(:,1);
ball_y=ball_pos_fir(:,2);
ball_z=ball_pos_fir(:,3);
ball_x2=ball_pos_sec(:,1);
ball_y2=ball_pos_sec(:,2);
ball_z2=ball_pos_sec(:,3);

%% 画图

fig=figure(1);
fig.Position=[100 100 1400 1200];
i=1;
radius=0.1;
[ball_x_mesh,ball_y_mesh,ball_z_mesh]=sphere;
ball_x_mesh=ball_x_mesh*radius;
ball_y_mesh=ball_y_mesh*radius;
ball_z_mesh=ball_z_mesh*radius;

v=[70 30];
w=[-3 7 -4 4 -1 4];
range_w=[w(2)-w(1),w(4)-w(3),w(6)-w(5)];
for i=1:1:i_stop
 ball=surf(ball_x_mesh+ball_x(i),ball_y_mesh+ball_y(i),ball_z_mesh+ball_z(i),'EdgeColor','none'); 
 hold on;
 ball2=surf(ball_x_mesh+ball_x2(i),ball_y_mesh+ball_y2(i),ball_z_mesh+ball_z2(i),'EdgeColor','none'); 
 
 shooter=plot3(ball_initial_pos(1),ball_initial_pos(2),ball_initial_pos(3),'Marker','o','MarkerSize',15,'LineWidth',3);
 shooter2=plot3(ball_initial_pos2(1),ball_initial_pos2(2),ball_initial_pos2(3),'Marker','o','MarkerSize',15,'LineWidth',3);
 trace_ball=plot3(ball_x(1:i),ball_y(1:i),ball_z(1:i),'LineStyle','--','LineWidth',2,'Color','r');
 trace_ball2=plot3(ball_x2(1:i),ball_y2(1:i),ball_z2(1:i),'LineStyle','--','LineWidth',2,'Color','b');
 
 axis(w);
 pbaspect(range_w);
 robot.plot3d(q_targ_t(i,:),'tilesize',0.5,'workspace',w,'path','.\stl','nowrist','view',v,'perspective');
 
 clf(fig)
end

for i=i_stop+1:1:size(data,1)
 ball2=surf(ball_x_mesh+ball_x2(i),ball_y_mesh+ball_y2(i),ball_z_mesh+ball_z2(i),'EdgeColor','none');  
 hold on;
 shooter=plot3(ball_initial_pos(1),ball_initial_pos(2),ball_initial_pos(3),'Marker','o','MarkerSize',15,'LineWidth',3);
 shooter2=plot3(ball_initial_pos2(1),ball_initial_pos2(2),ball_initial_pos2(3),'Marker','o','MarkerSize',15,'LineWidth',3);
 trace_ball2=plot3(ball_x2(1:i),ball_y2(1:i),ball_z2(1:i),'LineStyle','--','LineWidth',2,'Color','b');
 axis(w);
 pbaspect(range_w);
 robot.plot3d(q_targ_t(i,:),'tilesize',0.5,'workspace',w,'path','.\stl','nowrist','view',v,'perspective');
 
 if i<length(time_vec)
 clf(fig)
 end
end


%%
function data=catchball2(ball_x,ball_y,ball_z,time_vec,ball_x2,ball_y2,ball_z2,time_vec2)
global robot;
global time_step;
N=length(time_vec);
q_init=[0 0 0 0 0 0];
iscatch=-1;
global iscatch2;
iscatch2=-1;
for i=1:1:N
targ_p=[ball_x(i);ball_y(i);ball_z(i)];
targ_p2=[ball_x2(i);ball_y2(i);ball_z2(i)];
%首先根据工作空间排除达不到的点↓
if (not((targ_p(1)>-2) & (targ_p(1)<2) & (targ_p(2)>-2) & (targ_p(2)<2) & (targ_p(3)>-2) & (targ_p(3)<3)))...
        &(not((targ_p2(1)>-2) & (targ_p2(1)<2) & (targ_p2(2)>-2) & (targ_p2(2)<2) & (targ_p2(3)>-2) & (targ_p2(3)<3)))
    continue;
end
%排除达不到的点↓
p=[0 0 -1 0;0 -1 0 0;-1 0 0 0;0 0 0 1];
p2=[0 0 -1 0;0 -1 0 0;-1 0 0 0;0 0 0 1];
p(1:3,4)=targ_p;
p2(1:3,4)=targ_p2;
q_targ=ikine(robot,p);
q_targ2=ikine(robot,p2);
if(isempty(q_targ))&(isempty(q_targ2))
    continue;
end
global ball_pos_fir ball_pos_sec;
if not (isempty(q_targ))
   ball_pos_fir=[ball_x,ball_y,ball_z,time_vec];
   ball_pos_sec=[ball_x2,ball_y2,ball_z2,time_vec2];
else
   ball_pos_sec=[ball_x,ball_y,ball_z,time_vec];
   ball_pos_fir=[ball_x2,ball_y2,ball_z2,time_vec2];
   q_targ=q_targ2;
end

time=0.5;
qd_limit=2.2;
robot_time_vec=0:time_step:time;
[q_targ_t ,qd_t, qdd_t]=jtraj(q_init,q_targ,robot_time_vec);
while(max(abs(qd_t),[],'all')>qd_limit)
    time=time*max(abs(qd_t),[],'all')/qd_limit+time_step;
    robot_time_vec=0:time_step:time;
    [q_targ_t ,qd_t, qdd_t]=jtraj(q_init,q_targ,robot_time_vec);
end

if time<=time_vec(i)
    iscatch=1;
    q_init=q_targ;
    break;
end
end
global i_stop;
i_stop=i;
i_start=i+4;
time_start_sec=time_vec(i_start);
N=size(ball_pos_sec,1);

for i=i_start:1:N;
    
    targ_p=[ball_pos_sec(i,1),ball_pos_sec(i,2),ball_pos_sec(i,3)];
    if (not((targ_p(1)>-2) & (targ_p(1)<2) & (targ_p(2)>-2) & (targ_p(2)<2) & (targ_p(3)>-2) & (targ_p(3)<3)))
            continue;
          end
    p=[0 0 -1 targ_p(1);0 -1 0 targ_p(2);-1 0 0 targ_p(3);0 0 0 1];
    p(1:3,1:3)=[targ_p(1)/sqrt(targ_p(1)^2+targ_p(2)^2) -targ_p(2)/sqrt(targ_p(1)^2+targ_p(2)^2) 0;targ_p(2)/sqrt(targ_p(1)^2+targ_p(2)^2) targ_p(1)/sqrt(targ_p(1)^2+targ_p(2)^2) 0;0 0 1]*p(1:3,1:3);
    q_targ=ikine(robot,p);
    if isempty(q_targ)
        continue;
    end
    time2=0.4;
    qd_limit=4;
    robot_time_vec2=0:time_step:time2;
    [q_targ_t2 ,qd_t2, qdd_t2]=jtraj(q_init,q_targ,robot_time_vec2);
while(max(abs(qd_t2),[],'all')>qd_limit)
    time2=time2*max(abs(qd_t2),[],'all')/qd_limit+time_step;
    robot_time_vec2=0:time_step:time2;
    [q_targ_t2,qd_t2, qdd_t2]=jtraj(q_init,q_targ,robot_time_vec2);
end
if time2+time_start_sec<=ball_pos_sec(i,4)
    iscatch2=1;
    break;
end
end



%输出结果↓
if iscatch==-1
    data=[];
else
    
if  iscatch2==-1
    N=size(ball_pos_sec,1);
    n1=length(robot_time_vec);
    data=zeros(N,7,1);
    data(:,1)=ball_pos_sec(1:N,4);
    data(i_stop-n1+1:i_stop,2:7)=q_targ_t;
    for j=i_stop+1:1:N
    data(j,2:7)=q_targ_t(n1,:);
    end
else
    n1=length(robot_time_vec);
    n2=length(robot_time_vec2);
    data=zeros(i,7,1);
    data(:,1)=ball_pos_sec(1:i,4);
    data(i_stop-n1+1:i_stop,2:7)=q_targ_t;
    for j=i_stop:1:i-n2
    data(j,2:7)=q_targ_t(n1,:);
    end
    data(i-n2+1:i,2:7)=q_targ_t2;
    
end
end


end

