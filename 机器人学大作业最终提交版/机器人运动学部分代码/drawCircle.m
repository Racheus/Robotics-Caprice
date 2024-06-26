function q = drawCircle(Centre,Radius,Face)
% Centre：圆心 Radius：半径 Face：法向旋转矩阵
% 给定C、R、F，得到机器人末端轨迹（方向同Face），逆运动学计算得到200组旋转角作为输出。
T0(1:3,:)=[Face,Centre];
T0(4,:)=[0 0 0 1];
q=zeros(100,6);
p=1;
d=zeros(3,100);
for s=1:200
    theta=s/100*pi;
    % T1=[cos(theta)  -sin(theta) 0 0;sin(theta)  cos(theta) 0 0;0 0 1 0;0 0 0 1];
    % T2=[1 0 0 Radius;0 1 0 0;0 0 1 0;0 0 0 1];
    T3=[1 0 0 Radius*cos(theta)
        0 1 0 Radius*sin(theta)
        0 0 1 0
        0 0 0 1];
    % T=T0*T1*T2;
    T=T0*T3;
    d(:,s)=T(1:3,4);
    % T(1:3,1:3)=Face;
    q0=ikine8(T);%得到逆运动学结果
    t2=1;
    chosen=1000;
    for t=1:8
       if s==1 
           q(1,:)=q0(p,:);%直接确定第一个点的逆运动学。p默认为1，若发生穿模则p++（暂未实现）
       else
           sum=0;
           for t1=1:6 %计算q与上一个q的六维距离，选取使其最小的q，确保运动连续，不出现瞬移
               if (q(s-1,t1)-q0(t,t1))^2<pi^2
                   sum=sum+(q(s-1,t1)-q0(t,t1))^2;
               else 
                   sum=sum+(2*pi-sqrt((q(s-1,t1)-q0(t,t1))^2))^2;
               end
           if sum<chosen
               chosen=sum;
               t2=t;
           end
           end
       end
    end
    q(s,:)=q0(t2,:);
end
end

