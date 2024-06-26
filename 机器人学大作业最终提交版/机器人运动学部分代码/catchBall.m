function q = catchBall(T0,P1,P2,V1,V2,tm,ts,R1,R2)
arguments
    T0(4,4)
    P1(3,1)
    P2(3,1)
    V1(3,1)
    V2(3,1)
    tm double=1000
    ts double=100
    R1(3,3)double=[0 0 1
                   0 -1 0
                   1 0 0]
    R2(3,3)double=[0 0 1
                   0 -1 0
                   1 0 0]
end
P0=T0(1:3,4);
q=zeros(1,6);
tp1=tm;
tp2=tm;
for k=1:tm
    if distance(getPoint(P2,V2,tp1),[0;0;0.38])<0.96
        break;
    end
    tp1=tp1-1;
end
for k=1:tm
    if distance(getPoint(P1,V1,tp2),[0;0;0.38])<0.96
        break;
    end
    tp2=tp2-1;
end
p=1;
%% 先球1后球2，确定经过点
tmp=1000;
t1=1;
for t=50:tp1-50
    distance(getPoint(P1,V1,t),[0;0;0.38]);
    if distance(getPoint(P1,V1,t),[0;0;0.38])<0.96
        X=abs(distance(getPoint(P1,V1,t),P0)/(distance(getPoint(P1,V1,t),P0)+distance(getPoint(P2,V2,tp1),getPoint(P1,V1,t)))-t/(tp1-ts));
        if tmp>abs(X)
            tmp=abs(X);
            t1=t;
        end
    end
end
    Pt=getPoint(P1,V1,t1);
    Pm=getPoint(P2,V2,tp1);
    T1(1:3,:)=[R1,Pt];
    T1(4,:)=[0 0 0 1];
    T2(1:3,:)=[R2,Pm];
    T2(4,:)=[0 0 0 1];
    t2=tp1-t1-ts;
    A =[ 1 0 0 0 0 0 0 0
         0 1 0 0 0 0 0 0
         1 t1/100 (t1/100)^2 (t1/100)^3 0 0 0 0
         0 1 2*t1/100 3*(t1/100)^2 0 -1 0 0
         0 0 0 0 1 0 0 0
         0 0 0 0 1 t2/100 (t2/100)^2 (t2/100)^3
         0 0 0 0 0 1 2*(t2/100) 3*(t2/100)^2
         0 0 2 6*t1/100 0 0 -2 0];
    T_1=zeros(3*(t1+t2),4);
    T_2=zeros(3*(t1+t2),4);
    for i=1:3
        for j=1:4
            B=[T0(i,j);0;T1(i,j);0;T1(i,j);T2(i,j);0;0]; %系数矩阵 A*y=B
            C1=A\B;
            for t=0:(t1+t2)
                if t<t1
                    T_1(i+3*t,j)=C1(1)+C1(2)*t/100+C1(3)*t^2/10000+C1(4)*(t/100)^3;
                else
                    T_1(i+3*t,j)=C1(5)+C1(6)*(t-t1)/100+C1(7)*(t-t1)^2/10000+C1(8)*(t-t1)^3/1000000;
                end
            end
        end
    end
    sum=0;
    for t=1:(t1+t2)
        sum=sum+distance(T_1(3*t+1:3*t+3,4),T_1(3*t-2:3*t,4));
    end
    sum1=sum;
    tt1=t1;
%% 先球2后球1，确定经过点 （同理） 
tmp=1000;
t1=1;
for t=50:tp2-50
    if distance(getPoint(P2,V2,t),[0;0;0.38])<0.96
        X=abs(distance(getPoint(P2,V2,t),P0)/distance(getPoint(P1,V1,tp2),getPoint(P2,V2,t))-t/(tp2-t-ts));
        if tmp>abs(X)
            tmp=abs(X);
            t1=t;
        end
    end
end
Pt=getPoint(P2,V2,t1);
Pm=getPoint(P1,V1,tp2);
T1(1:3,:)=[R2,Pt];
T1(4,:)=[0 0 0 1];
T2(1:3,:)=[R1,Pm];
T2(4,:)=[0 0 0 1];
t2=tp2-t1-ts;
for i=1:3
    for j=1:4
        B=[T0(i,j);0;T1(i,j);0;T1(i,j);T2(i,j);0;0]; %系数矩阵 A*y=B
        C2=A\B;
        for t=0:(t1+t2)
            if t<t1
                tc=t/100;
                T_2(i+3*t,j)=C2(1)+C2(2)*tc+C2(3)*tc^2+C2(4)*tc^3;
            else
                tc=t/100;
                T_2(i+3*t,j)=C2(5)+C2(6)*(tc-t1/100)+C2(7)*(tc-t1/100)^2+C2(8)*(tc-t1/100)^3;
            end
        end
    end
end
sum=0;
for t=1:(t1+t2)
    sum=sum+distance(T_2(3*t+1:3*t+3,4),T_2(3*t-2:3*t,4));
end
sum2=sum;
%% 后续
if sum1<sum2
    T=T_1;
    tm=tp1;
    t1=tt1;
    ball1=1;
    ball2=2;
else
    T=T_2;
    tm=tp2;
    ball1=2;
    ball2=1;
end
q(1,:)=[tm,t1,ts,tm-ts-t1,ball1,ball2];
for t=0:tm
    if t<t1
        a=t;
    elseif t<t1+ts
        a=t1;
    else
        a=t-ts;
    end
    Ta(1:3,:)=T(3*a+1:3*a+3,:);
    Ta(4,:)=[0,0,0,1];
    t=t+1;
    q0=ikine8(Ta);%得到逆运动学结果
    tmp=1;
    chosen=1000;
    for line=1:8
       if t==1 
       q(2,:)=q0(p,:);
       else
           sum=0;
           for tmp2=1:6 %计算q与上一个q的六维距离，选取使其最小的q，确保运动连续，不出现瞬移
               if (q(t-1,tmp2)-q0(line,tmp2))^2<pi^2
                   sum=sum+(q(t-1,tmp2)-q0(line,tmp2))^2;
               else 
                   sum=sum+(2*pi-sqrt((q(t-1,tmp2)-q0(line,tmp2))^2))^2;
               end
               if sum<chosen
                   chosen=sum;
                   tmp=line;
               end
           end
       end
    end
    if t>1
    q(t+1,:)=q0(tmp,:);
    end
end
end

