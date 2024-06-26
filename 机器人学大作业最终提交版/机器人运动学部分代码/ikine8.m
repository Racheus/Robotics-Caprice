function q2 = ikine8(T)
q=zeros(6,8);
R6=T(1:3,1:3);
p6=T(1:3,4);
pw=p6-R6*[0;0;0.27];
pwx=pw(1);
pwy=pw(2);
pwz=pw(3);
%% 计算 \theta_1 （2种）
for i=1:4
    q(1,i)=atan2(pwy,pwx);
end
for i=5:8
    q(1,i)=atan2(pwy,pwx)+pi;
end
%% 计算\theta_3 （2种）
for i=1:2
    q(3,i)=acos(500/119*(pwx^2+pwy^2-19/25*pwz+pwz^2-937/10000));
end
for i=5:6
    q(3,i)=acos(500/119*(pwx^2+pwy^2-19/25*pwz+pwz^2-937/10000));
end
for i=3:4
    q(3,i)=acos(500/119*(pwx^2+pwy^2-19/25*pwz+pwz^2-937/10000))+pi;
end
for i=7:8
    q(3,i)=acos(500/119*(pwx^2+pwy^2-19/25*pwz+pwz^2-937/10000))+pi;
end
%% 计算\theta_2 （1种）
for i=1:8
    q(2,i)=atan2(-((7/20+17/50*cos(q(3,i)))*(pwz-19/50)+17/50*sin(q(3,i))*sqrt(pwx^2+pwy^2)),((7/20+17/50*cos(q(3,i)))*sqrt(pwx^2+pwy^2)-17/50*sin(q(3,i))*(pwz-19/50)));
end
%% 计算\theta_4、5、6 （2种）
for i=1:8
   R3=[cos(q(1,i))*sin(q(2,i)+q(3,i)) cos(q(1,i))*cos(q(2,i)+q(3,i)) -sin(q(1,i))
       sin(q(1,i))*sin(q(2,i)+q(3,i)) sin(q(1,i))*cos(q(2,i)+q(3,i)) cos(q(1,i))
       cos(q(2,i)+q(3,i)) -sin(q(2,i)+q(3,i)) 0];
   R36=R3\R6;   
   ax=R36(1,3);
   ay=R36(2,3);
   az=R36(3,3);
   sy=R36(2,2);
   ny=R36(2,1);
   if mod(i,2)==0
       q(4,i)=atan2(az,-ax);
       q(5,i)=atan2(sqrt(1-ay^2),ay);
       q(6,i)=atan2(-sy,ny);
   else
       q(4,i)=atan2(-az,ax);
       q(5,i)=atan2(-sqrt(1-ay^2),ay);
       q(6,i)=atan2(sy,-ny);
   end
end
for i=1:6
    for j=1:8
        if q(i,j)>pi
            q(i,j)=q(i,j)-2*pi;
        end
    end
end
%% 共有2*2*1*2=8组解
q2=q';
end

