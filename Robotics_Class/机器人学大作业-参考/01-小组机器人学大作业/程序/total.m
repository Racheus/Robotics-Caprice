clear;
clc;
% %�����˽�ģ
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
robot = SerialLink(L); %SerialLink �ຯ��
robot.name='B09';

 L(1).qlim=[-180,180]/180*pi;  %�����ռ䣬3�ؽں�5�ؽڵĿռ�ռ䵽����[-180,180]
 L(2).qlim=[-135,135]/180*pi;
 L(3).qlim=[-90,90]/180*pi;
 L(4).qlim=[-135,135]/180*pi;
 L(5).qlim=[-90,90]/180*pi;
 L(6).qlim=[-180,180]/180*pi;

%��е�۵Ĺ����ռ�,�ؽ�3��5���ù����Ƕ�Ϊ[-90,90]
A=unifrnd(-pi,pi,[1,3000]);
B=unifrnd(-pi*3/4,pi*3/4,[1,3000]);
C=unifrnd(-pi/2,pi*3/4,[1,3000]);
D=unifrnd(-pi*3/4,3*pi/4,[1,3000]);
E=unifrnd(-pi/2,pi/2,[1,3000]);
F=unifrnd(-pi,pi,[1,3000]);

G=cell(3000,3);%����Ԫ������
for n=1:3000
    G{n}=[A(n) B(n) C(n) D(n) E(n) F(n)];
end                          %����3000�������
H1=cell2mat(G);              %��Ԫ������ת��Ϊ����         
T=double(robot.fkine(H1));      %��е������ 
%figure(1)
%scatter3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:))) %�����ռ��ͼ
 
%% �˶�ѧ����
theta2 = [0.1,0,0,0,0,0];   			%�ؽڽ�
p1=robot.fkine(theta2)  ;     			%fkine���⺯�������ݹؽڽ�theta������ĩ��λ��p
q1=ikine(robot,p1) ;           			%ikine��⺯��������ĩ��λ��p�������ؽڽ�q

%��֪��ʼ�����ֹ���λ�ˣ���ζ���ʽ�滮·��
init_ang=robot.ikine(T(:,:,10));				%������ʼ��λ�ˣ��õ���ʼ��ؽڽ�
targ_ang=robot.ikine(T(:,:,20));				%������ֹ��λ�ˣ��õ���ֹ��ؽڽ�

%�켣�滮���������ɶ���
step = 20;
f=3;
figure(f);
[q ,qd, qdd]=jtraj(init_ang,targ_ang,step); %��ζ���ʽ�켣���õ��ؽڽǶȣ����ٶȣ��Ǽ��ٶȣ�20Ϊ���������
grid on
T=robot.fkine(q(:,:,:));						%���ݲ�ֵ���õ�ĩ��ִ����λ��
nT=T.T;
plot3(squeeze(nT(1,4,:)),squeeze(nT(2,4,:)),squeeze(nT(3,4,:)));%���ĩ�˹켣
title('���ĩ�˹켣');

w=3*[-1 1 -1 1 -1 2];
v=[35 20];
robot.plot3d(q,'tilesize',0.1,'workspace',w,'path','.\stl','nowrist','view',v);				%������ʾ 

%% ���λ�á��ٶȡ����ٶȱ仯����
f = 4;
figure(f)
subplot(3,2,[1,3]); 					%subplot �Ի������ �������� ռ��1��3��λ��
plot3(squeeze(nT(1,4,:)),squeeze(nT(2,4,:)),squeeze(nT(3,4,:)));%���ĩ�˹켣
robot.plot3d(q,'tilesize',0.1,'workspace',w,'path','.\stl','nowrist','view',v);							%������ʾ

figure(f)
subplot(3, 2, 2);
for i = 1:6
    plot(q(:,i));
    hold on;
end
title('λ��');
grid on;

figure(f)
subplot(3, 2, 3);
for i = 1:6
    plot(qd(:,i));
    hold on;
end
title('�ٶ�');
grid on;

figure(f)
subplot(3, 2, 4);
for i = 1:6
    plot(qdd(:,i));
    hold on;
end
title('���ٶ�');
grid on;

t = robot.fkine(q);					%�˶�ѧ����
rpy=tr2rpy(t);   						%t����ȡλ�ã�xyz��
figure(f)
subplot(3,2,5);
plot2(rpy);

%% ctraj�滮�켣 ����ĩ��ִ�����������ѿ���λ��֮���ƶ�  
f = 5
T0 = robot.fkine(init_ang);			%�˶�ѧ����
T1 = robot.fkine(targ_ang);			%�˶�ѧ����

Tc = ctraj(T0,T1,step);  				%�õ�ÿһ����T��

tt = transl(Tc);
figure(f)
plot2(tt,'r');
title('ֱ�߹켣');







