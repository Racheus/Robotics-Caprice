clear;
clc;
% %�����˽�ģ
th(1) = 0; d(1) = 0.2/2.8;   a(1) = -0.22/2.8;  alp(1) = pi/2;
th(2) = 0; d(2) = 0;     a(2) = -1.069/2.8; alp(2) = 0;   
th(3) = 0; d(3) = 0.1/2.8;   a(3) = 0;      alp(3) = -pi/2;
th(4) = 0; d(4) = 1.04/2.8;  a(4) = 0;      alp(4) = pi/2;
th(5) = 0; d(5) = 0;     a(5) = 0;      alp(5) = pi/2;
th(6) = 0; d(6) = 0.495/2.8; a(6) = 0;      alp(6) = pi;
% DH parameters  th     d    a    alpha  sigma
L(1) = Link([th(1), d(1), a(1), alp(1), 0], 'revolute');
L(2) = Link([th(2), d(2), a(2), alp(2), 0], 'revolute');L(2).offset=-pi/2;
L(3) = Link([th(3), d(3), a(3), alp(3), 0], 'revolute');
L(4) = Link([th(4), d(4), a(4), alp(4), 0], 'revolute');L(4).offset=pi;
L(5) = Link([th(5), d(5), a(5), alp(5), 0], 'revolute');L(5).offset=-pi;
L(6) = Link([th(6), d(6), a(6), alp(6), 0], 'revolute');

robot = SerialLink(L); %SerialLink �ຯ��
robot.name='B09';

%�ٶ��ſɱȼ���
q0=[0 -pi/2 0 pi -pi 0];
J0=Jacob_SDH(q0);

%�ٶ��ſɱ�
function [ J ] = Jacob_SDH( q )
%JACOB_SDH ����ժҪ
%   ����q0Ϊ�ƽ��ǣ���λΪ���ȣ������С1*6;
%   ���JΪ�ٶ��Ÿ��Ⱦ��󣬾����С6*6��
%   �����������ķ������ϵͳ���Ÿ��Ⱦ���
%   ����ⷽ������SDH������ģ

d=[0.2,0,0.1,1.04,0,0.495]./2.8;
a=[-0.22,-1.069,0,0,0,0]./2.8;
alp=[pi/2,0,-pi/2,pi/2,pi/2,pi];
offset=[0,-pi/2,0,pi,-pi,0];
thd=q+offset;

% ������ؽڼ�ı任����
T0=trotz(0)*transl(0,0,0)*trotx(0)*transl(0,0,0);
T1=trotz(thd(1))*transl(0,0,d(1))*trotx(alp(1))*transl(a(1),0,0);
T2=trotz(thd(2))*transl(0,0,d(2))*trotx(alp(2))*transl(a(2),0,0);
T3=trotz(thd(3))*transl(0,0,d(3))*trotx(alp(3))*transl(a(3),0,0);
T4=trotz(thd(4))*transl(0,0,d(4))*trotx(alp(4))*transl(a(4),0,0);
T5=trotz(thd(5))*transl(0,0,d(5))*trotx(alp(5))*transl(a(5),0,0);
T6=trotz(thd(6))*transl(0,0,d(6))*trotx(alp(6))*transl(a(6),0,0);

% ������ؽ�����ڹ�������ϵ�ı任����
T00 = T0;
T01 = T1;
T02 = T1*T2;
T03 = T1*T2*T3;
T04 = T1*T2*T3*T4;
T05 = T1*T2*T3*T4*T5;
T06 = T1*T2*T3*T4*T5*T6;

% ������ؽ������ĩ������ϵ�ı任����
T06 = T1*T2*T3*T4*T5*T6;
T16 = T2*T3*T4*T5*T6;
T26 = T3*T4*T5*T6;
T36 = T4*T5*T6;
T46 = T5*T6;
T56 = T6;

% ��ȡ���任�������ת����
R00 = t2r(T00);
R01 = t2r(T01);
R02 = t2r(T02);
R03 = t2r(T03);
R04 = t2r(T04);
R05 = t2r(T05);
R06 = t2r(T06);

% ȡ��ת�����3�У���Z�᷽�����
Z0 = R00(: , 3);
Z1 = R01(: , 3);
Z2 = R02(: , 3);
Z3 = R03(: , 3);
Z4 = R04(: , 3);
Z5 = R05(: , 3);
Z6 = R06(: , 3);

% ��ĩ�˹ؽ�����ϵ�����ǰ���������ϵ��λ�ã�����α任����ĵ�����
% pi6Ϊ����ϵi��ĩ������ϵ�����λ��������ϵi�µı�ʾ
P06 = T06(1:3, 4);
P16 = T16(1:3, 4);
P26 = T26(1:3, 4);
P36 = T36(1:3, 4);
P46 = T46(1:3, 4);
P56 = T56(1:3, 4);
P66 = [0; 0; 0];

% ʹ������������ſɱȾ���
% R0iΪ����ϵ0������ϵi����ת����
% R0i*Pi6ָ����ϵi��ĩ������ϵ�����λ����0����ϵ�µı�ʾ
J1 = [cross(Z0, R00*P06); Z0];
J2 = [cross(Z1, R01*P16); Z1];
J3 = [cross(Z2, R02*P26); Z2];
J4 = [cross(Z3, R03*P36); Z3];
J5 = [cross(Z4, R04*P46); Z4];
J6 = [cross(Z5, R05*P56); Z5];

J = [J1, J2, J3, J4, J5, J6];

end






