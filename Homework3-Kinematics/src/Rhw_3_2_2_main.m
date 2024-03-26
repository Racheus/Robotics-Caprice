clear;
clc;
addpath(genpath('.'));


L1 = Link([0,5,0,0,0],'modified');
L2 = Link([0,0,0,-pi/2,0],'modified');
L3 = Link([pi/2, 0, 5, 0],'modified');
L4 = Link([0,6,0,-pi/2,0],'modified');
L5 = Link([0,0,0,pi/2,0],'modified');
L6 = Link([0,5,0,-pi/2,0],'modified');

L3.offset = -pi/2;

SixRrobot = SerialLink([L1,L2,L3,L4,L5,L6],'name','ME3403-6Rrobot');
SixRrobot.display
SixRrobot.teach

rmpath(genpath('.'));
