% ME3403-01 , Robotics Homework 1-4
% 赵四维 , 521021910696
% Copyright 2023-2024-2 Spring Racheus Zhao

clear;
clc;
addpath(genpath('.'))

% Create a 2R robot model
L1 = Link('d', 0, 'a', 2, 'alpha', 0);
L2 = Link('d', 0, 'a', 1, 'alpha', 0);
robot = SerialLink([L1 L2], 'name', 'ME3403-2Rrobot');



q_now=[0;0];
trajectory = []; % Trace array


figure;
trajectory = [];
v = VideoWriter('robot_trajectory.avi');
open(v);

for t = 0:0.005:1
    x = 4.5 - 3 * sin((pi/3) * t + pi/6);
    y = 3 * sqrt(3)/2 - 3 * cos((pi/3) * t + pi/6);
    
    disp("time(t) :")
    disp(t);
    
    q_stack = getQ_2para(x, y);
    q_now = q_Judgemnt(q_stack, q_now);
    q_now_col = reshape(q_now, [], 1);

    T = robot.fkine(q_now_col);
    end_effector_pos = T.t(1:2)'; 

    trajectory = [trajectory; end_effector_pos];
    
    robot.plot(q_now_col');
    disp("q matrix :");
    disp(q_now_col);
    
    view(0, 90); 
    axis([-1 5 -1 5 0 5]);
    hold on; 
    plot(trajectory(:, 1), trajectory(:, 2), 'r'); 
    hold off;

    title('HW-1 2R-robot Trajectory Planning');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis equal;
    drawnow; 
    pause(0.05); 
    
    % Capture frame and write to video
    frame = getframe(gcf);
    writeVideo(v,frame);
end


close(v);
pause(1);
rmpath(genpath('.'));


function q = getQ_2para(x1,y1)
D_now = (x1*x1+y1*y1-2*2-1*1)/(2*2*1);
theta2_1 = atan2(sqrt(1-D_now*D_now),D_now);
theta2_2 = atan2(-sqrt(1-D_now*D_now),D_now);
theta1_1 = atan2(y1,x1)-atan2(sin(theta2_1),(2+cos(theta2_1)));
theta1_2 = atan2(y1,x1)-atan2(sin(theta2_2),(2+cos(theta2_2)));
q=[theta1_1;theta2_1;theta1_2;theta2_2];
end


function outputq = q_Judgemnt(q_stack, q_now)
    dist_1 = norm(q_stack(1:2) - q_now); 
    dist_2 = norm(q_stack(3:4) - q_now); 
    if dist_1 <= dist_2
        outputq = q_stack(1:2); 
    else
        outputq = q_stack(3:4); 
    end
end


    