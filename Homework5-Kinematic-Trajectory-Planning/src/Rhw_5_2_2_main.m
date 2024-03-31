% Robotics Homework-5-2-1
% 2023-2024-2 Spring 
% Racheus Zhao ,School of Mechanical Engineering ,SJTU

clear;
clc;
addpath(genpath('.'));
%---------------------Model Construction----------------------
T1=[0 0 1 3;0 -1 0 -2;1 0 0 0 ;0 0 0 1];
T2=[0 0 1 3;0 -1 0 2;1 0 0 2;0 0 0 1];
T3=[0 0 1 3;0 -1 0 -2;1 0 0 4;0 0 0 1];
L1 = Link([0,2,0,0,0],'modified');
L2 = Link([0,0,0,-pi/2,0],'modified');
L3 = Link([pi/2, 0, 2, 0],'modified');
L4 = Link([0,1,0,-pi/2,0],'modified');
L5 = Link([0,0,0,pi/2,0],'modified');
L6 = Link([0,2,0,-pi/2,0],'modified');

L3.offset = -pi/2;

SixRrobot = SerialLink([L1,L2,L3,L4,L5,L6],'name','ME3403-6Rrobot');


%---------------------Cubic spline interpolation calculation----------------------

t1 = T1(1:3, 4);
t2 = T2(1:3, 4);
t3 = T3(1:3, 4);
disp([t1;t2;t3]);
R1 = T1(1:3, 1:3);
R2 = T2(1:3, 1:3);
R3 = T3(1:3, 1:3);

num_interp_points = 200;
t_interp = linspace(0, 1, num_interp_points); % 插值点从0到1均匀取num_interp_points个点

t_interp_x = spline([0, 2, 4], [t1(1), t2(1), t3(1)], linspace(0, 4, num_interp_points));
t_interp_y = spline([0, 2, 4], [t1(2), t2(2), t3(2)], linspace(0, 4, num_interp_points));
t_interp_z = spline([0, 2, 4], [t1(3), t2(3), t3(3)], linspace(0, 4, num_interp_points));

R_interp(:,:,1:0.5*num_interp_points) = slerp_rotm(R1, R2, 0.5*num_interp_points);
R_interp(:,:,0.5*num_interp_points+1:num_interp_points) = slerp_rotm(R2, R3, 0.5*num_interp_points);

time_points = linspace(0, 4, num_interp_points);
poses = zeros(4, 4, num_interp_points);
q = zeros(6,num_interp_points);
euler_angles = zeros(3,num_interp_points);

for i = 1:num_interp_points
    poses(:,:,i) = eye(4);
    poses(1:3, 1:3, i) = R_interp(:,:,i);
    euler_angles(:,i)=tr2rpy(R_interp(:,:,i));
    poses(1:3, 4, i) = [t_interp_x(i); t_interp_y(i); t_interp_z(i)];
    q(:,i) = SixRrobot.ikine(poses(:,:,i));
end
%---------------------Plot Angle Diagram ----------------------

figure;
sgtitle('Angle diagram of each joint under \color{red}pose spline');
for i = 1:6
    subplot(3, 2, i);
    plot(time_points, q(i,:), '-');
    hold on;
    plot([0, 2, 4], [q(i,1),q(i,0.5*num_interp_points),q(i,num_interp_points)], 'r.', 'MarkerSize', 10);
    hold off;
    xlabel('Time');
    ylabel(['Joint ', num2str(i), ' Angle']);
    title(['Joint ', num2str(i), ' Angle vs Time']);
end
saveas(gcf,'angle_spline_diagram_pose.png')



%---------------------Plot Trans&Rot Vectors----------------------
translation_vectors = [t_interp_x;t_interp_y;t_interp_z];
figure;
sgtitle('Translation vectors of effector under \color{red}pose spline');
subplot(3, 1, 1); 
plot(time_points, translation_vectors(1, :), 'c-', 'LineWidth', 2);
hold on;
plot([0, 2, 4], translation_vectors(1, [1,0.5*end, end]), 'r*', 'MarkerSize', 10);
xlabel('Time');
ylabel('X Translation');
title('X Translation vs Time');

subplot(3, 1, 2); 
plot(time_points, translation_vectors(2, :), 'b-', 'LineWidth', 2);
hold on;
plot([0, 2, 4], translation_vectors(2, [1,0.5*end, end]), 'r*', 'MarkerSize', 10); 
hold off;
xlabel('Time');
ylabel('Y Translation');
title('Y Translation vs Time');

subplot(3, 1, 3);
plot(time_points, translation_vectors(3, :), 'y-', 'LineWidth', 2);
hold on;
plot([0, 2, 4], translation_vectors(3, [1, 0.5*end, end]), 'r*', 'MarkerSize', 10); 
xlabel('Time');
ylabel('Z Translation');
title('Z Translation vs Time');

saveas(gcf,'effector_trans_diagram_pose.png')



figure;
sgtitle('Rotation vectors of effector under \color{red}pose spline(RPY)');
subplot(3, 1, 1); 
plot(time_points, rad2deg(euler_angles(1, :)), 'c-', 'LineWidth', 2);
hold on;
plot([0, 2, 4], rad2deg(euler_angles(1, [1,0.5*end, end])), 'r*', 'MarkerSize', 10);
xlabel('Time');
ylabel('Roll (degrees)');
title('Roll vs Time');

subplot(3, 1, 2); 
plot(time_points, rad2deg(euler_angles(2, :)), 'b-', 'LineWidth', 2);
hold on;
plot([0, 2, 4], rad2deg(euler_angles(2, [1, 0.5*end, end])), 'r*', 'MarkerSize', 10);
xlabel('Time');
ylabel('Pitch (degrees)');
title('Pitch vs Time');

subplot(3, 1, 3);
plot(time_points, rad2deg(euler_angles(3, :)), 'y-', 'LineWidth', 2);
hold on;
plot([0, 2, 4], rad2deg(euler_angles(3, [1, 0.5*end, end])), 'r*', 'MarkerSize', 10);
xlabel('Time');
ylabel('Yaw (degrees)');
title('Yaw vs Time');
saveas(gcf,'effector_rotation_diagram_pose.png')

%---------------------Making animation----------------------

video_writer = VideoWriter('robot_animation_bypose.avi');
open(video_writer);
figure;
for i = 1:num_interp_points
    SixRrobot.plot(q(:,i)');
    frame = getframe(gcf);
    writeVideo(video_writer, frame);
    clf;
end
close(video_writer);



rmpath(genpath('.'));
function R_interp = slerp_rotm(R1, R2, num_interp_points)

    euler1 = rotm2eul(R1);
    euler2 = rotm2eul(R2);
    interp_euler = zeros(num_interp_points, 3);
    for i = 1:3
        interp_euler(:, i) = linspace(euler1(i), euler2(i), num_interp_points);
    end

    R_interp = zeros(3, 3, num_interp_points);
    for i = 1:num_interp_points
        R_interp(:,:,i) = eul2rotm(interp_euler(i, :));
    end
end

