clear;
clc;
addpath(genpath('.'));

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

q1=SixRrobot.ikine(T1);
q2=SixRrobot.ikine(T2);
q3=SixRrobot.ikine(T3);

disp(q1);
disp(q2);
disp(q3);

num_waypoints = 200;
time_points = linspace(0, 4, num_waypoints);

Q = zeros(num_waypoints, 6);
for i = 1:6
    Q(:,i) = spline([0, 2, 4], [q1(i), q2(i), q3(i)], linspace(0, 4, num_waypoints));
end

figure;
sgtitle('Angle diagram of each joint under \color{red}joint spline');
for i = 1:6
    subplot(3, 2, i);
    plot(time_points, Q(:,i), '-');
    hold on;
    plot([0, 2, 4], [q1(i), q2(i), q3(i)], 'r.', 'MarkerSize', 10);
    hold off;
    xlabel('Time');
    ylabel(['Joint ', num2str(i), ' Angle']);
    title(['Joint ', num2str(i), ' Angle vs Time']);
end
saveas(gcf,'angle_spline_diagram.png')

translation_vectors = zeros(3, num_waypoints);
euler_angles = zeros(3,num_waypoints);
for i = 1:num_waypoints
    qi = Q(i,:);
    end_effector_pose = SixRrobot.fkine(qi);
    translation_vectors(:, i) = end_effector_pose.t;
    R = end_effector_pose.R;
    euler_angles(:,i) = tr2rpy(R);
end

% 绘制每个方向的平移向量随时间的图表
figure;
sgtitle('Translation vectors of effector under \color{red}joint spline');
subplot(3, 1, 1); % 创建一个3行1列的图表，并指定第1个子图
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

saveas(gcf,'effector_trans_diagram.png')



figure;
sgtitle('Rotation vectors of effector under \color{red}joint spline(RPY)');
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
saveas(gcf,'effector_rotation_diagram.png')


video_writer = VideoWriter('robot_animation.avi');
open(video_writer);
figure;
for i = 1:size(Q, 1)
    SixRrobot.plot(Q(i, :));
    frame = getframe(gcf);
    writeVideo(video_writer, frame);
    clf;
end
close(video_writer);


rmpath(genpath('.'));