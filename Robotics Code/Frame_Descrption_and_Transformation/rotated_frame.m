%Rotation Matrix.m
%Robotics , 2023-2024-2 , ME3403-01
%Racheus Zhao , School of Mechanic Engineering , SJTU
%Chapter 1 Frame Description and Transformation  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rotate_frame();

function rotate_frame()
    % 获取旋转矩阵
    R = get_Rotation_Matrix();
    
    % 创建图形窗口
    fig = figure;
    
    % 设置标题和坐标轴标签
    title('Rotated Frame');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    % 设置坐标轴范围
    xlim([-15, 15]);
    ylim([-15, 15]);
    zlim([-15, 15]);
    
    ax = gca;
    ax.FontName = 'Carlito-Italic.ttf';
    ax.FontSize = 12;
    
    hold on;
    
    % 绘制原始的坐标轴
    quiver3(0, 0, 0, 10, 0, 0, 'r', 'LineWidth', 1.5);
    quiver3(0, 0, 0, 0, 10, 0, 'g', 'LineWidth', 1.5);
    quiver3(0, 0, 0, 0, 0, 10, 'b', 'LineWidth', 1.5);
    
    % 对旋转矩阵进行正交化处理
    [Q, ~] = qr(R);
    
    % 计算旋转后的坐标轴
    rotated_coordsx = Q * [10; 0; 0];
    rotated_coordsy = Q * [0; 10; 0];
    rotated_coordsz = Q * [0; 0; 10];
    
    % 绘制旋转后的坐标轴
    quiver3(0, 0, 0, rotated_coordsx(1), rotated_coordsx(2), rotated_coordsx(3), 'r--', 'LineWidth', 1.5);
    quiver3(0, 0, 0, rotated_coordsy(1), rotated_coordsy(2), rotated_coordsy(3), 'g--', 'LineWidth', 1.5);
    quiver3(0, 0, 0, rotated_coordsz(1), rotated_coordsz(2), rotated_coordsz(3), 'b--', 'LineWidth', 1.5);
    
    hold off;
    
    % 显示图形
    view(3);
    grid on;
end

function R = get_Rotation_Matrix()
    prompt = 'Please input the Rotation Matrix (start with "[",rows separated by ";"): ';
    matrix = input(prompt);
    
    R = reshape(matrix, 3, 3);
    
    disp('Input Accepted!');
    disp(R);
end


% 调用函数绘制旋转后的坐标轴
