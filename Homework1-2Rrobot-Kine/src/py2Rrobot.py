#  ME3403-01 , Robotics Homework 1-4
#  Python Version
#  Package needed: numpy, matplotlib
#  赵四维 , 521021910696
#  Copyright 2023-2024-2 Spring Racheus Zhao


import numpy as np
import imageio
import os
import matplotlib.pyplot as plt


def get_q(l1,l2,x,y):
    D = (x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2)
    theta2_1 = np.arctan2(-np.sqrt(1-D**2),D)
    theta1_1 = np.arctan2(y,x) - np.arctan2(l2*np.sin(theta2_1),l1+l2*np.cos(theta2_1))
    theta2_2 = np.arctan2(np.sqrt(1-D**2),D)
    theta1_2 = np.arctan2(y,x) - np.arctan2(l2*np.sin(theta2_2),l1+l2*np.cos(theta2_2))
    return [theta1_1,theta2_1],[theta1_2,theta2_2]

def norm_dist(x,y):
    return np.sqrt(x**2 + y**2)

def check_q(q1,q2,q_now):
    d1 = norm_dist(q1[0]-q_now[0],q1[1]-q_now[1])
    d2 = norm_dist(q2[0]-q_now[0],q2[1]-q_now[1])
    if d1 < d2:
        return q1
    else:
        return q2

def plot_robot(l1,l2,theta1,theta2, trajectory):
    x1 = 0
    y1 = 0
    x2 = l1*np.cos(theta1)
    y2 = l1*np.sin(theta1)
    x3 = x2 + l2*np.cos(theta1+theta2)
    y3 = y2 + l2*np.sin(theta1+theta2)
    plt.title('2R Robot Trajectory Simulation')
    plt.plot([x1,x2,x3],[y1,y2,y3],'o-')
    plt.pause(0.1)
    trajectory.append([x3, y3])  # 将当前位置添加到轨迹中

if __name__ == "__main__":
    l1 = 2
    l2 = 1
    q_now = [0,0]
    trajectory = []  # 存储机器人的轨迹

    for t in np.linspace(0,1,100):
        x = 4.5 - 3 * np.sin((np.pi/3)*t + np.pi/6)
        y = (3 * np.sqrt(3) / 2) - 3 * np.cos((np.pi/3)*t + np.pi/6)
        q1 , q2 = get_q(l1,l2,x,y)
        q_now = check_q(q1,q2,q_now)
        plot_robot(l1,l2,q_now[0],q_now[1], trajectory)

        # Save each frame as an image
        plt.savefig(f'frame_{t}.png')

    # Create a GIF from the saved frames
    with imageio.get_writer('robot_trajectory.gif', mode='I') as writer:
        for t in np.linspace(0,1,100):
            image = imageio.imread(f'frame_{t}.png')
            writer.append_data(image)

    trajectory = np.array(trajectory)
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'r-')
    plt.xlim(-l1-l2,l1+l2)
    plt.ylim(-l1-l2,l1+l2)
    # Delete all the saved frames
    for t in np.linspace(0,1,100):
        filename = f'frame_{t}.png'
        if os.path.exists(filename):
            os.remove(filename)