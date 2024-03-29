#######################################################################################################
#This program takes a 3x3 Rotation Matrix as input and plots the original and rotated coordinate axes.
#Robotics ，ME3403-01 ，2023-2024-2                                                                   
#Racheus Zhao , School of Mechanical Engineering, Shanghai Jiao Tong University                       
#Chapter 1 Frame Description and Transformation                                                       
####################################################################################################

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

def get_Rotation_Matrix():
    print("Please input alpha , beta , gamma :")
    alpha, beta, gamma = input().split()
    alpha = int(alpha) / 180 * np.pi
    beta = int(beta) / 180 * np.pi
    gamma = int(gamma) / 180 * np.pi
    matrix = np.array([[np.cos(alpha)*np.cos(beta), np.cos(alpha)*np.sin(beta)*np.sin(gamma)-np.sin(alpha)*np.cos(gamma), np.cos(alpha)*np.sin(beta)*np.cos(gamma)+np.sin(alpha)*np.sin(gamma)], 
    [np.sin(alpha)*np.cos(beta), np.sin(alpha)*np.sin(beta)*np.sin(gamma)+np.cos(alpha)*np.cos(gamma), np.sin(alpha)*np.sin(beta)*np.cos(gamma)-np.cos(alpha)*np.sin(gamma)], [-np.sin(beta), np.cos(beta)*np.sin(gamma), np.cos(beta)*np.cos(gamma)]])

    print("Input Accepted!")
    return matrix



font = FontProperties(fname=r".\Carlito-Italic.ttf", size=12)


fig = plt.figure()
title = fig.suptitle('Z-Y-X Rotated Frame', fontproperties=font)


title.set_y(0.9)  

ax = fig.add_subplot(111, projection='3d')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.set_xlim([-15, 15])
ax.set_ylim([-15, 15])
ax.set_zlim([-15, 15])

ax.quiver(0, 0, 0, 10, 0, 0, color='r', arrow_length_ratio=0.1)
ax.quiver(0, 0, 0, 0, 10, 0, color='g', arrow_length_ratio=0.1)
ax.quiver(0, 0, 0, 0, 0, 10, color='b', arrow_length_ratio=0.1)

ax.xaxis.set_ticklabels(ax.get_xticks(), fontproperties=font)
ax.yaxis.set_ticklabels(ax.get_yticks(), fontproperties=font)
ax.zaxis.set_ticklabels(ax.get_zticks(), fontproperties=font)


R = get_Rotation_Matrix()
print("================= Rotation Matrix =============")
print(R)
print("===============================================")

rotated_coordsx = np.dot(R, [10, 0, 0])
rotated_coordsy = np.dot(R, [0, 10, 0])
rotated_coordsz = np.dot(R, [0, 0, 10])

ax.quiver(0, 0, 0, rotated_coordsx[0], rotated_coordsx[1], rotated_coordsx[2], color='r', arrow_length_ratio=0.1, linestyle='--')
ax.quiver(0, 0, 0, rotated_coordsy[0], rotated_coordsy[1], rotated_coordsy[2], color='g', arrow_length_ratio=0.1, linestyle='--')
ax.quiver(0, 0, 0, rotated_coordsz[0], rotated_coordsz[1], rotated_coordsz[2], color='b', arrow_length_ratio=0.1, linestyle='--')

plt.show()



