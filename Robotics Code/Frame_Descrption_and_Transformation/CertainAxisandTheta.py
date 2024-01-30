#######################################################################################################
#This program takes a certain axis and theta as input and plots the original and rotated coordinate axes.
#Robotics ，ME3403-01 ，2023-2024-2                                                                   
#Racheus Zhao , School of Mechanical Engineering, Shanghai Jiao Tong University                       
#Chapter 1 Frame Description and Transformation                                                       
####################################################################################################

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

def get_Rotation_Matrix():
    print("Please input the axis(x & y & z):")                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    axis = input()
    print("Please input the theta(0 - 180 deg):")
    theta = int(input())
    theta = theta / 180 * np.pi
    
    if axis == 'x':
        matrix = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
    elif axis == 'y':
        matrix = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
    elif axis == 'z':
        matrix = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])

    return matrix

font = FontProperties(fname=r".\Carlito-Italic.ttf", size=12)


fig = plt.figure()
title = fig.suptitle('Rotated Frame', fontproperties=font)


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
print("============== Rotation Matrix ===============")
print(R)
print("==============================================")
      
rotated_coordsx = np.dot(R, [10, 0, 0])
rotated_coordsy = np.dot(R, [0, 10, 0])
rotated_coordsz = np.dot(R, [0, 0, 10])

ax.quiver(0, 0, 0, rotated_coordsx[0], rotated_coordsx[1], rotated_coordsx[2], color='r', arrow_length_ratio=0.1, linestyle='--')
ax.quiver(0, 0, 0, rotated_coordsy[0], rotated_coordsy[1], rotated_coordsy[2], color='g', arrow_length_ratio=0.1, linestyle='--')
ax.quiver(0, 0, 0, rotated_coordsz[0], rotated_coordsz[1], rotated_coordsz[2], color='b', arrow_length_ratio=0.1, linestyle='--')

plt.show()



