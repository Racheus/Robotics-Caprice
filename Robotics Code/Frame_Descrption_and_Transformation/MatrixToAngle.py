#######################################################################################################
#This program takes a 3x3 Rotation Matrix as input , converts it to axis and theta, and plots the original and rotated coordinate axes.
#Robotics ，ME3403-01 ，2023-2024-2                                                                   
#Racheus Zhao , School of Mechanical Engineering, Shanghai Jiao Tong University                       
#Chapter 1 Frame Description and Transformation                                                       
####################################################################################################

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

def get_Rotation_Matrix():
    print("Please input the Rotation Matrix (3x3, separated by spaces):")
    elements = input().split()
    matrix = np.fromstring(' '.join(elements), dtype=int, sep=' ').reshape(3, 3)

    print("Input Accepted!Rotation matrix is :")
    print(matrix)
    return matrix

####X-Y-Z(RPY) Fixed Angle###
def get_RPY(matrix):
    if matrix[2, 0] == 1:
        alpha = 0
        beta = 0.5 * np.pi
        gamma = np.arctan2(matrix[0, 1], matrix[1, 1])
    elif matrix[2, 0] == -1:
        alpha = 0
        beta = -0.5 * np.pi
        gamma = - np.arctan2(matrix[0, 1], -matrix[1, 1])
    else:
        beta = np.arctan2(-matrix[2,0], np.sqrt(matrix[0,0]**2 + matrix[1,0]**2))
        alpha = np.arctan2(matrix[1,0]/np.cos(beta), matrix[0,0]/np.cos(beta))
        gamma = np.arctan2(matrix[2,1]/np.cos(beta), matrix[2,2]/np.cos(beta))
    
    alpha = np.rad2deg(alpha)
    beta = np.rad2deg(beta)
    gamma = np.rad2deg(gamma)

    return alpha, beta, gamma

####Z-Y-Z Euler Angle###
def get_ZYZ_Euler(matrix):
    if matrix[2, 2] == 1:
        alpha = 0
        beta = 0
        gamma = np.arctan2(-matrix[0, 1], matrix[0, 0])
    elif matrix[2, 2] == -1:
        alpha = 0
        beta = np.pi
        gamma = np.arctan2(matrix[0, 1], - matrix[0, 0])
    else:
        beta = np.arctan2(np.sqrt(matrix[2,0]**2 + matrix[2,1]**2),matrix[2,2])
        alpha = np.arctan2(matrix[1,2]/np.sin(beta), matrix[0,2]/np.sin(beta))
        gamma = np.arctan2(matrix[2,1]/np.sin(beta), -matrix[2,0]/np.sin(beta))

    alpha = np.rad2deg(alpha)
    beta = np.rad2deg(beta)
    gamma = np.rad2deg(gamma)
    return alpha, beta, gamma

####main function###
if __name__ == '__main__':
    matrix = get_Rotation_Matrix()
    print("******************************************************************")
    print("X-Y-Z(RPY) Fixed Angle expression:")
    alpha, beta, gamma = get_RPY(matrix)
    print("alpha = ", alpha, "beta = ", beta, "gamma = ", gamma)
    
    print("******************************************************************")
    print("Z-Y-Z Euler Angle expression:")
    alpha, beta, gamma = get_ZYZ_Euler(matrix)
    print("alpha = ", alpha, "beta = ", beta, "gamma = ", gamma)
    print("******************************************************************")