import roboticstoolbox as rtb
import numpy as np
import math
import matplotlib
from roboticstoolbox import ctraj

matplotlib.use('Qt5Agg')
from spatialmath.base import *

pi = 3.1415926          # 定义pi常数

l1 = 0.104             # 定义第一连杆长度
l2 = 0.08285            # 定义第三连杆长度
l3 = 0.08285            # 定义第四连杆长度
l4 = 0.07404      # 定义第五连杆长度




# student version
# 用改进DH参数发表示机器人正运动学
# TODO: modify the dh param
dofbot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=l1),
        rtb.RevoluteMDH(alpha=-pi/2,offset=-pi/2),
        rtb.RevoluteMDH(a=l2),
        rtb.RevoluteMDH(a=l3,offset=pi/2),   #旋转关节
        rtb.RevoluteMDH(alpha=pi/2,d=l4)   #平动关节
    ]
)

# 输出机器人DH参数矩阵
print(dofbot)
dofbot.plot([0,0,0,0,0],block=True)

'''
Part1 给出一下关节姿态时的机械臂正运动学解，并附上仿真结果
0.(demo) [0., pi/3, pi/4, pi/5, 0.]
1.[pi/2, pi/5, pi/5, pi/5, pi]
2.[pi/3, pi/4, -pi/3, -pi/4, pi/2]
3.[-pi/2, pi/3, -pi/3*2, pi/3, pi/3]
'''

fkine_input0 = [0., pi/3, pi/4, pi/5, 0.]
fkine_result0 = dofbot.fkine(fkine_input0)
print(fkine_result0)
dofbot.plot(q=fkine_input0, block=True)

fkine_input1 = [pi/2, pi/5, pi/5, pi/5, pi]
fkine_result1 = dofbot.fkine(fkine_input1)
print(fkine_result1)
dofbot.plot(q=fkine_input1, block=True)

fkine_input2 = [pi/3, pi/4, -pi/3, -pi/4, pi/2]
fkine_result2 = dofbot.fkine(fkine_input2)
print(fkine_result2)
dofbot.plot(q=fkine_input2, block=True)

fkine_input3 = [-pi/2, pi/3, -pi/3*2, pi/3, pi/3]
fkine_result3 = dofbot.fkine(fkine_input3)
print(fkine_result3)
dofbot.plot(q=fkine_input3, block=True)

# #part2
target_pos0 = np.array([
        [-1., 0., 0., 0.1,],
        [0., 1., 0., 0.],
        [0., 0., -1., -0.1],
        [0., 0., 0., 1.] 
])
ikine_result0 = dofbot.ik_LM(target_pos0)[0]
print("ikine: ", np.array(ikine_result0))
dofbot.plot(q=ikine_result0, block=True)

target_pose_1 = np.array([
    [1., 0., 0., 0.1,],
    [0., 1., 0., 0.],
    [0., 0., 1., 0.1],
    [0., 0., 0., 1.]
])
ikine_result1 = dofbot.ik_LM(target_pose_1)[0]
print("ikine: ", np.array(ikine_result1))
dofbot.plot(q=ikine_result1, block=True)


target_pose2 = np.array([
    [np.cos(pi/3), 0., -np.sin(pi/3), 0.05,],
    [0., 1., 0., 0.03],
    [np.sin(pi/3), 0., np.cos(pi/3), -0.1],
    [0., 0., 0., 1.]
])
ikine_result2 = dofbot.ik_LM(target_pose2)[0]
print("ikine: ", np.array(ikine_result2))
dofbot.plot(q=ikine_result2, block=True)

target_pose3 = np.array([
    [-0.866, -0.25, -0.433, -0.03704,],
    [0.5, -0.433, -0.75, -0.06415],
    [0., -0.866, 0.5, 0.3073],
    [0., 0., 0., 1.]
])
ikine_result3 = dofbot.ik_LM(target_pose3)[0]
print("ikine: ", np.array(ikine_result3))
dofbot.plot(q=ikine_result3, block=True)


