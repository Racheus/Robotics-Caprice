import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb

dofbot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=1.0),
        rtb.RevoluteMDH(alpha=-np.pi/2, offset=-np.pi/2),
        rtb.RevoluteMDH(a=1.0),
        rtb.RevoluteMDH(a=1.0, offset=np.pi/2),
        rtb.RevoluteMDH(alpha=np.pi/2, d=1.0)
    ]
)

# Each joint's angle limits
joint_limits = {
    0: (-np.pi, np.pi),  
    1: (-np.pi/2, np.pi/2), 
    2: (-np.pi, np.pi), 
    3: (-np.pi, np.pi),  
    4: (-np.pi/2, np.pi/2),  
}
# 均匀采样结果

num_samples = 10  
joint_angles = []

for j in range(5):
    joint_min, joint_max = joint_limits[j]
    joint_angles.append(np.linspace(joint_min, joint_max, num_samples))

# 网格化关节角度（5个关节的笛卡尔积）
joint_angle_grid = np.array(np.meshgrid(*joint_angles)).T.reshape(-1, 5)


end_effector_positions = []

for angles in joint_angle_grid:
    T = dofbot.fkine(angles)
    end_effector_positions.append(T.t)  
end_effector_positions = np.array(end_effector_positions)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(end_effector_positions[:, 0], end_effector_positions[:, 1], end_effector_positions[:, 2], s=1)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Robot Workspace')

plt.show()


#不均匀（正态）采样结果

# joint_angles_1=[]
# for j in range(5):
#     joint_min, joint_max = joint_limits[j]
#     joint_angles_1.append(np.random.uniform(joint_min, joint_max, num_samples))


# joint_angle_grid = np.array(np.meshgrid(*joint_angles)).T.reshape(-1, 5)
# end_effector_positions = []

# for angles in joint_angle_grid:
#     T = dofbot.fkine(angles)
#     end_effector_positions.append(T.t) 

# end_effector_positions = np.array(end_effector_positions)

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(end_effector_positions[:, 0], end_effector_positions[:, 1], end_effector_positions[:, 2], s=0.5)

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('Robot Workspace')

# plt.show()