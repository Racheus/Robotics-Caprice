from dofbot import DofbotEnv
import numpy as np
import time
import copy
from scipy.spatial.transform import Rotation as R
import time
import pybullet as p
import pybullet_data
import math

class MyDOF_PID():
    def __init__(self):
        self.Kp = 0.12
        self.Ki = 0.03
        self.Kd = 0.02
        self.error = 0
        self.last_error = 0
        self.integral = 0
        self.derivative = 0
        self.output = 0
        self.target = 0

    def update(self, target, current):
        self.target = target
        self.error = target - current
        self.integral += self.error
        self.derivative = self.error - self.last_error
        self.output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        self.last_error = self.error
        return self.output

if __name__ == '__main__':
    env = DofbotEnv()
    env.reset()
    Reward = False


    '''
    constants here
    '''
    GRIPPER_DEFAULT_ANGLE = 20. / 180. * 3.1415
    GRIPPER_CLOSE_ANGLE = -20. / 180. * 3.1415

    # define state machine
    INITIAL_STATE = 0
    GRASP_STATE = 1
    LIFT_STATE = 2
    PUT_STATE = 3
    MOVE_STATE = 4
    BACK_STATE = 5
    current_state = INITIAL_STATE
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "simulation_video.mp4")

    initial_jointposes = [1.57, 0., 1.57, 1.57, 1.57]

    # offset to grasp object
    obj_offset = [-0.021, -0.021, 0.09]
    obj_offset2 = [-0.032, 0.032, 0.07]
    obj_offset3 = [-0.021, 0.020, 0.09]

    finite_pose = [0.2, -0.1, 0]
    block_pos, block_orn = env.get_block_pose()
    print("block_pos : ",block_pos)
    start_time = None
    myPID = MyDOF_PID()

    while not Reward:

        '''
        #获取物块位姿、目标位置和机械臂位姿，计算机器臂关节和夹爪角度，使得机械臂夹取绿色物块，放置到紫色区域。
        '''
        if current_state == INITIAL_STATE:
            target_pose = np.array(block_pos) + np.array(obj_offset)
            target_joint_pose = env.dofbot_setInverseKine(target_pose,-1*block_orn)
            env.dofbot_control(target_joint_pose,GRIPPER_DEFAULT_ANGLE)
            current_joint_state , gripper_angle = env.get_dofbot_jointPoses()
            temp_pose = target_pose
            if(np.sum(np.isclose(current_joint_state,target_joint_pose,atol=0.01)) == 5):
                current_state = GRASP_STATE
                # print("current_joint_state : ",current_joint_state)
                # print("target_gripper_angle : ",gripper_angle)
                # print("target_pose : ",target_joint_pose)
                print("Go to Grasp State")
        elif current_state == GRASP_STATE:
                env.dofbot_control(target_joint_pose,GRIPPER_CLOSE_ANGLE)
                if start_time is None:
                    start_time = time.time()
                current_time = time.time()
                if current_time - start_time > 2 :
                    current_state = LIFT_STATE
                    start_time = None
                    print("Go to Lift State")
        elif current_state == LIFT_STATE:
                target_pose = (temp_pose[0],temp_pose[1],temp_pose[2]+0.06)
                target_joint_pose = env.dofbot_setInverseKine(target_pose,-1*block_orn)
                env.dofbot_control(target_joint_pose,GRIPPER_CLOSE_ANGLE)
                current_joint_state , gripper_angle = env.get_dofbot_jointPoses()
                if(np.sum(np.isclose(current_joint_state,target_joint_pose,atol=0.01)) == 5):
                    print("Go to Put State")
                    current_state = PUT_STATE
                    print("temp_pose : ",temp_pose)
                    print("target_pose : ",target_pose)
                    print("target_joint_pose : ",target_joint_pose) 
                    print("current_state : ",current_joint_state)
        elif current_state == PUT_STATE:
                target_pose = np.array([0.2,-0.1,0.15])+obj_offset2
                target_joint_pose = env.dofbot_setInverseKine(target_pose,-1*block_orn)
                env.dofbot_control(target_joint_pose,GRIPPER_CLOSE_ANGLE)
                current_joint_state , gripper_angle = env.get_dofbot_jointPoses()
                if(np.sum(np.isclose(current_joint_state,target_joint_pose,atol=0.01)) == 5):
                    print(target_pose)
                    print("target_joint_pose : ",target_joint_pose)
                    print("current_joint_state : ",current_joint_state)
                    print("Go to Move State")
                    current_state = MOVE_STATE
        elif current_state == MOVE_STATE:
                target_pose = np.array([0.2,-0.1,0.015])+obj_offset3
                target_joint_pose = env.dofbot_setInverseKine(target_pose,-1*block_orn)
                env.dofbot_control(target_joint_pose,GRIPPER_CLOSE_ANGLE)
                current_joint_state , gripper_angle = env.get_dofbot_jointPoses()
                if(np.sum(np.isclose(current_joint_state,target_joint_pose,atol=0.01)) == 5):
                    print("Go to Back State")
                    print(env.get_dofbot_pose())
                    print(env.get_block_pose())
                    current_state = BACK_STATE
               
        
        elif current_state == BACK_STATE:
            pos , orn  = env.get_block_pose()
            standard = (pos[0]-0.2)**2 + (pos[1]+0.1)**2
            print("standard : ",standard)
            target = 1e-4
            if(standard > target):
                this_adjust = myPID.update(target,standard)
                target_pose = np.array([0.2,-0.1,0.015]) + this_adjust*np.array([1,1,0])
                target_joint_pose = env.dofbot_setInverseKine(target_pose,-1*block_orn)
                env.dofbot_control(target_joint_pose,GRIPPER_CLOSE_ANGLE)


        Reward = env.reward()
    p.stopStateLogging(log_id)