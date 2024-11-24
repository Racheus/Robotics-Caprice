<center>
    <font size=12>
        Robotics Caprice
    </font>
</center>


<center>
ME3403 , Robotics , 2023-2024-2 & 2024-2025-1
<center>
    Racheus Zhao , school of Mechanic Enginnering ,SJTU
</center>

<center>
    <img alt="Static Badge" src="https://img.shields.io/badge/MATLAB-2023b-green">
    <img alt="PyPI - Python Version" src="https://img.shields.io/pypi/pyversions/roboticstoolbox-python">
	<img alt="GitHub last commit" src="https://img.shields.io/github/last-commit/Racheus/Robotics-Caprice">
    <img alt="GitHub License" src="https://img.shields.io/github/license/Racheus/Robotics-Caprice">
</center>

---
莽红尘，何处觅知音？青衫湿。
<p align="right">————秋瑾《满江红》</p>

---
This is a project I 've thought so many times. For piece of love, passion, and hope.I really want to mark some shining time in my short-and-tired undergraduate life.This repo contains a lot of notes about basic Robotics in Chinese(always,always and always aiming at examination,huh~),some python and matlab script as well.

I'm not certain about how long I can insist.🤕🤕🤕For the reason that I 'm just a regular student and have such a lot of courses in one semester.😭😭😭

However I will try my best to maintain it.

---
### Chapter 1 Mathematical Foundations of Robotics
Notes PDF edition:
[Chapter1_机器人学的数学基础](https://github.com/Racheus/Robotics-Caprice/blob/master/Chapter1_%E6%95%B0%E5%AD%A6%E5%9F%BA%E7%A1%80/%E6%95%B0%E5%AD%A6%E5%9F%BA%E7%A1%80.pdf)

Code in this chpt: about rotation matrix & different rotation methods:

[Chapter1_Code for Python and MATLAB](https://github.com/Racheus/Robotics-Caprice/tree/master/Robotics%20Code/Frame_Descrption_and_Transformation)

Example for Chapter 1:

![Insert Error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Images/Figure_1.svg)



**Homework #1**:2R-Robotics-Kinematics

Report PDF edition：

[Homework1: 2R-robotics-Kinematics](https://github.com/Racheus/Robotics-Caprice/blob/master/Homework1-2Rrobot-Kine/RoboticsHomework1.pdf)

Results:

MATLAB Version:

![Insert error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Homework1-2Rrobot-Kine/src/robot_trajectory.gif)

Python Version:

![Insert Error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Homework1-2Rrobot-Kine/src/robot_trajectory_py.gif)

**Homework #2**:[Homework2: Math foundations](https://github.com/Racheus/Robotics-Caprice/blob/master/Homework2-Mathematic-foundations/RoboticsHomework2.pdf)

**Homework #3**:[Homework3:Kinematics](https://github.com/Racheus/Robotics-Caprice/blob/master/Homework3-Kinematics/RoboticsHomework3.pdf)(May have errors!)

**Homework#4**: Dynamics(为什么我的眼里常含泪水？)

**Homework #5**：[homework5_Kinematic_Trajectory_Planning](https://github.com/Racheus/Robotics-Caprice/blob/master/Homework5-Kinematic-Trajectory-Planning/RoboticsHomework5.pdf)

Results: From joint operation space:

![Insert Error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Homework5-Kinematic-Trajectory-Planning/src/angle_spline_diagram.png)

Animation demo:

![Insert Error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Homework5-Kinematic-Trajectory-Planning/src/robot_animation.gif)

If planning it by pose space, the robot will have some *configuration mutation points*,which may harmful in practical engineering applications.(Maybe my method have something wrong?**MARK here**)

![Insert Error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Homework5-Kinematic-Trajectory-Planning/src/robot_animation_bypose.gif)



---

### Chapter 2 Final works of Robotics 2023-2024-2（ME3403）

This time we make a model from *Solidworks*. After using Solidworks to build the model, we used the relevant theories of robotics to calculate the reachable space, kinematics (forward and inverse), dynamics (forward and inverse) and other functions in MATLAB, and finally combined Simulink to simulate the kinematics and dynamics.

![Insert Error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Images/logo.png)

We took inspiration from the ABB IRB 1200 and built a model ourselves in Solidworks.

![Insert Error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Images/Model.jpg)

And we calculate the workspace by Traditional Monte Carlo method and Improved Monte Carlo method.

![Insert Error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Images/workspace.jpg)

We build a model in Simulink with our rigid model to simulate the Kinematics and Dynamics issues.

![Insert Error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Images/ABBIRB.png)

## 

---

### Chapter 3 Final works of Robotics 2024-2025-1（AU3307）

This part focuses on an actual robot, DOFBOT. It is a small robot composed of 5 rotating joints.

![Insert Error!](https://github.com/Racheus/Robotics-Caprice/blob/master/Images/DOFBOT.png)

Perform simulation tasks in the *Pybullet* simulation environment and implement the robot's grasping of objects offline.

Environment base:

Robotic-Toolbox-Python: [https://github.com/petercorke/robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python)

Pybullet Documentation: [Pybullet](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#)

A demo of pybullet-simulation : grasping

https://github.com/user-attachments/assets/dead7a45-c393-4056-9aa5-e25e425fe059



Offline experiment:

https://github.com/user-attachments/assets/ee737601-054e-4ef9-b9b1-49a3318c01ea

