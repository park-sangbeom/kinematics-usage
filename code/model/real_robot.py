import numpy as np 
import math 
import json 
import os 
import matplotlib.pyplot as plt 
from kinematics.class_robot import RobotClass
from model.grp import GaussianRandomPathClass, kernel_se
""" FOR MODERN DRIVER """
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import time 

class RealURClass:
    def __init__(self, file_name= "../urdf/ur5e/ur5e_onrobot.urdf",
                       base_offset=[1.8, 0, 0.79]):
        rospy.init_node("REAL_WORLD")
        self.client      = None
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.arm_pub     = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size = 10)
        self.up_offset   = np.array([0, 0, 0.25])        
        self.file_name   = file_name
        self.base_offset = base_offset
        self.kin_robot   = RobotClass(file_name=self.file_name, 
                                      base_offset=self.base_offset)

    def get_joint_value(self, target_position=[0.6,0,1.2], 
                                traget_rotation=[0,0,0]):
        q         = self.kin_robot.solve_ik(target_name = ['wrist_3_joint'],
                    target_position  = [target_position],
                    target_rotation  = [traget_rotation],
                    solve_position   = [1],
                    solve_rotation   = [1],
                    weight_position  = 1,
                    weight_rotation  = 1,
                    joi_ctrl_num= 6)
        print("Control joint value: {} \nShape: {}".format(q, q.shape))
        return q

    def move_single_q(self, joints):
        try: 
            q = joints
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise    

    def init_pose(self):
        try: 
            q = [0.9, -0.6596, 1.3364, 0.0350, 0, 0]
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def start_pose(self):
        try: 
            q = [-3.35916187e-01, -13.90421628e-01,  2.52584599e+00, -1.13542436e+00, 1.23408381e+00, -1.59279665e-03]
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos   = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def move_trajectory(self, joint_list, num_interpol):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        for i, q in enumerate(joint_list):
            if i==0:
                joint_states = rospy.wait_for_message("joint_states", JointState)
                joints_pos   = joint_states.position
                g.trajectory.points = [
                    JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                    JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(3))]  
                d=3
            else:
                vel = (q-prev_q) #num_interpol # TODO: CHECK VELOCITY
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=q, velocities=vel,time_from_start=rospy.Duration(d))) 
            prev_q = q
            d+=0.002
        try:
            print("MOVE")
            self.client.send_goal(g)
            self.client.wait_for_result()
        except:
            raise
    
    def real_move_trajectory(self, target_position, target_rotation):
        try:
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            """ Initialize """
            self.init_pose()     
            """ Solve IK """
            q = self.get_joint_value(target_position=target_position, traget_rotation=target_rotation)
            self.move_single_q(q)
            time.sleep(1)
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def real_move_init(self):
        try:
            self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print("Waiting for server...")
            self.client.wait_for_server()
            print("Connected to server")
            """ Initialize """
            self.init_pose()     
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise