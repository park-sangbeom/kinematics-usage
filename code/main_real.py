import numpy as np 
import math 
import json 
import os 
import matplotlib.pyplot as plt 
from kinematics.class_robot import RobotClass
from model.grp import GaussianRandomPathClass, kernel_se
from model.real_robot import RealURClass
from model.realsense435 import Realsense435 
""" FOR ONROBOT RG2 """
# from pymodbus.client.sync import ModbusTcpClient
# from structure.utils.util_grasp import *
""" FOR MODERN DRIVER """
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import time 

def main(file_name, base_offset):
    real_robot=RealURClass(file_name, base_offset)
    real_robot.real_move_init()
    time.sleep(1)
    real_robot.real_move_trajectory(target_position=[0.5, 0.0, 1.2], target_rotation=[0, math.pi/2, 0])