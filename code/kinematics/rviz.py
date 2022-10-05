import numpy as np 
import math 
import rospy 
from tf import transformations
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import sys
sys.path.append("..")
from kinematics.structure import CHAIN
from kinematics.robot import RobotClass
from kinematics.utils.util_ik import make_ik_input, add_joints, find_route
from kinematics.utils.util_rviz import publish_viz_robot, publish_viz_markers
from kinematics.utils.util_structure import update_q_chain, get_p_chain, get_R_chain, get_rpy_from_R_mat, get_mesh_chain, get_scale, get_link_color, get_viz_ingredients, decompose_rotation_matrix

class RvizClass:
    def __init__(self, file_name = "../urdf/ur5e/ur5e_onrobot.urdf", base_offset=[0,0,0]):
        rospy.init_node("Run_Robot")
        self.pub_robot      = rospy.Publisher('viz_robot', MarkerArray, queue_size=10)
        self.pub_obj     = rospy.Publisher('viz_objs', MarkerArray, queue_size=10)
        self.chain          = CHAIN(file_name=file_name, base_offset=base_offset, verbose=False)
        self.chain.add_joi_to_robot()
        self.chain.add_link_to_robot()
        self.ctrl_joint_num = 7

    def check_fk(self, q_list):
        update_q_chain(self.chain.joint, q_list, self.ctrl_joint_num)
        self.chain.fk_chain(1)
        p_list     = get_p_chain(self.chain.joint)
        return p_list 

    def publish_robot(self, q_list):
        update_q_chain(self.chain.joint, q_list, self.ctrl_joint_num)
        self.chain.fk_chain(1)
        p_list     = get_p_chain(self.chain.joint)
        R_list     = get_R_chain(self.chain.joint)
        rpy_list   = get_rpy_from_R_mat(R_list)
        mesh_list  = get_mesh_chain(self.chain.link)
        scale_list = get_scale(self.chain.link)
        color_list = get_link_color(self.chain.link)
        viz_links  =  get_viz_ingredients(p_list, rpy_list, mesh_list, scale_list, color_list)
        viz_trg_robot = publish_viz_robot(viz_links)
        self.pub_robot.publish(viz_trg_robot)

    def publish_markers(self, obj):
        viz_obj = publish_viz_markers(obj)
        self.pub_obj.publish(viz_obj)

def make_markers(name, type, pos, rot, size, color): 
    return {"name":name, "type":type, "info":pos+rot+size, "color":color}
