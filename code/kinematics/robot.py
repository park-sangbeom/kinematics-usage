import numpy as np 
import sys
sys.path.append("..")
from kinematics.utils.util_ik import make_ik_input
from kinematics.utils.util_structure import update_q_chain, get_p_chain, get_R_chain, get_rpy_from_R_mat, get_mesh_chain, get_scale, get_link_color, get_viz_ingredients, decompose_rotation_matrix
from kinematics.structure import CHAIN

class RobotClass(object):
    def __init__(self, file_name, base_offset, ctrl_joint_num=7):
        super(RobotClass,self).__init__() 
        self.chain = CHAIN(file_name=file_name, base_offset=base_offset)
        self.chain.add_joi_to_robot()
        self.chain.add_link_to_robot()
        self.chain.fk_chain(1)
        self.ctrl_joint_num = ctrl_joint_num

    def solve_ik(self,  target_name      = ['wrist_3_joint'],
                        target_position  = [[0,0,0]],
                        target_rotation  = [[-1.57, 0, -1.57]],
                        solve_position   = [1],
                        solve_rotation   = [1],
                        weight_position  = 1,
                        weight_rotation  = 1,
                        joi_ctrl_num= 6): 
                        
        ingredients = self.make_ik_input(target_name = target_name,
                                    target_pos  = target_position,
                                    target_rot  = target_rotation,
                                    solve_pos   = solve_position,
                                    solve_rot   = solve_rotation,
                                    weight_pos  = weight_position,
                                    weight_rot  = weight_rotation,
                                    joi_ctrl_num= joi_ctrl_num)
        total_q = self.chain.get_q_from_ik(ingredients)
        ctrl_q  = total_q[:joi_ctrl_num+1] #Including base joint for Unity Env
        ctrl_q  = ctrl_q.reshape(-1,)
        return ctrl_q 

    def make_ik_input(self, target_name=["base_joint"], 
                    target_pos=[[0, 0, 0]], 
                    target_rot=[[0, 0, 0]], 
                    solve_pos=[1], 
                    solve_rot=[1], 
                    weight_pos=1, 
                    weight_rot=0, 
                    disabled_joi_id=[], 
                    joi_ctrl_num=7):
        return {"target_joint_name":target_name, 
                "target_joint_position":target_pos,
                "target_joint_rotation":target_rot,
                "solve_position":solve_pos,
                "solve_rotation":solve_rot,
                "position_weight":weight_pos,
                "rotation_weight":weight_rot,
                "disabled_joi_id":disabled_joi_id, 
                "joint_ctrl_num":joi_ctrl_num}
    
    def is_collision(self, q_list, objs=None):
        update_q_chain(self.chain.joint, q_list, self.ctrl_joint_num)
        self.chain.fk_chain(1)
        cap_R_list = get_cap_R_chain(self.chain.link)
        rpy_list = get_rpy_from_R_mat(cap_R_list)
        link_p_list = get_center_p_chain(self.chain.link)
        name_list = get_name_chain(self.chain.link)
        size_list = get_cap_size_chain(self.chain.link)
        scale_list = get_scale_chain(self.chain.link)                  
        height_list = get_height_chain(self.chain.link)
        radius_list = get_cap_radius(self.chain.link)