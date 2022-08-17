import numpy as np 
import rospy 
from kinematics.utils.util_ik import make_ik_input
from kinematics.class_structure import CHAIN

class RobotClass(object):
    def __init__(self, file_name, base_offset):
        super(RobotClass,self).__init__() 
        self.chain = CHAIN(file_name=file_name, base_offset=base_offset)
        self.chain.add_joi_to_robot()
        self.chain.add_link_to_robot()
        self.chain.fk_chain(1)
    
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