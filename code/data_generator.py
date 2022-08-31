import numpy as np 
import math 
import json 
import os 
import matplotlib.pyplot as plt 
from kinematics.class_robot import RobotClass
from model.grp import GaussianRandomPathClass, kernel_se

def main_one_step(json_name):
    file_name = "../code/urdf/ur5e/ur5e_onrobot.urdf"
    base_offset = [0.18,0,0.79]
    robot     = RobotClass(file_name=file_name, base_offset=base_offset)    
    robot.chain.joint[2].q =1
    x         = 1.0
    q_start   = robot.solve_ik(target_name = ['wrist_3_joint'],
                target_position  = [[0.7,0,1.2]],
                target_rotation  = [[-math.pi/2, 0, math.pi/2]],
                solve_position   = [1],
                solve_rotation   = [10],
                weight_position  = 1,
                weight_rotation  = 1,
                joi_ctrl_num= 6)
    for traj_idx in range(1000):
        print(traj_idx)
        y         = np.random.uniform(-0.3,0.3)
        q_last    = robot.solve_ik(target_name = ['wrist_3_joint'],
                    target_position  = [[x,y,1.2]],
                    target_rotation  = [[-math.pi/2, 0, math.pi/2]],
                    solve_position   = [1],
                    solve_rotation   = [10],
                    weight_position  = 1,
                    weight_rotation  = 1,
                    joi_ctrl_num= 6)
        q_trajs = np.linspace(q_start, q_last, 20)
        for step, q in enumerate(q_trajs): 
            result = {"num_traj":traj_idx+1,
                        "num_step":step+1,
                        "q":q[1:].tolist()}        
            with open(json_name,"a") as f:
                f.write(json.dumps(result)+'\n')

def main_np(json_name):
    file_name = "../code/urdf/ur5e/ur5e_onrobot.urdf"
    base_offset = [0.18,0,0.79]
    robot     = RobotClass(file_name=file_name, base_offset=base_offset)
    grp = GaussianRandomPathClass(name = 'GRP',kernel = kernel_se)
    last_position = 0
    t_test        = np.linspace(start=0.6,stop=1.0,num=20).reshape((-1,1))
    x_anchor1     = np.array([0, last_position])

    total_anchor_num = len(x_anchor1)
    grp.set_data(t_anchor    = np.linspace(start=0.6,stop=1.0,num=total_anchor_num).reshape((-1,1)),
            x_anchor    = np.array([x_anchor1]).T,
            t_test      = t_test,
            hyp_mean    = {'g':0.06,'l':0.1,'w':1e-6},
            hyp_var     = {'g':0.06,'l':0.1,'w':1e-6},
            APPLY_EPSRU = False,
            )
    trajs, t_test = grp.sample(n_sample=1000)

    for traj_idx, traj in enumerate(trajs):
        print("traj_idx", traj_idx)
        plt.plot(t_test, traj)
        for step, (x, y) in enumerate(zip(t_test, traj)):
            q         = robot.solve_ik(target_name = ['wrist_3_joint', 'gripper_finger1_finger_tip_joint'],
                        target_position  = [[x,y,0.82], [x+0.15,y,0.82]],
                        target_rotation  = [[0, math.pi, -math.pi/2],[0,0,0]],
                        solve_position   = [1, 1],
                        solve_rotation   = [1, 0],
                        weight_position  = 1,
                        weight_rotation  = 1,
                        joi_ctrl_num= 6)
            result = {"num_traj":traj_idx+1,
                      "num_step":step+1,
                      "q":q[1:].tolist()}
            with open(json_name,"a") as f:
                f.write(json.dumps(result)+'\n')
    plt.ylim(-0.42, 0.42)
    plt.xlabel("X axis")
    plt.ylabel("Y axis")
    plt.title("Sampled trajecotires", fontsize=15)
    plt.savefig("sampled_trajs.png")

if __name__ == "__main__":
    PATH     = "data/one_step/"
    json_name = "data/one_step/manipulator_traj_one_step.json"
    if not os.path.exists(PATH):
        os.makedirs(PATH)
    main_one_step(json_name)