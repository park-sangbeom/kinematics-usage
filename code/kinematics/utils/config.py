import numpy as np 
# Base link 
base_link = {"name":"base_link", "size": [0.225630, 0.189169, 0.140000], "height":0.0957, "radius":0.0165, #"height":0.189169, "radius":0.225630,#
            "T_offset":np.array([[0.363900908988889, 0.496963178867832, 0.787784061330667, -3.614658268431305e-05],
                                [0.496963178867832, 0.611739106940326, 0.615469628730315, -1.875989353224022e-04],
                                [0.787784061330667, 0.615469628730315, 0.024359984070784, 0.038693440073181],
                                [0, 0, 0, 1]])}
# Shoulder link 
shoulder_link = {"name":"shoulder_link", "size": [0.110016, 0.184406, 0.247002], "height":0.0110, "radius":0.0897, #"height":0.184406, "radius":0.110016,#
            "T_offset":np.array([[0.999876675449620, 0.015034217093482, 0.004539406150671, 1.480307594761187e-05], 
                                [0.015034217093482, -0.832787412709144, -0.553388739991695, 0.009757553957810], 
                                [-0.0045, 0.5534, -0.8329, -0.0040], 
                                [0, 0, 0, 1]])}
# Upperarm link 
upper_arm_link = {"name":"upper_arm_link", "size": [0.110033, 0.249024, 0.184393], "height":0.4120, "radius":0.0944, #"height":0.249024, "radius":0.110033,#
            "T_offset":np.array([[1, 9.685099510399192e-08, 1.820334514398292e-04, -3.687429068974174e-05], 
                            [9.685099510399192e-08, 1, -0.001064100758520, -0.009033312051339],
                            [-1.820334514398292e-04, 0.001064100758520, 0.999999417276529, 0.2185], 
                            [0, 0, 0, 1]])}
# Forearm link 
forearm_link = {"name":"forearm_link", "size": [0.192511, 0.166063, 0.176002], "height":0.3880, "radius":0.0698, #"height":0.166063, "radius":0.192511,#
            "T_offset":np.array([[0.999999948539096, -7.695183022168247e-06, -3.207219803874709e-04, -5.650462263822941e-05],
                                [-7.695183022168247e-06, 0.998849304275560, 0.047959016764460, 0.011556718817272], 
                                [3.207219803874709e-04, 0.047959016764460, 0.998849252814657, 0.192971090193446],
                                [0, 0, 0, 1] ])}
# Wrist1 link
wrist_1_link = {"name":"wrist_1_link", "size": [0.192507, 0.179000, 0.166053], "height":0.0083, "radius":0.0731, #"height":0.179000, "radius":0.192507,#
            "T_offset":np.array([[0.999307897534311, -0.004213859281416, 0.037, 4.879510550714419e-05], 
                                [-0.0042, 0.9743, 0.2250, 0.1123], 
                                [-0.037, -0.2250, 0.9737, 0.0016],
                                [0, 0, 0, 1]])}
# Wrist2 link
wrist_2_link = {"name":"wrist_2_link", "size": [0.109996, 0.184930, 0.311199], "height":0.0186, "radius":0.0602, #"height":0.184930, "radius":0.109996,#
            "T_offset":np.array([[0.9998, 0.0134, -0.0151, 1.968906132085006e-05],
                                [0.0134, 0.1194, 0.9928, 0.0013],
                                [0.0151, -0.9928, 0.1192, 0.0971],
                                [0, 0, 0, 1]])}
# Wrist3 link 
wrist_3_link = {"name":"wrist_3_link", "size": [0.179926, 0.132863, 0.100243], "height":0.0026, "radius":0.0443, #"height":0.132863, "radius":0.179926,#
            "T_offset":np.array([[0.9994, -0.0025, -0.0341, -8.856842280769180e-05],
                                [-0.0025, 0.9896, -0.1441, 0.0714],
                                [0.0341 , 0.1441, 0.9890, 6.174182054691084e-04],
                                [0, 0, 0, 1]])}
# Mount link 
mount_link = {"name":"mount_link", "size": [0., 0., 0.], "height":0.0564, "radius":0.0491, #"height":0.125297, "radius":0.125333,#
            "T_offset":np.array([[-0.3890, -0.9188, -0.0667, 0.0020],
                    [-0.9188, 0.3923, -0.0441, 0.0038],
                    [-0.0667 , -0.0441, -0.9968, 0.0282],
                    [0, 0, 0, 1]])}

camera_base_link = {"name":"camera_base_link", "size": [0.09, 0.02, 0.02], "height":0.056, "radius":0.025,
                "T_offset":np.array([[-0.0024, -0.0064, 1, 2.251605357981266e-04],
                    [-0.0064, 1, -0.0064, -0.0055],
                    [-1 , -0.0064, -0.0024, 5.713196902388803e-05],
                    [0, 0, 0, 1]])}

gripper_base_link = {"name":"gripper_base_link", "size": [0., 0., 0.], "height":0.0784, "radius":0.0384,
            "T_offset":np.array([[0.999987485555153, 2.814043238879283e-05, 0.0050, 4.583645816269062e-05],
                                [2.814043238879283e-05, 0.9999, -0.0112, -3.276256850283257e-04],
                                [-0.0050 , 0.0112, 0.9999, 0.0578],
                                [0, 0, 0, 1]])}

gripper_tcp_link={"name":"gripper_tcp_link", "size": [0., 0., 0.], "height":0.0, "radius":0.0,
            "T_offset":np.eye(4)}       

gripper_finger1_finger_link = {"name":"gripper_finger1_finger_link", "size": [0., 0., 0.], "height":0.0461, "radius":0.0185,
            "T_offset":np.array([[0.9998, -5.709402026350889e-07, 0.0200, 9.173590961243574e-04],
                                [-5.709402026350889e-07, 1, 5.716492737084250e-09, 0.0103],
                                [-0.0200 , -5.716492737084250e-09, 0.9998, 0.0224],
                                [0, 0, 0, 1]])}

# gripper-finger1-finger-link
gripper_finger1_inner_knuckle_link = {"name":"gripper_finger1_inner_knuckle_link", "size": [0., 0., 0.], "height":0.0486, "radius":0.0159,
            "T_offset":np.array([[-0.9993, -1.470439193356746e-06, -0.0364, 8.839410521185181e-04],
                                [-1.470439193356746e-06, 1, -2.674468038488923e-08, 0.0100],
                                [0.0364 , 2.674468038488923e-08, -0.9993, 0.0243],
                                [0, 0, 0, 1]])}
#gripper_finger1_inner_knuckle

gripper_finger1_finger_tip_link = {"name":"gripper_finger1_finger_tip_link", "size": [0., 0., 0.], "height":0.0429, "radius":0.0128,
            "T_offset":np.array([[0.9579, 1.625202823558144e-09, -0.2870, -0.0014],
                                [1.625202823558144e-09, 1, 1.108750919739873e-08, 0.0058],
                                [0.2870 , -1.108750919739873e-08, 0.9579, 0.0054],
                                [0, 0, 0, 1]])}
#gripper_finger1_finger_tip_link

gripper_finger2_finger_link = {"name":"gripper_finger2_finger_link", "size": [0., 0., 0.], "height":0.0461, "radius":0.0185,
            "T_offset":np.array([[-0.999799523015637, 5.703917086330493e-07, 0.020022831410793, -9.173590950956462e-04],
                                [5.703917086330493e-07, 1, 5.711003318322819e-09, 0.0103],
                                [-0.0200 , 5.711003318322819e-09, -0.9998, 0.0224],
                                [0, 0, 0, 1]])}
#gripper_finger2_finger_link

gripper_finger2_inner_knuckle_link = {"name":"gripper_finger2_inner_knuckle_link", "size": [0., 0., 0.], "height":0.0486, "radius":0.0159,
            "T_offset":np.array([[0.9993, -6.082136522406284e-10, -0.0364, -8.836780543482941e-04],
                                [-6.082136522406284e-10, 1, -3.344125215015056e-08, 0.0100],
                                [0.036363024172047 , 3.344125215015056e-08, 0.9993, 0.0243],
                                [0, 0, 0, 1]])}
#gripper_finger2_finger_inner_knuckle

gripper_finger2_finger_tip_link = {"name":"gripper_finger2_finger_tip_link", "size": [0., 0., 0.], "height":0.0429, "radius":0.0128,
            "T_offset":np.array([[-0.9579, -1.689170968418665e-07, -0.2870, 0.0014],
                                [-1.689170968418665e-07, 1, -2.475855602457809e-08, 0.0058],
                                [0.2870, 2.475855602457809e-08, -0.9579, 0.0054],
                                [0, 0, 0, 1]])}

joint_limits_low  = np.array([0, -2.8973, -3.0, -2.618, -3.1415, -3.1415, -3.1415])
joint_limits_high = np.array([0, 2.8973, 0.31, 2.6, 2.6, 3.1415, 3.1415])
start_joints      = np.array([0, -0.3004884065435629,-1.89270333133803,1.9277298476124773,-1.6044670780114405,-1.5697175550123657,2.8396582340451655])
ctrl_joint_num    = 7 # 6DoF + Base_joint

CONFIG = {"capsule": [base_link, shoulder_link, upper_arm_link, forearm_link, 
                      wrist_1_link, wrist_2_link, wrist_3_link, 
                      mount_link, camera_base_link, 
                      gripper_base_link, 
                      gripper_tcp_link, 
                      gripper_finger1_finger_link, gripper_finger1_inner_knuckle_link, gripper_finger1_finger_tip_link, 
                      gripper_finger2_finger_link, gripper_finger2_inner_knuckle_link, gripper_finger2_finger_tip_link], 
        "joint_limits_low": joint_limits_low, 
        "joint_limits_high": joint_limits_high, 
        "start_joints": start_joints,
        "ctrl_joint_num": ctrl_joint_num}