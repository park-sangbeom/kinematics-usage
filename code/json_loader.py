import matplotlib.pyplot as plt 
import json 
name = '../code/data/grp/manipulator_traj.json'
json_content = []
for line in open(name, 'r'):
    json_content.append(json.loads(line))
    print("[Number of trajectory: {}] [Number of step: {}] [Joint value: {}]".format(json.loads(line)["num_traj"], json.loads(line)["num_step"], json.loads(line)["q"]))