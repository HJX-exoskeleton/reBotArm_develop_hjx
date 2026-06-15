import h5py
import matplotlib.pyplot as plt
import numpy as np

path = '/home/hjx/hjx_file/rebot_devarm_ws/reBotArm_develop_hjx/master_slave_control/Servo_control/data/control_test/episode_0.hdf5'

obj = h5py.File(path)
print(obj.keys())
print(obj['action'])
# print(obj['action'].keys())
# print(obj['action']['target_pos'])
print('-------------------------')
print(obj['observations'])
print(obj['observations'].keys())
print('-------------------------')
print(obj['observations']['qpos'])
print(obj['observations']['qvel'])
print('-------------------------')

# print(obj['observations']['images'].keys())
# print(obj['observations']['images']['cam_high'])
# print('-------------------------')
