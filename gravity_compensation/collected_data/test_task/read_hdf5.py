import h5py
import matplotlib.pyplot as plt
import numpy as np

path = '/home/hjx/hjx_file/rebot_devarm_ws/reBotArm_develop_hjx/gravity_compensation/collected_data/test_task/episode_0.hdf5'

obj = h5py.File(path)
print(obj.keys())
print('-------------------------')
print(obj['qpos'])
print(obj['timestamp'])
