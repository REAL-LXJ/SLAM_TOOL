#coding:utf-8
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

def _quat2euler(quat):
    r = R.from_quat(quat)
    euler = r.as_euler('xyz', degrees=True)
    return euler

def _process_data_quat_nDof(sourceFile):
    t_list = []
    roll_list = []
    pitch_list = []
    yaw_list = []
    datadic = {}
    with open(sourceFile, 'r') as f:
        for line in f:
            line = line.strip('\n').split(' ')
            timestamp = int(line[0])
            timestamp = timestamp * 1e-12
            t_list.append(timestamp)
            x = float(line[1])
            y = float(line[2])
            z = float(line[3])
            qx = float(line[4])
            qy = float(line[5])
            qz = float(line[6])
            qw = float(line[7])
            quat = [qx, qy, qz, qw]
            euler = _quat2euler(quat)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            roll_list.append(roll)
            pitch_list.append(pitch)
            yaw_list.append(yaw)
    datadic = {'roll': roll_list, 'pitch':pitch_list, 'yaw': yaw_list}
    return datadic

def draw(datadic):
    df = pd.DataFrame(datadic)
    df.plot.box(title = "box_and_line")
    plt.grid()

if __name__ == "__main__":
    # get wheel-IMU n-Dof exterior parameter
    quat_2Dof_file = "/home/lxj/桌面/VIWO/2Dof-IMU-Wheel_Extrinsic.csv"
    quat_3Dof_file = "/home/lxj/桌面/VIWO/3Dof-IMU-Wheel_Extrinsic.csv"
    datadic = _process_data_quat_nDof(quat_2Dof_file)
    draw(datadic)
    plt.show()
