#coding:utf-8
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R


def _quat2euler(quat):
    r = R.from_quat(quat)
    euler = r.as_euler('xyz', degrees=True)
    return euler

'''
description: 处理轮速里程计-IMU外参(欧拉角表示) timestamp x y z roll pitch yaw
param {*} sourceFile
return {*}
'''
def _process_data_euler(sourceFile):
    t_list = []
    roll_list = []
    pitch_list = []
    sigma3_list = []
    _roll_std = 0.06565229085949092
    with open(sourceFile, 'r') as f:
        for line in f:
            line = line.strip('\n').split(' ')
            timestamp = int(line[0]) - 1544594828559965184
            timestamp = timestamp * 1e-9
            t_list.append(timestamp)
            x = float(line[1])
            y = float(line[2])
            z = float(line[3])
            roll = float(line[4])
            sigma3 = 3*(roll - _roll_std)
            pitch = float(line[5])
            yaw = float(line[6])
            roll_list.append(roll)
            sigma3_list.append(sigma3)
            pitch_list.append(pitch)
    return t_list, roll_list, pitch_list, sigma3_list

'''
description: 处理2Dof轮速里程计-IMU外参(四元数表示） # timestamp x y z qx qy qz qw
param {*} sourceFile
return {*}
'''
def _process_data_quat_2Dof(sourceFile):
    t_list = []
    roll_list = []
    pitch_list = []
    yaw_list = []
    roll_std_2Dof = 0.000273274830788816
    pitch_std_2Dof = 0.00675129458256553
    yaw_std_2Dof = 0.02584744642818669
    sigma3_roll_list = []
    sigma3_pitch_list = []
    sigma3_yaw_list = []
    _sigma3_roll_list = []
    _sigma3_pitch_list = []
    _sigma3_yaw_list = []
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
            sigma3_roll = 3*(roll - roll_std_2Dof)
            sigma3_pitch = 3*(pitch - pitch_std_2Dof)
            sigma3_yaw = 3*(yaw - yaw_std_2Dof)
            roll_list.append(roll)
            pitch_list.append(pitch)
            yaw_list.append(yaw)
            sigma3_roll_list.append(sigma3_roll)
            sigma3_pitch_list.append(sigma3_pitch)
            sigma3_yaw_list.append(sigma3_yaw)
            _sigma3_roll_list.append(-sigma3_roll)
            _sigma3_pitch_list.append(-sigma3_pitch)
            _sigma3_yaw_list.append(-sigma3_yaw)

        return t_list, roll_list, pitch_list, yaw_list, \
            sigma3_roll_list, sigma3_pitch_list, sigma3_yaw_list, \
            _sigma3_roll_list, _sigma3_pitch_list, _sigma3_yaw_list
            

'''
description: 处理3Dof轮速里程计-IMU外参(四元数表示） # timestamp x y z qx qy qz qw
param {*} sourceFile
return {*}
'''
def _process_data_quat_3Dof(sourceFile):
    t_list = []
    roll_list = []
    pitch_list = []
    yaw_list = []
    roll_std_3Dof = 3.1818743291285414
    pitch_std_3Dof = 0.01581286773766555
    yaw_std_3Dof = 0.04360796360266706
    sigma3_roll_list = []
    sigma3_pitch_list = []
    sigma3_yaw_list = []
    _sigma3_roll_list = []
    _sigma3_pitch_list = []
    _sigma3_yaw_list = []
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
            sigma3_roll = 3*(roll - roll_std_3Dof)
            sigma3_pitch = 3*(pitch - pitch_std_3Dof)
            sigma3_yaw = 3*(yaw - yaw_std_3Dof)
            roll_list.append(roll)
            pitch_list.append(pitch)
            yaw_list.append(yaw)
            sigma3_roll_list.append(sigma3_roll)
            sigma3_pitch_list.append(sigma3_pitch)
            sigma3_yaw_list.append(sigma3_yaw)
            _sigma3_roll_list.append(-sigma3_roll)
            _sigma3_pitch_list.append(-sigma3_pitch)
            _sigma3_yaw_list.append(-sigma3_yaw)

        return t_list, roll_list, pitch_list, yaw_list, \
            sigma3_roll_list, sigma3_pitch_list, sigma3_yaw_list, \
            _sigma3_roll_list, _sigma3_pitch_list, _sigma3_yaw_list

# draw 2Dof or 3Dof
def draw_euler_1(t_list, roll_list, pitch_list, yaw_list, \
                sigma3_roll_list, sigma3_pitch_list, sigma3_yaw_list, \
                _sigma3_roll_list, _sigma3_pitch_list, _sigma3_yaw_list):

    fig1, ax1 = plt.subplots()
    ax1.plot(t_list, roll_list, label = '2Dof-IMU-Wheel-Extinsic', lw = 2)
    ax1.plot(t_list, sigma3_roll_list, label = '3sigma-roll', lw = 2, ls = '--')
    ax1.plot(t_list, _sigma3_roll_list, label = '-3sigma-roll', lw = 2, ls = '--')
    ax1.legend(loc = "best", fontsize = 10)
    ax1.grid()
    ax1.set_xlabel("t(s)", fontsize = 16)
    ax1.set_ylabel("roll(deg)", fontsize = 16)
    ax1.set_title("Kasit Urban 28", fontsize = 16)

    fig2, ax2 = plt.subplots()
    ax2.plot(t_list, pitch_list, label = '2Dof-IMU-Wheel-Extinsic', lw = 2)
    ax2.plot(t_list, sigma3_pitch_list, label = '3sigma-pitch', lw = 2, ls = '--')
    ax2.plot(t_list, _sigma3_pitch_list, label = '-3sigma-pitch', lw = 2, ls = '--')
    ax2.legend(loc = "best", fontsize = 10)
    ax2.grid()
    ax2.set_xlabel("t(s)", fontsize = 16)
    ax2.set_ylabel("pitch(deg)", fontsize = 16)
    ax2.set_title("Kasit Urban 28", fontsize = 16)

    fig3, ax3 = plt.subplots()
    ax3.plot(t_list, yaw_list, label = '2Dof-IMU-Wheel-Extinsic', lw = 2)
    ax3.plot(t_list, sigma3_yaw_list, label = '3sigma-yaw', lw = 2, ls = '--')
    ax3.plot(t_list, _sigma3_yaw_list, label = '-3sigma-yaw', lw = 2, ls = '--')
    ax3.legend(loc = "best", fontsize = 10)
    ax3.grid()
    ax3.set_xlabel("t(s)", fontsize = 16)
    ax3.set_ylabel("yaw(deg)", fontsize = 16)
    ax3.set_title("Kasit Urban 28", fontsize = 16)

# draw 2Dof and 3DOF
def draw_euler_2(t_list_2Dof, roll_list_2Dof, pitch_list_2Dof, yaw_list_2Dof,\
                sigma3_roll_list_2Dof, sigma3_pitch_list_2Dof, sigma3_yaw_list_2Dof,\
                _sigma3_roll_list_2Dof, _sigma3_pitch_list_2Dof, _sigma3_yaw_list_2Dof,\
                t_list_3Dof, roll_list_3Dof, pitch_list_3Dof, yaw_list_3Dof,\
                sigma3_roll_list_3Dof, sigma3_pitch_list_3Dof, sigma3_yaw_list_3Dof, \
                _sigma3_roll_list_3Dof, _sigma3_pitch_list_3Dof, _sigma3_yaw_list_3Dof):
    
    fig1, ax1 = plt.subplots()
    ax1.plot(t_list_2Dof, roll_list_2Dof, label = '2Dof-IMU-Wheel-Extinsic', lw = 2)
    ax1.plot(t_list_3Dof, roll_list_3Dof, label = '3Dof-IMU-Wheel-Extinsic', lw = 2)
    ax1.plot(t_list_2Dof, sigma3_roll_list_2Dof, label = '3sigma-roll-2Dof', lw = 2, ls = '--')
    ax1.plot(t_list_2Dof, _sigma3_roll_list_2Dof, label = '-3sigma-roll-2Dof', lw = 2, ls = '--')
    ax1.plot(t_list_3Dof, sigma3_roll_list_3Dof, label = '3sigma-roll-3Dof', lw = 2, ls = '--')
    ax1.plot(t_list_3Dof, _sigma3_roll_list_3Dof, label = '-3sigma-roll-3Dof', lw = 2, ls = '--')
    ax1.legend(loc = "best", fontsize = 10)
    ax1.grid()
    ax1.set_xlabel("t(s)", fontsize = 16)
    ax1.set_ylabel("roll(deg)", fontsize = 16)
    ax1.set_title("Kasit Urban 28", fontsize = 16)

    fig2, ax2 = plt.subplots()
    ax2.plot(t_list_2Dof, pitch_list_2Dof, label = '2Dof-IMU-Wheel-Extinsic', lw = 2)
    ax2.plot(t_list_3Dof, pitch_list_3Dof, label = '3Dof-IMU-Wheel-Extinsic', lw = 2)
    ax2.plot(t_list_2Dof, sigma3_pitch_list_2Dof, label = '3sigma-pitch-2Dof', lw = 2, ls = '--')
    ax2.plot(t_list_2Dof, _sigma3_pitch_list_2Dof, label = '-3sigma-pitch-2Dof', lw = 2, ls = '--')
    ax2.plot(t_list_3Dof, sigma3_pitch_list_3Dof, label = '3sigma-pitch-3Dof', lw = 2, ls = '--')
    ax2.plot(t_list_3Dof, _sigma3_pitch_list_3Dof, label = '-3sigma-pitch-3Dof', lw = 2, ls = '--')
    ax2.legend(loc = "best", fontsize = 10)
    ax2.grid()
    ax2.set_xlabel("t(s)", fontsize = 16)
    ax2.set_ylabel("pitch(deg)", fontsize = 16)
    ax2.set_title("Kasit Urban 28", fontsize = 16)

    fig3, ax3 = plt.subplots()
    ax3.plot(t_list_2Dof, yaw_list_2Dof, label = '2Dof-IMU-Wheel-Extinsic', lw = 2)
    ax3.plot(t_list_3Dof, yaw_list_3Dof, label = '3Dof-IMU-Wheel-Extinsic', lw = 2)
    ax3.plot(t_list_2Dof, sigma3_yaw_list_2Dof, label = '3sigma-yaw-2Dof', lw = 2, ls = '--')
    ax3.plot(t_list_2Dof, _sigma3_yaw_list_2Dof, label = '-3sigma-yaw-2Dof', lw = 2, ls = '--')
    ax3.plot(t_list_3Dof, sigma3_yaw_list_3Dof, label = '3sigma-yaw-3Dof', lw = 2, ls = '--')
    ax3.plot(t_list_3Dof, _sigma3_yaw_list_3Dof, label = '-3sigma-yaw-3Dof', lw = 2, ls = '--')
    ax3.legend(loc = "best", fontsize = 10)
    ax3.grid()
    ax3.set_xlabel("t(s)", fontsize = 16)
    ax3.set_ylabel("yaw(deg)", fontsize = 16)
    ax3.set_title("Kasit Urban 28", fontsize = 16)

if __name__ == "__main__":
    # get wheel-IMU n-Dof exterior parameter
    quat_2Dof_file = "/home/lxj/桌面/VIWO/2Dof-IMU-Wheel_Extrinsic.csv"
    quat_3Dof_file = "/home/lxj/桌面/VIWO/3Dof-IMU-Wheel_Extrinsic.csv"
    
    # process wheel-IMU n-Dof exterior parameter data
    t_list_2Dof, roll_list_2Dof, pitch_list_2Dof, yaw_list_2Dof, \
        sigma3_roll_list_2Dof, sigma3_pitch_list_2Dof, sigma3_yaw_list_2Dof, \
        _sigma3_roll_list_2Dof, _sigma3_pitch_list_2Dof, _sigma3_yaw_list_2Dof = _process_data_quat_2Dof(quat_2Dof_file)
    
    t_list_3Dof, roll_list_3Dof, pitch_list_3Dof, yaw_list_3Dof,\
        sigma3_roll_list_3Dof, sigma3_pitch_list_3Dof, sigma3_yaw_list_3Dof, \
        _sigma3_roll_list_3Dof, _sigma3_pitch_list_3Dof, _sigma3_yaw_list_3Dof = _process_data_quat_3Dof(quat_3Dof_file)

    '''
    # list to array
    roll_array_2Dof = np.array(roll_list_2Dof)
    pitch_array_2Dof = np.array(pitch_list_2Dof)
    yaw_array_2Dof = np.array(yaw_list_2Dof)

    roll_array_3Dof = np.array(roll_list_3Dof)
    pitch_array_3Dof = np.array(pitch_list_3Dof)
    yaw_array_3Dof = np.array(yaw_list_3Dof)

    # solve roll，pitch，yaw std
    roll_std_2Dof = np.std(roll_array_2Dof)             # 2Dof-roll的标准差
    pitch_std_2Dof = np.std(pitch_array_2Dof)           # 2Dof-pitch的标准差
    yaw_std_2Dof = np.std(yaw_array_2Dof)               # 2Dof-yaw的标准差

    roll_std_3Dof = np.std(roll_array_3Dof)             # 3Dof-roll的标准差
    pitch_std_3Dof = np.std(pitch_array_3Dof)           # 3Dof-pitch的标准差
    yaw_std_3Dof = np.std(yaw_array_3Dof)               # 3Dof-yaw的标准差

    # print roll,pitch,yaw std
    print("roll_std_2Dof = ", roll_std_2Dof)
    print("pitch_std_2Dof = ", pitch_std_2Dof)
    print("yaw_std_2Dof = ", yaw_std_2Dof)

    print("roll_std_3Dof = ", roll_std_3Dof)
    print("pitch_std_3Dof = ", pitch_std_3Dof)
    print("yaw_std_3Dof = ", yaw_std_3Dof)
    '''


    draw_euler_1(t_list_2Dof, roll_list_2Dof, pitch_list_2Dof, yaw_list_2Dof, \
                    sigma3_roll_list_2Dof, sigma3_pitch_list_2Dof, sigma3_yaw_list_2Dof,\
                    _sigma3_roll_list_2Dof, _sigma3_pitch_list_2Dof, _sigma3_yaw_list_2Dof)
                    
    draw_euler_1(t_list_3Dof, roll_list_3Dof, pitch_list_3Dof, yaw_list_3Dof, \
                    sigma3_roll_list_3Dof, sigma3_pitch_list_3Dof, sigma3_yaw_list_3Dof, \
                    _sigma3_roll_list_3Dof, _sigma3_pitch_list_3Dof, _sigma3_yaw_list_3Dof)
    
    draw_euler_2(t_list_2Dof, roll_list_2Dof, pitch_list_2Dof, yaw_list_2Dof,\
                    sigma3_roll_list_2Dof, sigma3_pitch_list_2Dof, sigma3_yaw_list_2Dof,\
                    _sigma3_roll_list_2Dof, _sigma3_pitch_list_2Dof, _sigma3_yaw_list_2Dof,\
                    t_list_3Dof, roll_list_3Dof, pitch_list_3Dof, yaw_list_3Dof,\
                    sigma3_roll_list_3Dof, sigma3_pitch_list_3Dof, sigma3_yaw_list_3Dof, \
                    _sigma3_roll_list_3Dof, _sigma3_pitch_list_3Dof, _sigma3_yaw_list_3Dof)


    plt.show()
