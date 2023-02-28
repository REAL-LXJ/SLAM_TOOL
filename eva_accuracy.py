#coding:utf-8

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# test enviroment 6m*5m
# 路标位置 Mark1 Mark2 Mark3 origin
Mark = [420, 826, 632, 0]       # 数据集路标点

# D455 N_fast
OPPO_D455_N_slow_VIO  = [420, 801, 606, 68.2]
OPPO_D455_N_slow_VO = [420, 834, 632, 8.4]
OPPO_D455_N_fast_VIO  = [414, 922, 754, 122] # OPPO VIO算法精度
OPPO_D455_N_fast_VO = [426, 835, 631, 9.7]   # OPPO VO精度
OPPO_D455_S_slow_VIO  = [328, 806, 657, 184]
OPPO_D455_S_slow_VO = [422, 835, 636, 39.8]
OPPO_D455_S_fast_VIO  = [418, 856, 631, 123]
OPPO_D455_S_fast_VO = [437, 830, 629, 36.2]

# D435i S_fast
OPPO_D435i_S_fast = [420+13, 826+11, 632+1 , 0+24] 
OPPO_D435i_S_slow = [420+6, 826+9, 632+2, 0+21]
OPPO_D435i_N_fast = [420+0.8, 826+14, 632+9, 0+15]
OPPO_D435i_N_slow = [420+0.7, 826+11, 632+0.5, 0+25]

def cal_Euc_distance(c, c0):
    return pow((c[0]-c0[0])**2+(c[1]-c0[1])**2,0.5)

def _quat2euler(quat):
    r = R.from_quat(quat)
    euler = r.as_euler('xyz', degrees=True)
    return euler

# vins
# timestamp x y z qx qy qz qw
def _process_odometry_data1(sourceFile):
    t_list = []
    dis_list = []
    degree_list = []
    with open(sourceFile, 'r') as f:
        for line in f:
            line = line.strip('\n').split(' ')
            timestamp = int(line[0][6:10])
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
            point = (x, y)
            _dis = cal_Euc_distance(point, (0,0))
            _degree = abs(90-yaw)
            dis_list.append(_dis*100)
            degree_list.append(_degree)
            #print(timestamp)
            #print(dis*100)
    return t_list, dis_list

def draw(t1, dis1, t2, dis2, t3, dis3, t4, dis4):
    fig, ax = plt.subplots()
    ax.plot(t1, dis1, label='pl_VO_ours_t', lw = 2)
    # #ax.plot(t1, degree1, label='pl_VIO_ours_R', lw = 2)
    # ax.plot(t2, dis2, label='filter4', lw = 2)
    # ax.plot(t3, dis3, label='filter8', lw = 2)
    # ax.plot(t4, dis4, label='filter12', lw = 2)

    line1 = ax.axhline(y = Mark[0], c = "r", lw = 2, label = "Mark")
    line2 = ax.axhline(y = Mark[1], c = "r", lw = 2)
    line3 = ax.axhline(y = Mark[2], c = "r", lw = 2)
    line4 = ax.axhline(y = Mark[3], c = "r", lw = 2)

    # line5 = ax.axhline(y = OPPO_D455_S_fast_VIO[0], c = "g", ls = '--', lw = 2, label = "OPPO_VIO")
    # line6 = ax.axhline(y = OPPO_D455_S_fast_VIO[1], c = "g", ls = '--', lw = 2)
    # line7 = ax.axhline(y = OPPO_D455_S_fast_VIO[2], c = "g", ls = '--', lw = 2)
    # line8 = ax.axhline(y = OPPO_D455_S_fast_VIO[3], c = "g", ls = '--', lw = 2)
    
    # line9  = ax.axhline(y = OPPO_D435i_S_fast[0], c = "c", ls = '-.', lw = 2, label = "OPPO_VO")
    # line10 = ax.axhline(y = OPPO_D435i_S_fast[1], c = "c", ls = '-.', lw = 2)
    # line11 = ax.axhline(y = OPPO_D435i_S_fast[2], c = "c", ls = '-.', lw = 2)
    # line12 = ax.axhline(y = OPPO_D435i_S_fast[3], c = "c", ls = '-.', lw = 2)

    ax.legend(loc = "best", fontsize = 10)
    ax.set_xlabel("timestamp", fontsize = 16)
    ax.set_yticks((0, 420, 826, 632))
    ax.set_yticklabels(['origin:0', 'Mark1:420', 'Mark2:826', 'Mark3:632'], fontsize = "small")
    ax.set_ylabel("distance(cm)", fontsize = 16)
    ax.set_title("S_fast_d435i", fontsize = 16)

# 无回环
if __name__ == "__main__":
    pl_N_fast_d435i = "/home/lxj/uv-slam_ws/src/UV-SLAM/output/N_fast_d435i.txt"
    pl_N_slow_d435i = "/home/lxj/VINS_original_ws/dnpl-slam_12.6_ws/src/DNPL-SLAM_12.6/output/7.28_N_slow.txt"
    pl_S_fast_d435i = "/home/lxj/VINS_original_ws/dnpl-slam_12.6_ws/src/DNPL-SLAM_12.6/output/8.3_S_fast.txt"
    pl_S_slow_d435i = "/home/lxj/VINS_original_ws/dnpl-slam_12.6_ws/src/DNPL-SLAM_12.6/output/8.3_S_slow.txt"
    
    pl_N_slow_d455 = "/home/lxj/uv-slam_ws/src/UV-SLAM/output/N_slow_d455.txt"
    pl_N_fast_d455 = "/home/lxj/uv-slam_ws/src/UV-SLAM/output/N_fast_d455.txt"
    pl_S_fast_d455 = "/home/lxj/uv-slam_ws/src/UV-SLAM/output/S_fast_d455.txt"
    pl_S_slow_d455 = "/home/lxj/uv-slam_ws/src/UV-SLAM/output/S_slow_d455.txt"

    test = "/home/lxj/uv-slam_ws/src/UV-SLAM/output/vins_result_no_loop.txt"
    pl_S_fast_d435i_orig = "/home/lxj/uv-slam_ws/src/UV-SLAM/output/S_fast_d435i_test1.txt"

    S_fast_d435i_filter4  = "/home/lxj/uv-slam_ws/src/UV-SLAM/output/S_fast_d435i_filter_4.txt"
    S_fast_d435i_filter8  = "/home/lxj/uv-slam_ws/src/UV-SLAM/output/S_fast_d435i_filter_8.txt"
    S_fast_d435i_filter12 = "/home/lxj/uv-slam_ws/src/UV-SLAM/output/S_fast_d435i_filter_12.txt" 

    t1, dis1 = _process_odometry_data1(pl_S_fast_d435i)
    t2, dis2 = _process_odometry_data1(S_fast_d435i_filter4)
    t3, dis3 = _process_odometry_data1(S_fast_d435i_filter8)
    t4, dis4 = _process_odometry_data1(pl_S_fast_d435i_orig)
    
    draw(t1, dis1, t2, dis2, t3, dis3, t4, dis4)
    plt.show()