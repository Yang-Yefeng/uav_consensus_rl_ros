import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os, sys


def plot_pos_consensus():
    n = len(controlData)
    L = controlData[0].shape[0]
    plt.figure(figsize=(12, 8))
    plt.subplots_adjust(left=0.05, right=0.97, top=0.95, bottom=0.07)
    for i in range(n):
        ref_pos = ref_cmdData[i][0: L - 2, 1: 4]
        uav_pos = uav_stateData[i][0: L - 2, 1: 4]
        time = controlData[i][0: L - 2, 0]
        plt.subplot(n, 3, i * 3 + 1)
        plt.plot(time, ref_pos[:, 0], 'red')
        plt.plot(time, uav_pos[:, 0], 'blue')
        plt.grid(True)
        y_min = np.min(uav_pos[:, 0])
        y_max = np.max(uav_pos[:, 0])
        plt.ylim((round(y_min - 1), round(y_max + 1)))
        # plt.yticks(np.arange(-2, 2, 0.5))
        plt.xlabel('time(s)')
        plt.title('X')
    
        plt.subplot(n, 3, i * 3 + 2)
        plt.plot(time, ref_pos[:, 1], 'red')
        plt.plot(time, uav_pos[:, 1], 'blue')
        plt.grid(True)
        y_min = np.min(uav_pos[:, 1])
        y_max = np.max(uav_pos[:, 1])
        plt.ylim((round(y_min - 1), round(y_max + 1)))
        # plt.yticks(np.arange(-2, 2, 0.5))
        plt.xlabel('time(s)')
        plt.title('Y')
    
        plt.subplot(n, 3, i * 3 + 3)
        plt.plot(time, ref_pos[:, 2], 'red')
        plt.plot(time, uav_pos[:, 2] + 0.00, 'blue')
        plt.grid(True)
        y_min = np.min(uav_pos[:, 2])
        y_max = np.max(uav_pos[:, 2])
        plt.ylim((round(y_min - 1), round(y_max + 1)))
        # plt.yticks(np.arange(0, 2, 0.5))
        plt.xlabel('time(s)')
        plt.title('Z')


def plot_att_consensus():
    n = len(controlData)
    L = controlData[0].shape[0]
    plt.figure(figsize=(12, 8))
    plt.subplots_adjust(left=0.05, right=0.97, top=0.95, bottom=0.07)
    for i in range(n):
        ref_angle = ref_cmdData[i][0: L - 2, 7: 10] * 180 / np.pi
        uav_angle = uav_stateData[i][0: L - 2, 7: 10] * 180 / np.pi
        time = controlData[i][0: L - 2, 0]
        
        plt.subplot(n, 3, i * 3 + 1)
        plt.plot(time, ref_angle[:, 0], 'red')
        plt.plot(time, uav_angle[:, 0], 'blue')
        plt.grid(True)
        plt.ylim((-90, 90))
        plt.yticks(np.arange(-90, 90, 30))
        plt.xlabel('time(s)')
        if i == 0:
            plt.title('Roll  $\phi$')
            
        plt.subplot(n, 3, i * 3 + 2)
        plt.plot(time, ref_angle[:, 1], 'red')
        plt.plot(time, uav_angle[:, 1], 'blue')
        plt.grid(True)
        plt.ylim((-90, 90))
        plt.yticks(np.arange(-90, 90, 30))
        plt.xlabel('time(s)')
        if i == 0:
            plt.title('Pitch  $\theta$')
        
        plt.subplot(n, 3, i * 3 + 3)
        plt.plot(time, ref_angle[:, 2], 'red')
        plt.plot(time, uav_angle[:, 2], 'blue')
        plt.grid(True)
        plt.ylim((-90, 90))
        plt.yticks(np.arange(-90, 90, 30))
        plt.xlabel('time(s)')
        if i == 0:
            plt.title('Yaw  $\psi$')


def plot_thrust_consensus():
    n = len(controlData)
    L = controlData[0].shape[0]
    plt.figure(figsize=(12, 3))
    plt.subplots_adjust(left=0.05, right=0.97, top=0.92, bottom=0.1)
    for i in range(n):
        time = controlData[i][0: L - 2, 0]
        thrust = controlData[i][0: L - 2, 2]
        plt.subplot(1, n, i + 1)
        plt.plot(time, thrust, 'red')  # 归一化油门
        plt.ylim((0.1, 0.9))
        plt.yticks(np.arange(0.1, 0.9, 0.1))
        plt.grid(True)
        plt.title('thrust')


def plot_throttle_consensus():
    n = len(controlData)
    L = controlData[0].shape[0]
    plt.figure(figsize=(12, 3))
    plt.subplots_adjust(left=0.05, right=0.97, top=0.92, bottom=0.1)
    for i in range(n):
        time = controlData[i][0: L - 2, 0]
        throttle = controlData[i][0: L - 2, 1]
        plt.subplot(1, n, i + 1)
        plt.plot(time, throttle, 'red')  # 归一化油门
        # plt.ylim((0.1, 0.9))
        # plt.yticks(np.arange(0.1, 0.9, 0.1))
        plt.grid(True)
        plt.title('throttle')


def plot_obs_consensus():
    n = len(controlData)
    L = controlData[0].shape[0]
    plt.figure(figsize=(12, 8))
    plt.subplots_adjust(left=0.05, right=0.97, top=0.95, bottom=0.07)
    for i in range(n):
        plt.subplot(n, 3, 3 * i + 1)
        time = controlData[i][0: L - 2, 0]
        delta_obs = observeData[i][0: L - 2, 1:4]
        plt.plot(time, delta_obs[:, 0], 'red')
        plt.grid(True)
        plt.xlabel('time(s)')
        if i == 0:
            plt.title('observe dx')
        # plt.ylim((-4, 4))
    
        plt.subplot(n, 3, 3 * i + 2)
        plt.plot(time, delta_obs[:, 1], 'red')
        plt.grid(True)
        plt.xlabel('time(s)')
        # plt.ylim((-4, 4))
        if i == 0:
            plt.title('observe dy')
    
        plt.subplot(n, 3, 3 * i + 3)
        plt.plot(time, delta_obs[:, 2], 'red')
        plt.grid(True)
        plt.xlabel('time(s)')
        # plt.ylim((-4, 4))
        if i == 0:
            plt.title('observe dz')


if __name__ == '__main__':
    """
    Data formation:
        control.csv:    t throttle
        observe.csv:    t dx_obs dy_obs dz_obs
        ref_cmd.csv:    t rx ry rz r_phi(roll) r_theta(pitch) r_psi(yaw)
        uav_state.csv:  t x y z vx vy vz phi theta psi p q r
    """
    # 检测文件夹数量
    path = os.path.dirname(os.path.abspath(__file__)) + '/'
    uav_index = []
    for i in range(4):
        if os.path.exists('uav' + str(i)):
            uav_index.append('uav' + str(i))
    
    # 批量读入数据
    controlData = []
    observeData = []
    ref_cmdData = []
    uav_stateData = []
    for i in range(len(uav_index)):
        controlData.append(pd.read_csv(path + uav_index[i] + '/control.csv', header=0).to_numpy())
        observeData.append(pd.read_csv(path + uav_index[i] + '/observe.csv', header=0).to_numpy())
        ref_cmdData.append(pd.read_csv(path + uav_index[i] + '/ref_cmd.csv', header=0).to_numpy())
        uav_stateData.append(pd.read_csv(path + uav_index[i] + '/uav_state.csv', header=0).to_numpy())
    
    plot_pos_consensus()
    plot_att_consensus()
    # plot_thrust_consensus()
    # plot_throttle_consensus()
    plot_obs_consensus()

    plt.show()
