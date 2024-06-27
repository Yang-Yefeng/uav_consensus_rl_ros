import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os, sys


def plot_pos():
    plt.figure()
    plt.subplot(1, 3, 1)
    plt.plot(time, ref_pos[:, 0], 'red')
    plt.plot(time, uav_pos[:, 0], 'blue')
    plt.grid(True)
    # plt.ylim((-2, 2))
    # plt.yticks(np.arange(-2, 2, 0.5))
    plt.xlabel('time(s)')
    plt.title('X')

    plt.subplot(1, 3, 2)
    plt.plot(time, ref_pos[:, 1], 'red')
    plt.plot(time, uav_pos[:, 1], 'blue')
    plt.grid(True)
    # plt.ylim((-2, 2))
    # plt.yticks(np.arange(-2, 2, 0.5))
    plt.xlabel('time(s)')
    plt.title('Y')

    plt.subplot(1, 3, 3)
    plt.plot(time, ref_pos[:, 2], 'red')
    plt.plot(time, uav_pos[:, 2], 'blue')
    plt.grid(True)
    # plt.ylim((0, 2))
    # plt.yticks(np.arange(0, 2, 0.5))
    plt.xlabel('time(s)')
    plt.title('Z')


def plot_phi():
    plt.figure()
    plt.plot(time, ref_angle[:, 0], 'red')
    plt.plot(time, uav_angle[:, 0], 'blue')
    plt.grid(True)
    plt.ylim((-90, 90))
    plt.yticks(np.arange(-90, 90, 10))
    plt.xlabel('time(s)')
    plt.title('roll  $\phi$')


def plot_theta():
    plt.figure()
    plt.plot(time, ref_angle[:, 1], 'red')
    plt.plot(time, uav_angle[:, 1], 'blue')
    plt.grid(True)
    plt.ylim((-90, 90))
    plt.yticks(np.arange(-90, 90, 10))
    plt.xlabel('time(s)')
    plt.title('pitch  $\Theta$')


def plot_psi():
    plt.figure()
    plt.plot(time, ref_angle[:, 2], 'red')
    plt.plot(time, uav_angle[:, 2], 'blue')
    plt.grid(True)
    plt.ylim((-90, 90))
    plt.yticks(np.arange(-90, 90, 15))
    plt.xlabel('time(s)')
    plt.title('yaw  $\psi$')


def plot_thrust():
    plt.figure()
    plt.plot(time, thrust, 'red')  # 归一化油门
    plt.ylim((0.1, 0.9))
    plt.yticks(np.arange(0.1, 0.9, 0.1))
    plt.grid(True)
    plt.title('thrust')


def plot_throttle():
    plt.figure()
    plt.plot(time, throttle, 'red')  # 归一化油门
    # plt.ylim((0.1, 0.9))
    # plt.yticks(np.arange(0.1, 0.9, 0.1))
    plt.grid(True)
    plt.title('throttle')


def plot_obs():
    plt.figure()
    plt.subplot(1, 3, 1)
    plt.plot(time, delta_obs[:, 0], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('observe dx')
    # plt.ylim((-4, 4))

    plt.subplot(1, 3, 2)
    plt.plot(time, delta_obs[:, 1], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    # plt.ylim((-4, 4))
    plt.title('observe dy')

    plt.subplot(1, 3, 3)
    plt.plot(time, delta_obs[:, 2], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    # plt.ylim((-4, 4))
    plt.title('observe dz')


if __name__ == '__main__':
    """
    Data formation:
        control.csv:    t throttle
        observe.csv:    t dx_obs dy_obs dz_obs
        ref_cmd.csv:    t rx ry rz r_phi(roll) r_theta(pitch) r_psi(yaw)
        uav_state.csv:  t x y z vx vy vz phi theta psi p q r
    """
    path = os.path.dirname(os.path.abspath(__file__)) + '/'
    controlData = pd.read_csv(path + 'uav0/control.csv', header=0).to_numpy()
    observeData = pd.read_csv(path + 'uav0/observe.csv', header=0).to_numpy()
    ref_cmdData = pd.read_csv(path + 'uav0/ref_cmd.csv', header=0).to_numpy()
    uav_stateData = pd.read_csv(path +'uav0/uav_state.csv', header=0).to_numpy()

    L = controlData.shape[0]
    time = controlData[0: L - 2, 0]

    throttle = controlData[0: L - 2, 1]
    thrust = controlData[0: L - 2, 2]

    delta_obs = observeData[0: L - 2, 1:4]

    ref_pos = ref_cmdData[0: L - 2, 1: 4]
    ref_angle = ref_cmdData[0: L - 2, 7: 10] * 180 / np.pi

    uav_pos = uav_stateData[0: L - 2, 1: 4]
    uav_angle = uav_stateData[0: L - 2, 7: 10] * 180 / np.pi
    plot_pos()
    plot_phi()
    plot_theta()
    plot_psi()
    # # plot_throttle()
    # plot_thrust()
    plot_obs()

    plt.show()
