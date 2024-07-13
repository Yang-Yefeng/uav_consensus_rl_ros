import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os, sys


def plot_param():
    plt.figure()
    for _p in param.T:
        plt.plot(index, _p)
    plt.legend(['k1x', 'k1y', 'k1z', 'k2x', 'k2y', 'k2z', 'k4x', 'k4y', 'k4z'])
    plt.grid(True)


if __name__ == '__main__':
    path = os.path.dirname(os.path.abspath(__file__)) + '/'
    param = pd.read_csv(path + 'uav1/ctrl_param.csv', header=0).to_numpy()
    L = param.shape[0]
    index = np.arange(L)
    
    plot_param()
    plt.show()
