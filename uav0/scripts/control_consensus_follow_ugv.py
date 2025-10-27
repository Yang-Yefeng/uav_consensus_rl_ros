#! /usr/bin/python3
import os, rospy

from control.uav_ros_consensus import UAV_ROS_Consensus
from control.FNTSMC import fntsmc_param, fntsmc_consensus
from control.RFNTSMC import rfntsmc_param, rfntsmc_consensus
from control.FTPD import ftpd
from control.PDT_FNTSMC import pdt_fntsmc_param, pdt_fntsmc_consensus
from control.observer import robust_differentiator_3rd as rd3
from control.observer import predefined_time_do as pdt_do
from control.collector import data_collector
from control.utils import *

cur_ws = os.path.dirname(os.path.abspath(__file__)) + '/../../'
ID = 0

if __name__ == "__main__":
    rospy.init_node("uav0_control_consensus")

    '''load some global configuration parameters'''
    t_miemie = rospy.get_param('/global_config/t_miemie')  # 轨迹跟踪前的初始化等待时间
    test_group = int(rospy.get_param('/global_config/test_group'))  # 使用的测试轨迹编号
    dt = rospy.get_param('/global_config/dt')  # 采样时间
    time_max = rospy.get_param('/global_config/time_max')  # 最大仿真时间
    TOTAL_SEQ = round((time_max + t_miemie) / dt)  # 参考序列长度
    use_gazebo = rospy.get_param('/global_config/use_gazebo')
    CONTROLLER = rospy.get_param('/global_config/controller')
    use_obs = rospy.get_param('/global_config/use_obs')
    uav_existance = rospy.get_param('/global_config/uav_existance')
    '''load some global configuration parameters'''

    if CONTROLLER == 'RFNTSMC':
        pos_ctrl_param = rfntsmc_param()
        pos_ctrl_param.load_param_from_yaml('~uav' + str(ID) + '_rfntsmc_parameters')
    else:
        pos_ctrl_param = fntsmc_param()
        pos_ctrl_param.load_param_from_yaml('~uav' + str(ID) + '_fntsmc_parameters')
    
    uav_ros = UAV_ROS_Consensus(uav_existance=uav_existance, use_ros_param=True, name='~uav0_parameters')
    uav_ros.connect()
    uav_ros.offboard_arm()
    
    print('Approaching...')
    uav_ros.global_flag = 1
    
    '''define controllers and observers'''
    obs_xy = rd3()
    obs_xy.load_param_from_yaml('~uav0_obs_xy')
    obs_z = rd3()
    obs_z.load_param_from_yaml('~uav0_obs_z')
    if CONTROLLER == 'RFNTSMC':
        controller = rfntsmc_consensus(pos_ctrl_param)
    elif CONTROLLER == 'FT-PD':
        controller = ftpd(kp_pos=np.array([2., 2., 2.5]),
                          ki_pos=np.array([0.005, 0.005, 0.4]),
                          kd_pos=np.array([3., 3., 3]),
                          p_v=np.array([0.75, 0.75, 0.8]))
    else:
        controller = fntsmc_consensus(pos_ctrl_param)
    data_record = data_collector(N=TOTAL_SEQ)
    ctrl_param_record = None
    '''define controllers and observers'''

    if test_group == 0 or test_group == 1:
            _s = str(test_group)
    else:
        _s = 'else'
    _traj = rospy.get_param('/global_config/trajectory_' + _s)

    oa = np.array(_traj['oa']).astype(float)[ID]
    op = np.array(_traj['op']).astype(float)[ID]
    oba = np.array(_traj['oba']).astype(float)[ID]
    obp = np.array(_traj['obp']).astype(float)[ID]

    t0 = rospy.Time.now().to_sec()

    