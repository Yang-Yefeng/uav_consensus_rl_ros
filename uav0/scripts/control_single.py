#! /usr/bin/python3
import os, rospy

import numpy as np

from control.uav_ros import UAV_ROS
from control.FNTSMC import fntsmc_param, fntsmc
from control.observer import robust_differentiator_3rd as rd3
from control.collector import data_collector
from control.utils import *

cur_path = os.path.dirname(os.path.abspath(__file__))

DT = 0.01
pos_ctrl_param = fntsmc_param(
    k1=np.array([0.3, 0.3, 0.3]).astype(float),
    k2=np.array([0.5, 0.5, 0.5]).astype(float),
    k3=np.array([0.5, 0.5, 0.5]).astype(float),        # 补偿观测器的，小点就行
    k4=np.array([3, 3, 3]).astype(float),
    alpha1=np.array([1.01, 1.01, 1.01]).astype(float),
    alpha2=np.array([1.01, 1.01, 1.01]).astype(float),
    dim=3,
    dt=DT
)


if __name__ == "__main__":
    rospy.init_node("uav0_control_single")
    
    uav_ros = UAV_ROS(m=0.722, dt=DT, time_max=20, pos0=np.array([0.0, 0., 1.0]), offset=np.array([0., 0., 0.]), group='/uav0')  # '/uav0'
    uav_ros.connect()   # 连接
    uav_ros.offboard_arm()      # OFFBOARD 模式 + 电机解锁
    
    print('Approaching...')
    uav_ros.global_flag = 1
    
    '''define controllers and observers'''
    obs_xy = rd3(use_freq=True,
                 omega=[[0.9, 0.9, 0.9], [0.9, 0.9, 0.9]],  # [0.8, 0.78, 0.75]
                 dim=2, dt=DT)
    obs_z = rd3(use_freq=True,
                omega=[[1.0, 1.0, 1.0]],
                dim=1, dt=DT)
    controller = fntsmc(pos_ctrl_param)
    t_MIEMIE = 5
    data_record = data_collector(N=round((uav_ros.time_max + t_MIEMIE) / DT))
    ctrl_param_record = None
    '''define controllers and observers'''
    
    ra = np.array([0., 0., 0., deg2rad(0)])
    rp = np.array([10, 10, 10, 10])  # xd yd zd psid 周期
    rba = np.array([0, 0, 1.0, deg2rad(0)])  # xd yd zd psid 幅值偏移
    rbp = np.array([np.pi / 2, 0, 0, 0])  # xd yd zd psid 相位偏移

    # ra = np.array([1.3, 1.3, 0.4, deg2rad(0)])
    # rp = np.array([6, 6, 6, 10])  # xd yd zd psid 周期
    # rba = np.array([0.0, 0.0, 1.0, deg2rad(0)])  # xd yd zd psid 幅值偏移
    # rbp = np.array([np.pi / 2, 0, 0, 0])  # xd yd zd psid 相位偏移
    
    USE_GAZEBO = False  # 使用gazebo时，无人机质量和悬停油门可能会不同
    USE_OBS = True
    
    CONTROLLER = 'FNTSMC'
    # CONTROLLER = 'RL'
    # CONTROLLER = 'PX4-PID'
    # CONTROLLER = 'MPC'
    
    e = np.zeros(3).astype(float)
    de = np.zeros(3).astype(float)
    
    ref_all, dot_ref_all, dot2_ref_all = ref_uav_sequence_with_dead(DT, uav_ros.time_max, t_MIEMIE, ra, rp, rba, rbp)
    
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        if uav_ros.global_flag == 1:  # approaching
            uav_ros.approaching()
            t0 = rospy.Time.now().to_sec()
        elif uav_ros.global_flag == 2:  # control
            t_now = round(t - t0, 4)
            if uav_ros.n % 100 == 0:
                print('time: ', t_now)
            
            '''1. generate reference command and uncertainty'''
            # if t_now < t_MIEMIE:
            #     ref, dot_ref, dot2_ref = ref_all[0], np.zeros(3), np.zeros(3)
            # else:
            ref, dot_ref, dot2_ref = ref_all[uav_ros.n], dot_ref_all[uav_ros.n], dot2_ref_all[uav_ros.n]
            
            '''2. generate outer-loop reference signal 'eta_d' and its 1st, 2nd, and 3rd-order derivatives'''
            eta_d, dot_eta_d, dot2_eta_d = ref[0: 3], dot_ref[0: 3], dot2_ref[0: 3]
            e = uav_ros.eta() - eta_d
            de = uav_ros.dot_eta() - dot_eta_d
            psi_d = ref[3]
            
            if USE_OBS:
                syst_dynamic = -uav_ros.kt / uav_ros.m * uav_ros.dot_eta() + uav_ros.A()
                observe_xy = obs_xy.observe(e=uav_ros.eta()[0:2], syst_dynamic=syst_dynamic[0:2])
                observe_z = obs_z.observe(e=uav_ros.eta()[2], syst_dynamic=syst_dynamic[2])
                observe = np.concatenate((observe_xy, observe_z))
            else:
                observe = np.zeros(3)
            
            '''3. Update the parameters of FNTSMC if RL is used'''
            if CONTROLLER == 'PX4-PID':
                uav_ros.pose.pose.position.x = ref[0] - uav_ros.offset[0]
                uav_ros.pose.pose.position.y = ref[1] - uav_ros.offset[1]
                uav_ros.pose.pose.position.z = ref[2] - uav_ros.offset[2]
                uav_ros.local_pos_pub.publish(uav_ros.pose)
                phi_d, theta_d, uf = 0., 0., 0.
            else:
                if CONTROLLER == 'RL':
                    ctrl_param_np = np.array(uav_ros.ctrl_param.data).astype(float)
                    if t_now > t_MIEMIE:        # 前几秒过渡一下
                        controller.get_param_from_actor(ctrl_param_np, update_z=False)
                    # controller.print_param()
                    ctrl_param_record = np.atleast_2d(ctrl_param_np) if ctrl_param_record is None else np.vstack((ctrl_param_record, ctrl_param_np))
                
                '''3. generate phi_d, theta_d, throttle'''
                controller.control_update_outer(e=e,
                                                de=de,
                                                dot_eta=uav_ros.vel,
                                                kt=uav_ros.kt,
                                                m=uav_ros.m,
                                                ref=eta_d,
                                                d_ref=dot_eta_d,
                                                dd_ref=dot2_eta_d,
                                                obs=observe if t_now > t_MIEMIE else np.zeros(3),
                                                e_max=0.5,
                                                dot_e_max=1.5,
                                                k_com_pos=np.array([0.0, 0.0, 0.05]),
                                                k_com_vel=np.array([0.0, 0.0, 0.05]))
                phi_d, theta_d, uf = uav_ros.publish_ctrl_cmd(controller.control_out, psi_d, USE_GAZEBO)
            
            '''5. get new uav states from Gazebo'''
            uav_ros.rk44(action=[phi_d, theta_d, uf], uav_state=uav_odom_2_uav_state(uav_ros.uav_odom))
            
            '''6. data storage'''
            data_block = {'time': uav_ros.time,  # simulation time
                          'throttle': uf,
                          'thrust': uav_ros.ctrl_cmd.thrust,
                          'ref_angle': np.array([phi_d, theta_d, psi_d]),
                          'ref_pos': ref[0: 3],
                          'ref_vel': dot_ref[0: 3],
                          'd_out_obs': observe,
                          'state': uav_ros.uav_state_call_back(),
                          'dot_angle': uav_ros.uav_dot_att()}
            data_record.record(data_block)
            
            if data_record.index == data_record.N:
                print('Data collection finish. Switching to offboard position...')
                save_path = cur_path + '/datasave/uav0/'
                if not os.path.exists(save_path):
                    os.mkdir(save_path)
                data_record.package2file(path=save_path)
                if CONTROLLER == 'RL':
                    pd.DataFrame(ctrl_param_record,
                                 columns=['k1x', 'k1y', 'k1z', 'k2x', 'k2y', 'k2z', 'k4x', 'k4y', 'k4z']). \
                        to_csv(save_path + 'ctrl_param.csv', sep=',', index=False)
                uav_ros.global_flag = 3
        elif uav_ros.global_flag == 3:  # finish, back to position
            uav_ros.pose.pose.position.x = 0
            uav_ros.pose.pose.position.y = 0
            uav_ros.pose.pose.position.z = 0.5
            uav_ros.local_pos_pub.publish(uav_ros.pose)
        else:
            uav_ros.pose.pose.position.x = 0
            uav_ros.pose.pose.position.y = 0
            uav_ros.pose.pose.position.z = 0.5
            uav_ros.local_pos_pub.publish(uav_ros.pose)
            print('working mode error...')
        uav_ros.nn_input.data = np.concatenate((e, de)).tolist()
        uav_ros.nn_input_state_pub.publish(uav_ros.nn_input)
        uav_ros.rate.sleep()
