#! /usr/bin/python3
import os, rospy

import numpy as np

from control.uav_ros_consensus import UAV_ROS_Consensus
from control.PDT_FNTSMC import pdt_fntsmc_param, pdt_fntsmc_consensus
from control.observer import predefined_time_do as pdt_do
from control.collector import data_collector
from control.utils import *

cur_ws = os.path.dirname(os.path.abspath(__file__)) + '/../../'
ID = 1

if __name__ == "__main__":
    rospy.init_node("uav1_control_consensus_pdt")
    
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
    
    pos_ctrl_param = pdt_fntsmc_param()
    pos_ctrl_param.load_param_from_yaml('~uav' + str(ID) + '_pdt_fntsmc_parameters')
    
    uav_ros = UAV_ROS_Consensus(uav_existance=uav_existance, use_ros_param=True, name='~uav1_parameters')
    uav_ros.connect()
    uav_ros.offboard_arm()
    
    print('Approaching...')
    uav_ros.global_flag = 1
    
    '''define controllers and observers'''
    observer = pdt_do()
    observer.load_param_from_yaml('~uav1_pdt_do')
    controller = pdt_fntsmc_consensus(pos_ctrl_param)
    data_record = data_collector(N=TOTAL_SEQ)
    ctrl_param_record = None
    '''define controllers and observers'''
    
    assert dt == controller.dt == observer.dt
    
    '''define trajectory'''
    if test_group == 0 or test_group == 1 or test_group == 2 or test_group == 3:
        _s = str(test_group)
    else:
        _s = 'else'
    _traj = rospy.get_param('/global_config/trajectory_' + _s)
    
    if test_group == 0 or test_group == 2:
        ra = np.array(_traj['ra']).astype(float)
        rp = np.array(_traj['rp']).astype(float)
        rba = np.array(_traj['rba']).astype(float)
        rbp = np.array(_traj['rbp']).astype(float)
        
        oa = np.array(_traj['oa']).astype(float)[ID]
        op = np.array(_traj['op']).astype(float)[ID]
        oba = np.array(_traj['oba']).astype(float)[ID]
        obp = np.array(_traj['obp']).astype(float)[ID]
        
        REF, DOT_REF, DOT2_REF = ref_uav_sequence_with_dead(dt, time_max, t_miemie, ra, rp, rba, rbp)
        NU, DOT_NU, DOT2_NU = offset_uav_sequence_with_dead(dt, time_max, t_miemie, oa, op, oba, obp)
    elif test_group == 1:
        center = np.array(_traj['center']).astype(float)
        offset = np.array(_traj['offset']).astype(float)[ID]
        REF, DOT_REF, DOT2_REF = ref_uav_set_point_sequence_with_dead(dt, time_max, t_miemie, center)
        NU, DOT_NU, DOT2_NU = offset_set_point_sequence_with_dead(dt, time_max, t_miemie, offset)
    elif test_group == 3:
        ra = np.array(_traj['ra']).astype(float)
        rp = np.array(_traj['rp']).astype(float)
        rba = np.array(_traj['rba']).astype(float)
        rbp = np.array(_traj['rbp']).astype(float)
        
        oa = np.array(_traj['oa']).astype(float)[ID]
        op = np.array(_traj['op']).astype(float)[ID]
        oba = np.array(_traj['oba']).astype(float)[ID]
        obp = np.array(_traj['obp']).astype(float)[ID]
        REF, DOT_REF, DOT2_REF = ref_uav_sequence_Bernoulli_with_dead(dt, time_max, t_miemie, ra, rp, rba, rbp)
        NU, DOT_NU, DOT2_NU = offset_uav_sequence_with_dead(dt, time_max, t_miemie, oa, op, oba, obp)
    else:
        _n = int((time_max + t_miemie) / dt)
        REF = np.tile([1.0, 0., 1.0, 0.], (_n, 1))
        DOT_REF = DOT2_REF = np.zeros((_n, 4))
        NU = DOT_NU = DOT2_NU = np.zeros((_n, 3))
    
    t0 = rospy.Time.now().to_sec()
    
    # uav_ros.pos0 = REF[0][0:3] + NU[0]
    
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        
        '''1. generate reference command and uncertainty'''
        _index = min(uav_ros.n, TOTAL_SEQ - 1)
        ref, dot_ref, dot2_ref = REF[_index], DOT_REF[_index], DOT2_REF[_index]
        nu, dot_nu, dot2_nu = NU[_index], DOT_NU[_index], DOT2_NU[_index]
        observe = np.zeros(3)
        
        if uav_ros.global_flag == 1:  # approaching
            okk = uav_ros.approaching()
            if okk:
                uav_ros.uav_msg[1].are_you_ok.data = True
            else:
                uav_ros.uav_msg[1].are_you_ok.data = False
            if okk and uav_ros.check_other_uav_ok():
                uav_ros.global_flag = 2
            t0 = rospy.Time.now().to_sec()
        elif uav_ros.global_flag == 2:  # control
            uav_ros.uav_msg[1].are_you_ok.data = True
            t_now = round(t - t0, 4)
            if uav_ros.n % 100 == 0:
                print('time: ', t_now)
            
            '''2. generate outer-loop reference signal 'eta_d' and its 1st, 2nd derivatives'''
            eta_d, dot_eta_d, dot2_eta_d = ref[0: 3], dot_ref[0: 3], dot2_ref[0: 3]
            # e = uav_ros.eta() - eta_d
            # de = uav_ros.dot_eta() - dot_eta_d
            psi_d = ref[3]
            
            if use_obs:
                syst_dynamic = -uav_ros.kt / uav_ros.m * uav_ros.dot_eta() + uav_ros.A()
                obs = observer.observe(de=uav_ros.dot_eta(), dy=syst_dynamic)
            else:
                obs = np.zeros(3)
            
            uav_ros.cal_consensus_e(nu=nu, eta_d=eta_d)
            uav_ros.cal_consensus_de(dot_nu=dot_nu, dot_eta_d=dot_eta_d)
            uav_ros.cal_Lambda_eta(dot2_eat_d=dot2_eta_d, dot2_nu=dot2_nu, obs=observe)
            
            '''3. Update the parameters of FNTSMC if RL is used'''
            if CONTROLLER == 'PX4-PID':
                uav_ros.pose.pose.position.x = ref[0] + nu[0]
                uav_ros.pose.pose.position.y = ref[1] + nu[1]
                uav_ros.pose.pose.position.z = ref[2] + nu[2]
                uav_ros.local_pos_pub.publish(uav_ros.pose)
                phi_d, theta_d, uf = 0., 0., 0.
            else:
                if CONTROLLER == 'RL':
                    ctrl_param_np = np.array(uav_ros.ctrl_param.data).astype(float)
                    # if t_now > t_miemie:  # 前几秒过渡一下
                    controller.get_param_from_actor(ctrl_param_np, update_z=True)
                    # controller.print_param()
                    ctrl_param_record = np.atleast_2d(ctrl_param_np) if ctrl_param_record is None else np.vstack((ctrl_param_record, ctrl_param_np))
                
                '''3. generate phi_d, theta_d, throttle'''
                controller.control_update_outer_consensus(b=uav_ros.b,
                                                          d=uav_ros.d,
                                                          consensus_e=uav_ros.consensus_e,
                                                          consensus_de=uav_ros.consensus_de,
                                                          Lambda_eta=uav_ros.lambda_eta,
                                                          ref=eta_d,
                                                          d_ref=dot_eta_d,
                                                          dd_ref=dot2_eta_d,
                                                          nu=nu,
                                                          d_nu=dot_nu,
                                                          dd_nu=dot2_nu,
                                                          e_max=np.array([1.5, 1.5, 1.5]),
                                                          dot_e_max=np.array([10., 10., 10.]))
                
                phi_d, theta_d, dot_phi_d, dot_theta_d, uf = uav_ros.publish_ctrl_cmd(ctrl=controller.control_out_consensus,
                                                                                      psi_d=psi_d,
                                                                                      phi_d_old=uav_ros.phi_d,
                                                                                      theta_d_old=uav_ros.theta_d,
                                                                                      dt=uav_ros.dt,
                                                                                      att_limit=[np.pi / 3, np.pi / 3],
                                                                                      dot_att_limit=[np.pi / 2, np.pi / 2],
                                                                                      use_gazebo=use_gazebo)
            
            '''5. get new uav states from Gazebo'''
            uav_ros.rk44(action=[phi_d, theta_d, uf], uav_state=uav_odom_2_uav_state(uav_ros.uav_odom))
            
            '''6. data storage'''
            data_block = {'time': uav_ros.time,  # simulation time
                          'throttle': uf,
                          'thrust': uav_ros.ctrl_cmd.thrust,
                          'ref_angle': np.array([phi_d, theta_d, psi_d]),
                          'ref_pos': ref[0: 3] + nu,
                          'ref_vel': dot_ref[0: 3] + dot_nu,
                          'd_out_obs': obs,
                          'state': uav_ros.uav_state_call_back(),
                          'dot_angle': uav_ros.uav_dot_att()}
            data_record.record(data_block)
            
            if data_record.index == data_record.N:
                print('Data collection finish. Switching to offboard position...')
                save_path = cur_ws + 'uav1/scripts/datasave/uav1/'
                if not os.path.exists(save_path):
                    os.mkdir(save_path)
                data_record.package2file(path=save_path)
                uav_ros.global_flag = 3
        elif uav_ros.global_flag == 3:  # finish, back to position
            uav_ros.uav_msg[1].are_you_ok.data = True
            uav_ros.pose.pose.position.x = uav_ros.pos0[0] - uav_ros.offset[0]
            uav_ros.pose.pose.position.y = uav_ros.pos0[1] - uav_ros.offset[1]
            uav_ros.pose.pose.position.z = uav_ros.pos0[2] - uav_ros.offset[2]
            uav_ros.local_pos_pub.publish(uav_ros.pose)
        else:
            uav_ros.uav_msg[1].are_you_ok.data = True
            uav_ros.pose.pose.position.x = uav_ros.pos0[0] - uav_ros.offset[0]
            uav_ros.pose.pose.position.y = uav_ros.pos0[1] - uav_ros.offset[1]
            uav_ros.pose.pose.position.z = uav_ros.pos0[2] - uav_ros.offset[2]
            uav_ros.local_pos_pub.publish(uav_ros.pose)
            print('working mode error...')
        uav_ros.uav_msg_publish(ref, dot_ref, nu, dot_nu, dot2_nu, controller.control_out_consensus, observe)
        uav_ros.nn_input_publish()
        uav_ros.rate.sleep()
