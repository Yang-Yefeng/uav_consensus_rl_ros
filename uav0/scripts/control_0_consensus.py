#! /usr/bin/python3
import os, rospy

from control.uav_ros_consensus import UAV_ROS_Consensus
from control.FNTSMC import fntsmc_param, fntsmc
from control.observer import robust_differentiator_3rd as rd3
from control.collector import data_collector
from control.utils import *

DT = 0.01
pos_ctrl_param = fntsmc_param(
    k1=np.array([0.3, 0.3, 1.0]),
    k2=np.array([0.5, 0.5, 1]),
    k3=np.array([0.05, 0.05, 0.05]),  # 补偿观测器的，小点就行
    k4=np.array([6, 6, 6]),
    alpha1=np.array([1.01, 1.01, 1.01]),
    alpha2=np.array([1.01, 1.01, 1.01]),
    dim=3,
    dt=DT
)

if __name__ == "__main__":
    rospy.init_node("uav0")
    
    uav_ros = UAV_ROS_Consensus(m=0.722,
                                dt=DT,
                                time_max=20,
                                pos0=np.array([1.5, 0., 1.0]),
                                offset=np.array([1.5, 0., 0.]),
                                uav_existance=[1, 0, 0, 0])
    uav_ros.connect()  # 连接
    uav_ros.offboard_arm()  # OFFBOARD 模式 + 电机解锁
    
    print('Approaching...')
    uav_ros.global_flag = 1
    
    '''define controllers and observers'''
    obs_xy = rd3(use_freq=True,
                 omega=[[1.0, 1.1, 1.2], [1.0, 1.1, 1.2]],  # [0.8, 0.78, 0.75]
                 dim=2, dt=DT)
    obs_z = rd3(use_freq=True,
                omega=[[1.2, 1.2, 1.2]],
                dim=1, dt=DT)
    controller = fntsmc(pos_ctrl_param)
    data_record = data_collector(N=round(uav_ros.time_max / DT))
    '''define controllers and observers'''
    
    ra = np.array([1.0, 1.0, 0.3, deg2rad(0)])
    rp = np.array([6, 6, 8, 10])
    rba = np.array([0, 0, 1, deg2rad(0)])
    rbp = np.array([np.pi / 2, 0, 0, 0])
    
    oa = np.array([0., 0., 0.])
    op = np.array([5, 5, 4])
    oba = np.array([0.5, 0, 0])
    obp = np.array([0., 0., 0.])
    
    USE_GAZEBO = True  # 使用gazebo时，无人机质量和悬停油门可能会不同
    USE_OBS = True
    
    CONTROLLER = 'FNTSMC'
    # CONTROLLER = 'RL'
    # CONTROLLER = 'PX4-PID'
    # CONTROLLER = 'MPC'
    
    REF, DOT_REF, DOT2_REF = ref_uav_sequence(DT, uav_ros.time_max, ra, rp, rba, rbp)
    NU, DOT_NU, DOT2_NU = offset_uav_sequence(DT, uav_ros.time_max, oa, op, oba, obp)
    
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        
        '''1. generate reference command and uncertainty'''
        ref, dot_ref, dot2_ref = REF[uav_ros.n], DOT_REF[uav_ros.n], DOT2_REF[uav_ros.n]
        nu, dot_nu, dot2_nu = NU[uav_ros.n], DOT_NU[uav_ros.n], DOT2_NU[uav_ros.n]
        observe = np.zeros(3)
        
        if uav_ros.global_flag == 1:  # approaching
            okk = uav_ros.approaching()
            if (okk and
                uav_ros.uav_msg_1.are_you_ok.data and
                uav_ros.uav_msg_2.are_you_ok.data and
                uav_ros.uav_msg_3.are_you_ok.data):
                uav_ros.global_flag = 2
            t0 = rospy.Time.now().to_sec()
        elif uav_ros.global_flag == 2:  # control
            t_now = round(t - t0, 4)
            if uav_ros.n % 100 == 0:
                print('time: ', t_now)
            
            '''2. generate outer-loop reference signal 'eta_d' and its 1st, 2nd derivatives'''
            eta_d, dot_eta_d, dot2_eta_d = ref[0: 3], dot_ref[0: 3], dot2_ref[0: 3]
            e = uav_ros.eta() - eta_d
            de = uav_ros.dot_eta() - dot_eta_d
            psi_d = ref[3]
            
            syst_dynamic = -uav_ros.kt / uav_ros.m * uav_ros.dot_eta() + uav_ros.A()
            observe_xy = obs_xy.observe(e=uav_ros.eta()[0:2], syst_dynamic=syst_dynamic[0:2])
            observe_z = obs_z.observe(e=uav_ros.eta()[2], syst_dynamic=syst_dynamic[2])
            if USE_OBS and t_now > 2:
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
                    pos_s = np.concatenate((e, de))
                    param_pos = uav_ros.opt_pos.evaluate(uav_ros.pos_norm(pos_s))
                    hehe = np.array([1, 1, 1, 1, 1, 1, 5, 5, 5]).astype(float)
                    controller.get_param_from_actor(param_pos * hehe, update_z=False)
                
                '''3. generate phi_d, theta_d, throttle'''
                controller.control_update_outer(e_eta=e,
                                                dot_e_eta=de,
                                                dot_eta=uav_ros.vel,
                                                kt=uav_ros.kt,
                                                m=uav_ros.m,
                                                dd_ref=dot2_eta_d,
                                                obs=observe)
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
                save_path = os.getcwd() + '/src/uav_consensus_rl_ros/uav0/scripts/datasave/uav0/'
                if not os.path.exists(save_path):
                    os.mkdir(save_path)
                data_record.package2file(path=save_path)
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
        uav_ros.uav_msg_publish(ref, dot_ref, nu, dot_nu, observe)
        uav_ros.rate.sleep()
