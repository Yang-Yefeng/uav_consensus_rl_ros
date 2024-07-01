#! /usr/bin/python3
import os, rospy
import numpy as np
from uav0.msg import uav_msg


uav_msg_0 = uav_msg()
uav_msg_1 = uav_msg()
uav_msg_2 = uav_msg()
uav_msg_3 = uav_msg()

def uav_msg_0_cb(msg:uav_msg):
    global uav_msg_0
    uav_msg_0 = msg


def uav_msg_1_cb(msg:uav_msg):
    global uav_msg_1
    uav_msg_1 = msg
    

def uav_msg_2_cb(msg:uav_msg):
    global uav_msg_2
    uav_msg_2 = msg
    

def uav_msg_3_cb(msg:uav_msg):
    global uav_msg_3
    uav_msg_3 = msg


def ok_to_record():
    return (uav_msg_0.are_you_ok.data and
            uav_msg_1.are_you_ok.data and
            uav_msg_2.are_you_ok.data and
            uav_msg_3.are_you_ok.data)


def finish_recording():
    return (uav_msg_0.finish.data and
            uav_msg_1.finish.data and
            uav_msg_2.finish.data and
            uav_msg_3.finish.data)


if __name__ == "__main__":
    t_MIEMIE = 5
    time_max = 20
    DT = 0.005
    
    rospy.init_node("test_topic_transmit")
    rate = rospy.Rate(int(1 / DT))
    
    uav_msg_0_sub = rospy.Subscriber("/uav0/uav_msg", uav_msg, callback=uav_msg_0_cb)
    uav_msg_1_sub = rospy.Subscriber("/uav1/uav_msg", uav_msg, callback=uav_msg_1_cb)
    uav_msg_2_sub = rospy.Subscriber("/uav2/uav_msg", uav_msg, callback=uav_msg_2_cb)
    uav_msg_3_sub = rospy.Subscriber("/uav3/uav_msg", uav_msg, callback=uav_msg_3_cb)
    
    # N = round((time_max + t_MIEMIE) / DT)
    record_eta_d = None
    record_nu = None
    n = 0
    
    # t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        # t = rospy.Time.now().to_sec()
        if not ok_to_record():
            rospy.loginfo("Not ready... ")
            # t0 = rospy.Time.now().to_sec()
        else:
            '''这里写记录数据的程序'''
            ref = np.concatenate((uav_msg_0.ref, uav_msg_1.ref, uav_msg_2.ref, uav_msg_3.ref))
            nu = np.concatenate((uav_msg_0.nu, uav_msg_1.nu, uav_msg_2.nu, uav_msg_3.nu))
            
            record_eta_d = np.atleast_2d(ref) if record_eta_d is None else np.vstack((record_eta_d, ref))
            record_nu = np.atleast_2d(nu) if record_nu is None else np.vstack((record_nu, nu))
            
            n += 1
        if finish_recording():
            rospy.signal_shutdown('Finish collecting data.')
        rate.sleep()
    