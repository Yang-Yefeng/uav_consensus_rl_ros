#! /usr/bin/python3

import rospy, os
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
import pandas as pd

current_state = State()
battery = BatteryState()
attitude = AttitudeTarget()

cur_path = os.path.dirname(os.path.abspath(__file__))

def state_cb(msg: State):
    global current_state
    current_state = msg


def battery_cb(msg: BatteryState):
    global battery
    battery = msg


def attitude_cb(msg: AttitudeTarget):
    global attitude
    attitude = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    group = '/uav0'  # '/uav0'
    
    state_sub = rospy.Subscriber(group+"/mavros/state", State, callback = state_cb)
    battery_sub = rospy.Subscriber(group + "/mavros/battery", BatteryState, callback=battery_cb)
    attitude_sub = rospy.Subscriber(group + "/mavros/setpoint_raw/target_attitude", AttitudeTarget, callback=attitude_cb)
    
    local_pos_pub = rospy.Publisher(group+"/mavros/setpoint_position/local", PoseStamped, queue_size=1000)
    
    rospy.wait_for_service(group+"/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy(group+"/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service(group+"/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy(group+"/mavros/set_mode", SetMode)

    rate = rospy.Rate(25)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1

    for i in range(100):
        if(rospy.is_shutdown()):
            break
        local_pos_pub.publish(pose)
        rate.sleep()
    
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    
    while (current_state.mode != "OFFBOARD") and (not rospy.is_shutdown()):  # 等待
        if set_mode_client.call(offb_set_mode).mode_sent:
            print('Switching to OFFBOARD mode is available.')
            break
        local_pos_pub.publish(pose)
        rate.sleep()
    
    while (not current_state.armed) and (not rospy.is_shutdown()):
        if arming_client.call(arm_cmd).success:
            print('UAV is armed now.')
            break
        local_pos_pub.publish(pose)
        rate.sleep()
    
    
    data_voltage_thrust = None
    t0 = rospy.Time.now().to_sec()
    flag = 0

    while(not rospy.is_shutdown()):
        t = rospy.Time.now().to_sec()
        local_pos_pub.publish(pose)
        
        if flag == 0:
            if battery.voltage > 15.5:
                if data_voltage_thrust is None:
                    data_voltage_thrust = np.atleast_2d([battery.voltage, attitude.thrust])
                else:
                    data_voltage_thrust = np.vstack((data_voltage_thrust, [battery.voltage, attitude.thrust]))
            else:
                rospy.loginfo('OK, finish recording, waiting for shutting down...')
                pd.DataFrame(data_voltage_thrust, columns=['voltage', 'thrust']).to_csv(cur_path + '/data_voltage_thrust.csv', sep=',', index=False)
                flag = 1
        else:
            pass
        
        rate.sleep()
