#! /usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    group = '/uav0'  # '/uav0'
    state_sub = rospy.Subscriber(group+"/mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher(group+"/mavros/setpoint_position/local", PoseStamped, queue_size=1000)

    rospy.wait_for_service(group+"/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy(group+"/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service(group+"/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy(group+"/mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(100)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1

    # Send a few setpoints before starting
    # print('1')
    for i in range(100):
        # print('嗨嗨嗨')
        if(rospy.is_shutdown()):
            break
        local_pos_pub.publish(pose)
        rate.sleep()
    # print('2')
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                else:
                    rospy.loginfo("Fuck your mother.")

                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()
