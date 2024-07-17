#! /usr/bin/python3
import rospy


if __name__ == "__main__":
    rospy.init_node("global_config")
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        rate.sleep()
