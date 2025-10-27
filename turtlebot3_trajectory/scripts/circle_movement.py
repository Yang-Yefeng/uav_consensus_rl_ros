#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import signal  # 用于捕获系统中断信号
import sys     # 用于退出程序


def signal_handler(sig, frame):
    """捕获Ctrl+C信号，发送停止指令后退出"""
    rospy.loginfo("接收到停止信号，正在停止机器人...")
    # 发送零速度指令
    stop_cmd = Twist()
    pub.publish(stop_cmd)
    rospy.sleep(0.1)  # 确保指令被发送
    sys.exit(0)


def circle_movement():
    global pub

    rospy.init_node('turtlebot3_circle', anonymous=True)
    pub = rospy.Publisher('turtlebot3/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(100)
    move_cmd = Twist()
    
    # 圆形轨迹参数（可根据需要调整）
    linear_speed = 0.5   # 线速度（m/s），正值前进
    angular_speed = 0.5  # 角速度（rad/s），正值顺时针转，负值逆时针转
    
    # 设置速度指令
    move_cmd.linear.x = linear_speed    # 只沿x轴（前进方向）运动
    move_cmd.angular.z = angular_speed  # 绕z轴旋转（转弯）
    
    # 显示提示信息
    rospy.loginfo("开始圆形轨迹运动...")
    rospy.loginfo(f"Linear velocity: {linear_speed} m/s, Angular velocity: {angular_speed} rad/s")
    rospy.loginfo("按Ctrl+C停止运动")

    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # 持续发布速度指令，直到节点被关闭
        while not rospy.is_shutdown():
            pub.publish(move_cmd)
            rate.sleep()
    # except rospy.ROSInterruptException:
    #     move_cmd.linear.x = 0.0
    #     move_cmd.angular.z = 0.0
    #     pub.publish(move_cmd)
    #     rospy.loginfo("Stop movement")
    #     pass
    finally:
        # 停止运动（发送零速度指令）
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.loginfo("Stop movement")

if __name__ == '__main__':
    circle_movement()
    # try:
    #     circle_movement()
    # except rospy.ROSInterruptException:
    #     pass