#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import time


def reset_robot_to_origin(model_name="turtlebot3_burger"):
    # 等待服务可用
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        # 创建服务客户端
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # 定义重置到原点的状态
        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = 0.0
        state.pose.position.y = 0.0
        state.pose.position.z = 0.0
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0  # 正前方朝向
        state.reference_frame = "world"
        
        # 调用服务重置位置
        response = set_state(state)
        if response.success:
            rospy.loginfo("The turtlebot3_burger has been reset ot the orogin.")
        else:
            rospy.logwarn("Reset failed.")
    except rospy.ServiceException as e:
        rospy.logerr("Call service failed %s.", e)

# 在主程序中调用（示例）
if __name__ == '__main__':
    rospy.init_node('reset_robot_node')
    reset_robot_to_origin()
    time.sleep(1.0)
