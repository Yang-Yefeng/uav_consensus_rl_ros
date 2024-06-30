#! /usr/bin/python3
import os, rospy
from control.utils import *

from std_msgs.msg import Float32, Float32MultiArray

nn_input_state = Float32([0, 0, 0, 0, 0, 0])

def uav_state_cb(msg: Float32):
    global nn_input_state
    nn_input_state = msg


if __name__ == "__main__":
    rospy.init_node("uav0_rl_param_opt")
    
    nn_input_state_sub = rospy.Subscriber("/uav0/nn_input_rl", Float32MultiArray, callback=uav_state_cb)
    param_pub = rospy.Publisher("/uav0/ctrl_param", Float32MultiArray, queue_size=10)
    
    rate = rospy.Rate(100)
    
    opt_pos = PPOActor_Gaussian(state_dim=6, action_dim=9)
    cur_path = os.path.dirname(os.path.abspath(__file__))
    optPathPos = cur_path + '/../nets/pos_maybe_good_1/'  # 最好的
    opt_pos.load_state_dict(torch.load(optPathPos + 'actor'))
    pos_norm = get_normalizer_from_file(6, optPathPos, 'state_norm.csv')
    
    while not rospy.is_shutdown():
        # print('嗨嗨嗨', nn_input_state.data)
        nn_input_state_np = np.array(nn_input_state.data).astype(float)
        t_1 = rospy.Time.now().to_sec() * 1000.
        
        param_pos = opt_pos.evaluate(pos_norm(nn_input_state_np))
        hehe = np.array([1, 1, 1, 1, 1, 1, 5, 5, 5]).astype(float)
        
        pp = np.clip(param_pos * hehe, 0.01, 100.).tolist()
        param_pub.publish(Float32MultiArray(data=pp))
        
        t_2 = rospy.Time.now().to_sec() * 1000.
        # print("NN Evaluation time: ", t_2 - t_1, '  ms.')
        # rospy.loginfo('NN evaluation time: %.8f' % (t_2 - t_1))
        rate.sleep()
