cd scripts/datasave/
zip -r uav1.zip uav1
sudo scp -P 22 uav1.zip yefeng@192.168.50.172:/home/yefeng/yefengGithub/uav_consensus_rl_ros/src/uav_consensus_rl_ros/master/datasave


# cd other/
# sudo scp -P 22 data_voltage_thrust.csv allen@192.168.10.54:/home/allen/yangyefeng/datasave
