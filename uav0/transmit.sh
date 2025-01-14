cd scripts/datasave/
zip -r uav0.zip uav0
sudo scp -P 22 uav0.zip yefeng@192.168.50.172:/home/yefeng/yefengGithub/uav_consensus_rl_ros/src/uav_consensus_rl_ros/master/datasave


# cd other/
# sudo scp -P 22 data_voltage_thrust.csv allen@192.168.10.54:/home/allen/yangyefeng/datasave
