cd scripts/datasave/
zip -r uav2.zip uav2
sudo scp -P 22 uav2.zip yefeng@192.168.50.172:/home/yefeng/yefengGithub/uav_consensus_rl_ros/src/uav_consensus_rl_ros/master/datasave


# cd other/
# sudo scp -P 22 data_voltage_thrust.csv allen@192.168.10.54:/home/allen/yangyefeng/datasave
