cd datasave/
zip -r uav0.zip uav0
zip -r uav1.zip uav1
zip -r uav2.zip uav2
zip -r uav3.zip uav3
sudo scp -P 22 uav0.zip uav1.zip uav2.zip uav3.zip -t yefeng@192.168.10.229:/home/yefeng/yefengGithub/uav_consensus_rl_ros/src/uav_consensus_rl_ros/master/datasave


# cd other/
# sudo scp -P 22 data_voltage_thrust.csv allen@192.168.10.54:/home/allen/yangyefeng/datasave
