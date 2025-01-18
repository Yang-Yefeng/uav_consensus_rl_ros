cd scripts/datasave/
mv uav0 uav3
zip -r uav3.zip uav3
sudo scp -P 22 uav3.zip yefeng@192.168.50.172:/home/yefeng/yefengGithub/uav_consensus_rl_ros/src/uav_consensus_rl_ros/master/datasave


# cd other/
# sudo scp -P 22 data_voltage_thrust.csv allen@192.168.10.54:/home/allen/yangyefeng/datasave
