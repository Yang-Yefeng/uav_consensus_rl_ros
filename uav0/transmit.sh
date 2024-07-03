cd scripts/datasave/
zip -r uav0.zip uav0
sudo scp -P 22 uav0.zip allen@192.168.10.54:/home/allen/yangyefeng/datasave
# mv uav0.zip /home/ht/yangyefeng/uav_consensus_rl_ros/src/uav_consensus_rl_ros/uav0/scripts
