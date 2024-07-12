# 注意

Gazebo 仿真与实际试验是有很大区别的，注意检查所有的模型参数和控制器参数。
包括：
1. 无人机质量！！！！！！！
2. 是否使用 Gazebo，这决定了不同的推力模型！！！！！！
3. 是否使用观测器
4. 如何设置 t_MIEMIE
5. FNTSMC 中位置和速度补偿

# 我是智障

# Simulation & Experiment

## single agent simulation
Only package **uav0** can be implemented for single agent simulation.
One needs to open two terminals.

- Open terminal one

```
cd $(YOUR_WORK_SPACE)
catkin_make
source devel/setup.bash
cd src/uav_consensus_rl_ros/uav0
./yyf_0_single_gazebo.sh
```

- Open terminal two
```
cd $(YOUR_WORK_SPACE)
source devel/setup.bash
roslaunch uav0 control_single.launch config:=gazebo
```
OKK!

## single agent experiment
Only package **uav0** can be implemented for single agent experiment.
One needs to open four terminals.

- Open terminal one
```
Open VRPN
```

- Open terminal two
```
cd $(YOUR_WORK_SPACE)
catkin_make
source devel/setup.bash
roslaunch uav0 start_vicon.launch
```

- Open terminal three

```
cd $(YOUR_WORK_SPACE)
source devel/setup.bash
roslaunch uav0 control_single.launch config:=vicon
```

OKK!

## multi agent simulation
One needs to open two terminals.

- Open terminal one

```
cd $(YOUR_WORK_SPACE)
catkin_make
source devel/setup.bash
cd src/uav_consensus_rl_ros/master
./yyf_conensus_gazebo.sh
```

- Open terminal two
```
cd $(YOUR_WORK_SPACE)
source devel/setup.bash
roslaunch master consensus_uav.launch config:=gazebo
```

OKK!

## multi agent experiment
奥利给干了