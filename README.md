# 注意

Gazebo 仿真与实际试验是有很大区别的，注意检查所有的模型参数和控制器参数。

所有的参数设置都在yaml文件中，一定要仔仔细细检查！！！！

# 开发环境配置
这个仿真平台依赖:
- ubuntu 20.04 + ROS noetic + 对应版本的 Gazebo
- PX4 源代码 (具体操作请参考这里: [E2ES](https://github.com/HKPolyU-UAV/E2ES/blob/master/install.md))
- 其他的 python 包，用的时候发现没有再装就行

注意: 配置好 PX4 之后，需要将本项目的 master 文件夹下边的 "yyf_0_single.launch" 和
"yyf_consensus_4_uav.launch" 复制进 PX4 源代码的 launch 文件夹中，即 "/PX4-Autopilot/launch/"

# 实验与仿真

## 单架无人机的仿真
只有 **uav0** 这个 ROS 包可以被用来运行单架无人机的仿真，
因为相关的launch文件只有在 **uav0** 中才有。 

执行程序时，需要打开两个终端

- 第一个终端 (用来启动 Gazebo 仿真环境) 

```
cd $(YOUR_WORK_SPACE)
catkin_make
source devel/setup.bash
cd src/uav_consensus_rl_ros/uav0
./yyf_0_single_gazebo.sh
```

- 第一个终端 (用来启动 uav0 的控制节点) 
```
cd $(YOUR_WORK_SPACE)
source devel/setup.bash
roslaunch uav0 control_single.launch config:=gazebo
```
OKK!

## 单无人机实验 (有室内动作捕捉系统, 仅适用于我课题组)
需要启动三个终端 

- 第一个终端 (用来启动室内定位) 
```
roslaunch uav0 vrpn_single.launch drone_name:=无人机刚体在动捕中的名字
```

- 第二个终端 (用来启动 mavros)
```
cd $(YOUR_WORK_SPACE)
catkin_make
source devel/setup.bash
roslaunch uav0 start_vicon_single.launch
```

- 第三个终端 (用于启动控制节点)

```
cd $(YOUR_WORK_SPACE)
source devel/setup.bash
roslaunch uav0 control_single.launch config:=vicon
```

OKK!

## 多无人机仿真
以 2 架无人机为例，需要打开三个终端

- 第一个终端 (用于打开 Gazebo 仿真环境)

```
cd $(YOUR_WORK_SPACE)
catkin_make
source devel/setup.bash
cd src/uav_consensus_rl_ros/master
./yyf_conensus_gazebo.sh
```

- 第二个终端 (用于启动第 1 个无人机控制节点)
```
cd $(YOUR_WORK_SPACE)
source devel/setup.bash
roslaunch master consensus_uav0.launch config:=gazebo
```

- 第三个终端 (用于启动第 2 个无人机控制节点)
```
cd $(YOUR_WORK_SPACE)
source devel/setup.bash
roslaunch master consensus_uav1.launch config:=gazebo
```

OKK!

注意: 如果是3架无人机，那么只需要再开启一个终端，然后执行
```
cd $(YOUR_WORK_SPACE)
source devel/setup.bash
roslaunch master consensus_uav2.launch config:=gazebo
```
即可，以此类推。目前仿真环境中有四架飞机，
Gazebo 默认最多支持 10 架，熟悉这个项目的代码之后可以随意增删。
此外，可以写一个 launch 文件让所有无人机的控制节点同时启动，但是不建议这么做。
因为每个控制节点会单独输出一些信息，如果强行用一个 terminal 启动的话，万一有问题，难以定位。
此外，实际实验时，本项目所拟定使用的飞机不是那种巴掌大的CrazyFile，所以一键启动容易有危险，
最好一架一架一次启动，这样便于观察异常。

### 仿真视频
Group 0 (中心点画圆，四个无人机偏移量不变):
<div align=center>
<img src="https://github.com/Yang-Yefeng/uav_consensus_rl_ros/blob/FJ005/master/gif/four_drone_gazebo_test0.gif" width="400px">
</div>
Group 1 (心点定点，四个无人机偏移量不变):
<div align=center>
<img src="https://github.com/Yang-Yefeng/uav_consensus_rl_ros/blob/FJ005/master/gif/four_drone_gazebo_test1.gif" width="400px">
</div>
Group 2 (中定点画圆，四个无人机偏移量也是圆):
<div align=center>
<img src="https://github.com/Yang-Yefeng/uav_consensus_rl_ros/blob/FJ005/master/gif/four_drone_gazebo_test2.gif" width="400px">
</div>
Group 3 (中心点 8 字型，四个无人机偏移量不变):
<div align=center>
<img src="https://github.com/Yang-Yefeng/uav_consensus_rl_ros/blob/FJ005/master/gif/four_drone_gazebo_test3.gif" width="400px">
</div>

## multi agent experiment
奥利给干了，老铁们，实验还没做呢......