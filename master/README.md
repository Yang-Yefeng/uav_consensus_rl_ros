# 多机实验说明文档

## 网络连接

这里感谢黄海龙老师和黄海龙老师课题组学生购置的高性能低延迟路由器。
路由器本来是他们自己实验用的，现在他们允许我们一起使用，十分感谢。
这个新的路由器的通信延迟平均在 7ms 左右，偶尔几十毫秒。
同时感谢我课题组姜百伦和 Patrick 我无人机组装标准化和 docker 多机 ROS 配置上所给予的帮助。

- 我的笔记本电脑

    实验时，我的笔记本电脑需要用网线连接到路由器，此时我的 IP 地址为: **192.168.50.172**

    若我的笔记本连接的是 wifi，那么我的 IP 地址为: **192.168.50.35**

- VICON

    实验室的 VCION 动作捕捉系统在这个路由器下边的 IP 地址为: **192.168.50.142**

- 无人机

    目前一共有三架无人机连接到这个路由器，它们的名字和 IP 地址分别为:

  | 设备名称          | IP地址           |
  |----------------|----------------|
  | gh034_alpha   | 192.168.20.226 |
  | gh034_bravo   | 192.168.50.149 |
  | gh034_charlie | 192.168.50.183 |
  | gh034_delta   | 192.168.50.125 |

  另: 实验中，将 0, 1, 2, 3 四个无人机分别定义为: gh034_charlie, gh034_delta, gh034_bravo, gh034_alpha

## 无人机与 QGC 的连接

实际上，如果能保证无人机不出现连接和模式切换错误的话，可以不使用 QGC 的。
但是考虑到无人机的连接经常会出现问题, 比如电池电压过低，加速度计异常，等等。
因此还是连接 QGC 比较稳妥。

然而，QGC 默认的端口是 14550，所以如果不进行人为设置的话，多台飞机会连接到同一个 QGC；因此，这里记录一下如何设置。

在文件夹 master/launch/mavros_launch/ 中，打开任意一个 launch 文件，找到

`<arg name="gcs_url" default="udp://:14550@192.168.50.172:14550" />`

这里面 "172" 就是要广播的地面站电脑的 IP 地址，“14550” 就是端口号。
所以，只需要在执行的时候把 IP 和 端口号做对应的修改就行。修改过后，还需要在 QGC 这个软件里面做对应修改。

打开 QGC，点击左上角的小图标
1. 选择 "Application Settings"，进入选择 "general"
2. 往下滑，有一个 “AutoCOnnect to the following devices”，把里面的 UDP 的对号取消
3. 左侧栏，选择 "Comm Links"，点击下边出现的 "Add"
4. Name 随便写，Type 选择 UDP (默认是 serial)，然后端口选择刚才指定的，点击 OK
 
Tips: 实际使用时，在执行过 consensus_uav*_mavros.launch 之后，需要在 QGC 进入这个界面，点击下边的连接，才能连上。
并且，记住在实验过后，要把这些设置还原才行，否则以后连不上了。

## 实验时
实验时，我的笔记本是主机，将 WiFi 关掉，连接网线，这时我的 IP 是 192.168.50.172 (前边提到过)。

1. 打开我电脑的终端，进入 .bashrc，输入以下指令

    ```
    export ROS_HOSTNAME=192.168.50.172					# HOST is myself
    export ROS_MASTER_URI=http://192.168.50.172:11311	# who is the master
    export ROS_IP=192.168.50.172					# my IP
    ```
    保存关闭，然后执行 source .bashrc

2. 打开终端，执行 roscore，这时已经打开了整个多机 ROS 的工作空间，实验全程，我的 roscore 不能关
3. 为每一架飞机单独执行程序，所以需要进入多个 docker，自己分清楚别乱了就行。这里我们用两架飞机为例来说明实验流程。

    实验前准备: 为了方便，我们飞机的名称和编号对应已经写在上边了，0，1，2，3 分别对应 charlie，delta，bravo，alpha
    如果只有两架飞机，那么我们就默认是 charlie 和 delta；同理，如果是三架，我们就默认是 charlie，delta，bravo

    3.1 笔记本电脑用网线连接路由器 (IP: 192.168.50.172)
    
    3.2 打开两架飞机，进入他们的 ssh (IP 在上边已经写了)，然后进入他们的 docker，通过 VS Code 远程进入他们各自的程序
    
    `注意: 不要进错地方！！一定要仔仔细细检查，不然会炸机，而且大概率一炸机就是两架飞机！！！`

    3.3 在主机上 (我的笔记本) 执行两个程序:
    
    ```
    # open terminal one, enter the ros workspace, and source
    roslaunch master vrpn_consensus.launch
    # 注意: 文件中的 vcion_sever 是 192.168.50.142; 还有各个飞机对应的编号顺序不要错
    ```
   
    ```
   # open terminal two, enter the ros workspace, and source
    roslaunch master global_config.launch config:=vicon
   # 注意: 仔细检查 global_config_vicon.launch 的各个参数！！！！
    ```

    3.4 在第一架飞机的 ssh 中，运行程序

    ```
    # open terminal one, enter the ros workspace, and source
    roslaunch uav0 consensus_uav0_mavros.launch
    ```
   
    ```
    # open terminal two, enter the ros workspace, and source
    roslaunch uav0 consensus_uav0.launch config:=vicon
    ```

    3.5 在第二架飞机的 ssh 中，运行程序

    ```
    # open terminal one, enter the ros workspace, and source
    roslaunch uav1 consensus_uav1_mavros.launch
    ```
   
    ```
    # open terminal two, enter the ros workspace, and source
    roslaunch uav1 consensus_uav1.launch config:=vicon
    ```

### 注意事项
1. 实验时，最好所有飞机都分别连接一下不同电脑的 QGC，这样做是为了方便观测他们的状态。如果有绝对信心，不连也行
2. 实验前，一定要反复仔细检查参考轨迹会不会是无人机相撞，无人机编号与 IP 是否对应
3. 实验时，最好有几个同学帮忙操控遥控器
4. 祝我好运！