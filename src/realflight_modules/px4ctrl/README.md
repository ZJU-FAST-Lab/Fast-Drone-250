# PX4 controller V1.4 先不要开源 除了最后一章“PX4 Controller详细介绍”以外都先看一遍免得踩坑

### 2021-10-26 更新：1. 修改了控制器，提高了特别低油门下的稳定性； 2. 加了些测试用代码包进来，包含飞8字、圆等轨迹生成，以及x86下的Vicon驱动等等，在`control_test/`文件夹下

# 使用说明

- [PX4 controller V1.4 先不要开源 除了最后一章“PX4 Controller详细介绍”以外都先看一遍免得踩坑](#px4-controller-v14-先不要开源-除了最后一章px4-controller详细介绍以外都先看一遍免得踩坑)
    - [2021-10-26 更新：1. 修改了控制器，提高了特别低油门下的稳定性； 2. 加了些测试用代码包进来，包含飞8字、圆等轨迹生成，以及x86下的Vicon驱动等等，在`control_test/`文件夹下](#2021-10-26-更新1-修改了控制器提高了特别低油门下的稳定性-2-加了些测试用代码包进来包含飞8字圆等轨迹生成以及x86下的vicon驱动等等在control_test文件夹下)
- [使用说明](#使用说明)
  - [飞控设置](#飞控设置)
    - [1. 简要阅读官方文档](#1-简要阅读官方文档)
    - [2. 固件烧写和基础设置](#2-固件烧写和基础设置)
    - [3. 调参](#3-调参)
  - [mavros安装和配置](#mavros安装和配置)
    - [1. 接线](#1-接线)
    - [2. 安装](#2-安装)
    - [3. 配置](#3-配置)
  - [PX4 Controller快速使用教程](#px4-controller快速使用教程)
    - [控制器简要介绍](#控制器简要介绍)
    - [坐标系定义](#坐标系定义)
    - [遥控通道设置](#遥控通道设置)
    - [控制器topic和service](#控制器topic和service)
    - [低精度简易控制](#低精度简易控制)
    - [自动起降](#自动起降)
    - [无遥控飞行](#无遥控飞行)
  - [PX4 Controller详细介绍](#px4-controller详细介绍)
    - [三种模式介绍](#三种模式介绍)
    - [角度/角速度控制](#角度角速度控制)
    - [前馈控制量解算算法](#前馈控制量解算算法)
    - [空气阻力补偿:](#空气阻力补偿)
    - [串级PID反馈控制](#串级pid反馈控制)
    - [简易/精确油门推力模型](#简易精确油门推力模型)
    - [提高精度总结](#提高精度总结)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


## 飞控设置
### 1. 简要阅读官方文档
PX4飞控**不是**开箱即用的，需要比较复杂的前期调试，推荐花半天时间把官方用户文档 docs.px4.io 中的 "Introduction" 至 "Flight Log Analysis"的部分粗略读一下.  
非常建议淘宝上买个十几元的凤凰飞控模拟器，先在电脑上熟悉飞行操作再上手.
### 2. 固件烧写和基础设置
1. 用QGC烧写代码库中的 .px4 后缀的固件，该固件主要修改了几个飞控消息的发送频率，烧不进去的话联系我，固件版本很繁杂的;
2. 机架选择并重启.  
   250mm轴距的穿越机: Generic 250 Racer  
   330mm轴距的: DJI F330  
   450mm轴距的: Generic Quadcopter 或 DJI F450  
   其他轴距的看着来
3. 校准传感器、遥控器，其中5通道设置为飞行模式切换,7通道设置为Emergency Stop(推荐);
4. 修改参数  
   CBRK_IO_SAFETY = 22027  
   CBRK_USB_CHK = 197848  
   MAV_1_CONFIG = TELEM 2  
5. 如果使用支持DShot的电调（**推荐**），修改参数  
   SYS_USE_IO = 0  
   DSHOT_CONFIG = DShot** (**是购买的电调所支持的最大值)  
   测试电机转向，如果转反了，**无需重焊**，在QGC的 MAVLink Console 页面用以下指令把转向反过来即可
   ```
   dshot reverse -m 1
   dshot save -m 1
   ```
   这里的"1"是电机编号，具体接线和操作参考 https://docs.px4.io/master/en/peripherals/dshot.html  
6. 如果使用的是PWM电调，记得做油门校准，百度搜索校准方法;
7. 飞控自带的ekf2（**首选**）需要磁力计和气压计做状态估计，这两者在小飞机上受干扰大，导致状态估计和控制不稳定、精度不高，且经常需要飞控重启，但好处是可以使用定高定点控制.如希望高的控制精度和稳定性，且不想频繁重启飞控，推荐使用飞控自带的互补滤波替换ekf2，设置参数  
   SYS_MC_EST_GROUP = Q attitude estimator(no position)  
   SYS_HAS_BARO = 0  
   SYS_HAS_MAG = 0  
   随后重启即可.这一设置会导致无法启动飞控的定高定点功能，只能使用姿态控制 Stabilized 模式或者 角速度控制 Acro 模式，且两者都是油门直通的，即油门推杆直通飞控的油门输出，需要较多练习才能掌握.
### 3. 调参
1. （**必须**）在Tuning页面下，调整 Hover Throttle，使得 Stabilize 模式下油门杆量为中时，飞机没有高度方向上的明显加减速.如果飞定高模式或定点模式，这一值调整的不对会导致定高时随机往上冲，比较危险. 在穿越机动力套件上，该值通常在20%附近，在较大轴距、较大重量的飞机上，该值通常在默认值50%附近.
2. （**可选但推荐**）在Tuning页面下点击"Advance"可以打开PID调参页面，按需调整PID参数，具体参考
   https://docs.px4.io/master/en/config_mc/pid_tuning_guide_multicopter.html
   https://docs.px4.io/master/en/config_mc/racer_setup.html
3. 好的控制器参数的表现：角度模式下，控制起来非常跟手，很灵敏。飞机配重到装完所有设备时候的重量，快速来回打满方向杆，飞机不会翻。
   注意:调参需要一定的飞机手控能力，因为需要先后飞角速度(Arco)模式和角度(Stabilized)模式，但如果需要高精度控制，PID调参是不可避免的.**参数没有调好，会导致快速改变姿态的时候飞机翻掉**，此时则必须进行PID调参了. PX4文档调参中的 THR_MDL_FAC参数 可改可不改， 这一参数会在后面的油门模型辨识中被辨识出来.

## mavros安装和配置

### 1. 接线
电脑的串口接飞控的 TELEM2 口， 如果使用的是支持串口硬件流控的设备（除了TX、RX还有RTX、CTX），推荐连接启用硬件流控以避免阻塞丢包.

### 2. 安装

mavros是PX4飞控同ros通信的包，推荐使用apt方式安装，有时需要外网
```
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```
以上参考 https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation .


### 3. 配置

```
roscd mavros
cd launch
```
跳转到安装在/opt下的mavros的launch文件夹中，sudo修改其中的"px4_pluginlists.yaml"和"px4.launch"，可以参考代码库中的这两个文件的[一份拷贝](https://github.com/ZJU-FAST-Lab/PX4-controller/tree/master/mavros_launch_files).  
px4_pluginlists.yaml主要是屏蔽了很多无关插件，免得发布和订阅大量无用的topic;  
px4.launch是配置mavros同PX4和QGC的通信方式，其中"fcu_url"是同PX4飞控通信，要打开的串口名按实际计算机的设备名来设置，波特率设置921600;"gcs_url"是其与QGC通信的端口，推荐设置为"udp-b://@"，即广播，此时局域网内任何一台电脑打开QGC就能连上PX4.  

mavros连接上飞控之后会打印飞控的固件版本等信息，随后输入`rostop hz /mavros/imu/data` 会得到200Hz的imu数据，说明连接正常.

如果用了硬件流控，则设置 fcu_url = serial-hwfc:///path/to/serial/device[:baudrate]

具体使用参考 http://wiki.ros.org/mavros 和 https://github.com/mavlink/mavros/blob/master/mavros/README.md

## PX4 Controller快速使用教程
PX4 Controller的节点名叫做 "px4ctrl"， 其以Odometry和飞机的IMU为反馈, 接收期望的位置姿态等控制指令，对飞机进行高精度控制.

### 控制器简要介绍
本px4ctrl控制器和实验室以前用的n3ctrl控制器虽然代码框架相似，但内部状态机、控制算法等部分全部重写了，以获得更高的稳定性和控制精度.   <br/><br/>
控制器共有三种模式，分别为  
**手动模式**: 终端打印绿色的"MANUAL_CTRL(L1)"， 此时控制器不起作用，飞控完全受遥控器控制.  
**定点模式**: 终端打印绿色的"AUTO_HOVER(L2)"， 此时控制器会通过代码接管飞控，飞控会处于"Offboard"模式.控制器依据odometry反馈做定点控制，此时拨动方向、油门等摇杆，飞机会慢慢移动，就像在飞大疆Mavic的定点模式一样，但此时不接收你的代码发出的指令.  
**指令控制模式**: 终端打印绿色的"CMD_CTRL(L3)"， 此时控制器允许执行你的代码发给它的飞行指令. 该模式需要你处在定点模式下，拨动6通道拨杆之后，再发送控制指令才能启动，以尽可能进行充分的安全检查.  <br/><br/>
控制器有不同控制选项来获得不同的控制精度，分别为  
**角度/角速度控制**: 前者相比后者控制精度低，但相对安全，飞机通常不会翻.  
**简易/精确油门推力模型**: 前者所需参数少且容易测量，后者需要大约半天时间来测量数据和辨识参数，但前者控制精度低.   
**空气阻力系数**: 在高速大范围飞行下比较有效，小范围大机动影响不大. <br/><br/>
控制器有三种姿态解算算法，分别为  
**ZhepeiWang0/ZhepeiWang1/rotor-drag**: 三者关于yaw角定义不一样，除此之外，第一个和第三个均支持空气阻力补偿，但前者姿态的奇点更少.

### 坐标系定义
Odometry的pose定义为前x，左y，上z，飞机机头指向x轴正向，油门拉力方向为z轴正向，一定要严格对齐！  
Odometry的速度定义和[ROS官方](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)不太一样，这点需要注意.ROS官方定义速度为机体系下的，但大部分开源VIO的速度都是定义在相对世界系的,px4ctrl默认也是定义在世界系下的.如果你用的速度是定义在机体系下的，则把 input.cpp 里面的 #define VEL_IN_BODY 0 改成 1.

### 遥控通道设置
在QGC中设置并检查:  
5通道: 飞行模式切换  
6通道: 有分配拨杆但不设置任何功能  
7通道: 紧急停桨（Emergency Stop）  
8通道: 有分配拨杆但不设置任何功能，推荐分配一个能自动回弹的拨杆  
它们在 px4ctrl 中的对应功能为  
5通道: 切换px4ctrl控制（Offboard模式）或飞控的原飞行模式  
6通道: 是否允许px4ctrl接收你的代码发给px4ctrl的控制指令  
7通道: 紧急停桨（Emergency Stop）  
8通道: 在飞控未解锁的状态下一键重启（飞控内选择ekf2估计器时常用）  

### 控制器topic和service
自己看 px4ctrl_node.cpp 文件了解控制器的topic和service.  
修改 run_ctrl.launch 中的两个topic到你用的topic名字. 用 `rqt_graph` 检查topic接收情况，确保控制器连接了  
`/mavros/state`  
`/mavros/imu/data`  
`/mavros/rc/in`  
`/mavros/setpoint_raw/attitude`  
`/你的odometry`.  
并且检查 `/mavros/imu/data`和`/你的odometry` 的频率是否在200Hz以上，`/mavros/rc/in` 在10Hz左右.


### 低精度简易控制
控制器默认运行在最低控制精度下，这里尽可能做到了自适应，因此默认参数通常就可用.  

简要的控制器使用方式如下:
1. 启动mavros和你的odometry节点  
   ```
   roslaunch mavros px4.launch
   roslaunch <你的odometry节点>
   ```
2. 启动px4ctrl
   ```
   roslaunch px4ctrl run_ctrl.launch
   ```  
3. 地面未解锁状态下检查.飞控**未解锁**状态下，拨动5通道拨杆，屏幕打印绿色的"[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)"即说明控制器检查通过，已切换到定点模式.再拨6通道摇杆，打印绿色的"[px4ctrl] TRIGGER sent， allow user command."说明可以切换到指令控制模式，允许外部控制指令接入.如遇到报错，按照显示的错误修正即可.
4. 地面解锁飞机，手动起飞到合适高度，拨动5通道遥感切换到自动定点模式，随后拨动6通道遥感切换到控制指令接收模式.注意初期不熟悉odom稳定性的时候建议切模式前看一下定位有没有飘，否则可能模式一切定点飞机就窜出去了.
5. 在拨动6通道切到控制指令接收模式时，控制器会有一个`/traj_start_trigger`的topic发出来，这个topic可以用作规划器启动的trigger，同时里面还有飞机此刻的pose.
6. :laughing::laughing::laughing:**Trouble Shooting**:laughing::laughing::laughing::  
   + 虽然px4ctrl会把检测到的错误打印出来，但有时候错误是飞控发出的，px4ctrl检测不到，表现为无法起飞、切定点没反应等等，这些飞控内的错误会打印在QGC地面站上.  
   + mavros报串口的错，检查是不是没有赋串口777权限(`sudo chmod 777 /dev/<串口名>`) ; 如果打开串口成功但没反应，检查线有没有接对，或者飞控的 MAV_1_CONFIG 参数有没有给错;  
   + 如果报别的错，检查是不是漏装了 geographiclib.  
   + 使用ekf2作为飞控的状态估计时，经常会遇到降落后无法解锁，报姿态估计错误，此时拨一下遥控的8通道，飞控就会重启，错误得到解决.
   + 如果切到定点模式后飞机持续往上或往下飞，排除定位飘了，那八成是因为油门杆没有回中，因为定点模式下是允许遥控器控制的。解决1：手动大致回中油门杆即可，因为中间附近设定了死区；解决2：config里面设置max_manual_vel为0即可，即关闭定点模式下的遥控控制功能。
   + 如果切到定点模式后飞机位置上下来回浮动，排除定位飘了，那八成是油门模型估计导致的，是因为飞行时IMU震动太大导致油门模型估计的反馈量不准确。减震不佳的IMU加速度计震动在±5~10m/s^2，而减震好的IMU（大疆N3、雷迅Nora、X7等）加速度计震动通常不超过±0.3m/s^2，差距巨大。可以录一个飞行时imu的bag文件检查一下。虽然可以通过改控制器参数来缓解，但还是希望可以做好减震，这对整个飞机都好，因此[该参数"rho2"](https://github.com/ZJU-FAST-Lab/PX4-controller/blob/536489877aa28cb61305789cf1bbbdbedbb7c26a/src/px4ctrl/src/controller.h#L70)设定成不可更改了。减震去淘宝买专用的imu海绵减震，贴飞控四个角下，期望震动应小于2m/s^2。
   + （不推荐）接上一个问题，如果实在加不了减震，可以1：增大["rho2"](https://github.com/ZJU-FAST-Lab/PX4-controller/blob/536489877aa28cb61305789cf1bbbdbedbb7c26a/src/px4ctrl/src/controller.h#L70)到0.999或0.9995或0.9998，但动态性能明显变差； 或者2：自行修改[油门模型估计函数“estimateThrustModel”](https://github.com/ZJU-FAST-Lab/PX4-controller/blob/536489877aa28cb61305789cf1bbbdbedbb7c26a/src/px4ctrl/src/controller.cpp#L549)，把原先的加速度反馈改成速度反馈，牺牲了少量动态性能，但效果提升明显；或者3：不介意控制的稳态误差的话，直接把[油门模型估计](https://github.com/ZJU-FAST-Lab/PX4-controller/blob/536489877aa28cb61305789cf1bbbdbedbb7c26a/src/px4ctrl/src/PX4CtrlFSM.cpp#L290)注释掉也行。  

### 自动起降
配置好“auto_takeoff_land”中的几个参数之后， 发送topic “/px4ctrl/takeoff_land” 即可。  
该topic内只有一个枚举类型的变量，赋值为 quadrotor_msgs::TakeoffLand::TAKEOFF 为自动起飞悬停，赋值为 quadrotor_msgs::TakeoffLand::LAND 为自动降落。  
由于自动起降涉及到自动解锁等危险功能，因此代码中有比较严格的检查，请参照程序打印出的各类ERROR信息谨慎操作，这里不做赘述。  
注意自动降落着陆后还需要经过约10s进行着陆检测和螺旋桨上锁，请耐心等待。  
任意时候都可以通过切换遥控的模式通道（通道5）取得手动控制权， 在降落过程中拨通道6则会切换到悬停模式。  

**相关参数：**  
auto_takeoff_land:  
&emsp; enable: 使能自动起降功能  
&emsp; enable_auto_arm: 使能自动解锁。如不使能，则需要手动遥控器解锁后再发送起飞 topic 指令  
&emsp; takeoff_height: 自动起飞悬停高度  
&emsp; takeoff_land_speed: 起飞、降落的速度  

:laughing::laughing::laughing:**Trouble Shooting**:laughing::laughing::laughing::  
   + 如果发现自动起飞起不动，则说明悬停油门参数hover_percentage给小了，因为在自动起降过程中不会估计油门模型的参数。
   + 如果自动起飞高度严重超调，则说明控制器反馈增益太小了，建议调大 gain: kp/kv.
   + 如果起飞就炸机，建议检查 odometry。
   + 如果自动降落电机怠速了很久（超过30s）都不停转，可能是PX4飞控的参数"MPC_MANTHR_MIN"被意外设置成了0，把它改成默认值0.08即可。

### 无遥控飞行
设置参数 no_RC 为 true 使能。 无遥控飞行只能在使能了自动解锁、起降等功能后才允许开启。   
注意：无遥控模式下遥控器仍旧可以连接飞控并在拨通道5后取得控制权， failsafe为返回遥控的手动模式。如果真的没有遥控器开着，飞控的 failsafe 为直接停桨，需注意安全。

**相关参数：**  
auto_takeoff_land:  
&emsp; no_RC: 使能不用遥控器

如需提高控制精度，多个参数需要调节，以下是详细控制器介绍和参数设置:

## PX4 Controller详细介绍
先读完快速使用教程，并且能够正确让飞机悬停或者按控制指令飞行，再修改这里的参数.  

### 三种模式介绍  
**手动模式**: 终端打印绿色的"MANUAL_CTRL(L1)"， 此时控制器不起作用，飞控完全受遥控器控制.  
**定点模式**: 终端打印绿色的"AUTO_HOVER(L2)"， 此时控制器会通过代码接管飞控，飞控会处于"Offboard"模式，这是飞控接收外部指控指令所处的模式，是由px4ctrl的代码来启动的，如果启动失败，飞控会在mavros终端打印错误消息，如果连接了QGC地面站，也会有消息打出来.该模式下，控制器依据odometry反馈做定点控制.为了方便微调飞机的起飞位置，此时拨动方向、油门等摇杆，飞机会慢慢移动，就像在飞大疆Mavica的定点模式一样，但此时不接收你的代码发出的指令. 切换这一模式前，为确保安全，你不应该发送控制代码，你的控制指令应当在收到 `/traj_start_trigger`之后才能发送，以确保不会忽然开飞.  
**指令控制模式**: 终端打印绿色的"CMD_CTRL(L3)"， 此时控制器允许执行你的代码发给它的飞行指令. 该模式需要按照确定的顺序来启动，以确保安全.首先飞机要处在定点模式下，此时把6通道拨杆从控制指令拒绝状态拨到使能状态，此刻px4ctrl会发一个`/traj_start_trigger`出来，当下还尚未切换到指令控制模式，px4ctrl会等待外部指令，一旦接收到指令，模式就会切换，屏幕也会打印绿色的"[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)"  
**起飞降落模式不做介绍**  <br/><br/>

**模式状态机跳转**: 优先级 手动模式>定点模式>指令控制模式. 低优先级向高跳转已在前几段介绍了，这里介绍高向低跳转. 任何模式下将5通道拨出定点模式，飞机将立刻退回到手控模式， 指令控制模式下将6通道拨离该模式，飞机会退到定点模式. 各个消息还有超时机制，odom、imu、rc等超时会退出到手动控制模式，command超时会退出到定点模式.<br/><br/>

**相关参数**:  
ctrl_freq_max: 控制频率，通常不用修改.
max_manual_vel: 定点模式下遥控器微调飞机位置时的最大速度，同时也是yaw角的最大角速度(弧度);  
rc_reverse: 如果发现定点时遥控的控制方向和期望方向相反，就把这里面的对应的轴反一下.不过通常PX4内部都是修正好了的.  
msg_timeout: 各个消息的超时时间.

### 角度/角速度控制
角速度环响应更快，同时还有jerk作为前馈信息，因此控制精度相比角度控制明显更高.但是在意外情况下，比如状态反馈(odom、imu)或者控制链路哪里卡住了，导致角速度输出为恒定值或有持续偏差，可能会导致飞机侧翻炸机.角度控制则相对安全，但控制精度明显差于角速度控制.  
注意: 有些参数只在角速度模式下有效，如控制指令中的 jerk、yaw_dot, 控制器参数中的 KAngR、KAngP、 KAngY.

**相关参数**:  
use_bodyrate_ctrl: 使能\失能角速度控制;  

### 前馈控制量解算算法

这一部分负责由给定的位置、速度、加速度、jerk、yaw、yaw_dot控制指令期望值(即`～cmd`消息)解算期望的控制量发给飞控，，包括角度和油门（角度控制模式下）或者角速度和油门（角速度模式下）.理想环境下，飞控精确跟踪收到的控制量，即可精确的飞出期望轨迹来，实际还需要加上下一节所讲的反馈补偿.控制量解算算法主要有三种可供选择:

**ZhepeiWang0**: [汪博文章0](https://arxiv.org/pdf/2109.08403.pdf)里Section IV.D的解算算法.考虑了水平旋转对称飞机的非线性drag，并且奇点个数最少，只在完全倒立的时候有1个奇点和小推力的1个奇点.该算法支持风阻补偿，适合高速飞行和大姿态大机动飞行.

**ZhepeiWang1**: [汪博文章1](https://arxiv.org/pdf/2103.00190v2.pdf)里的解算算法.具体参考该文章Appendix G和公式(109).其Yaw角定义和《minimum-snap》文章中一致，即Z-X-Y欧拉角转序下飞机绕Z轴转角.该算法的2个奇点出现在飞机pitch接近±90°的时候，另外无推力落体也是1个奇点，适合普通飞行和特别大机动.注意《minimum-snap》文章的角速度解算是错误的.本解算方式修复了该错误.

**rotor-drag**: [Faessler文章](https://ieeexplore.ieee.org/abstract/document/8118153)中的解法.Yaw角定义为飞机机体x轴在水平面上的投影，更贴近探索应用中对yaw角的理解.但是该解法在roll和pitch接近±90°的时候一共有4个奇点，另外小推力时也有1个无解的奇点，因此大姿态飞行下可能会表现的有些奇怪，比如Yaw角出现很大控制量.但是该算法支持线性风阻补偿，适合高速下的飞行.

**如何选择**:  
小姿态（<45°）、慢速（<3m/s）飞行时，三者区别不大.  
大姿态飞行时选汪博1.  
高速飞行时选rotor-drag，可以线性补偿空气阻力;如果希望yaw对应的是机头在水平面上的朝向，选rotor-drag的.  
高速且大姿态飞行时选汪博0, 奇点最少且可以非线性补偿空气阻力,需要准确的mass.
总体来说，平时飞选1号，高速大姿态0号，2号只有在飞机机体水平两个方向尺寸差异过大才用否则不推荐

**相关参数**:
pose_solver: 三种解法的选择， 0:汪博(带drag) 1:汪博, 2:rotor-drag    
gra:重力常数  
max_angle: 最大角度限制

### 空气阻力补偿: 
__线性空气阻力补偿__，细节建议参考[Faessler文章](https://ieeexplore.ieee.org/abstract/document/8118153)，只在rotor-drag解算下有用. 空气阻力补偿在高速大范围飞行下比较有效，小范围大机动影响不大. 以下为简略介绍和简单测试方法:  <br/><br/>
rotor_drag.x/y/z: 三个轴的空气阻力系数，文章认为该系数与速度成正比， 在此可以粗略定义为 delta_acc = rotor_drag * velocity. 即它表示某个速度下空气阻力给飞机带来的反向加速度. 参数的测试方法比较简单，所有空气阻力系数给0，在开阔场地上， 用 `rostopic pub -r 100 /<你设置的控制指令的名字>` 的方式来给某个轴一个期望速度，比如5米每秒，同时位置x、y轴给 `.NaN`, 这表示临时失能x、y轴的位置反馈.当飞机速度稳定的时候记录下的稳定速度值（会小于期望值）和稳定角度，此时该角度下理论上产生的加速度值便和空气阻力产生的反向加速度抵消了，系数便得到了.  <br/><br/>
k_thrust_horz是速度对油门值的影响，文章认为水平速度会产生推力方向的正向加速度，因此在[controller.cpp](https://github.com/ZJU-FAST-Lab/PX4-controller/blob/master/src/px4ctrl/src/controller.cpp)文件中使用这个参数来减去速度对油门的影响，可以在该文件中搜索这一值.  <br/><br/>
有趣的是，z轴的空气阻力系数和k_thrust_horz参数原文章辨识出来都是0，因此我们通常也给0.但是速度过大(>10m/s)的时候，该参数给0会让期望推力偏小<br/><br/>
__非线性空气阻力补偿__，细节建议参考[汪博文章0](https://arxiv.org/pdf/2109.08403.pdf)里Section IV.D,只在pose_solver=0下有用.空气阻力补偿空气阻力补偿在高速大范围飞行下比较有效，小范围大机动影响不大. 以下为简略介绍和简单测试方法:  <br/><br/>
minimumSingularityFlatWithDrag函数里有dh,dv和cp: 水平和竖直方向的的空气阻力系数以及二次项，文章认为该系数与速度成二次关系， 在机体系上可以粗略定义为 delta_acc_horizontal = dh * (velocity + cp * velocity^2)以及delta_acc_vertical = dv * (velocity + cp * velocity^2). 即它表示某个速度下空气阻力给飞机带来的反向加速度. 参数的测试方法同线性空气补偿一致，但是需要对dh,dv和cp进行进一步拟合。一个参考值是(dv, dh, cp) = (0.35, 0.25, 0.01).注意这些参数是和飞机质量相关的，因此飞机质量mass应该给准确。此外，机体z轴的系数dv应该不小于机体水平轴的系数dh，因为高速下面风阻和迎风面正相关。

**相关参数**: 
rotor_drag: 这里面的四个参数，如上介绍 

### 串级PID反馈控制
控制器的控制框架是:　由你发送的控制指令中的高阶导的期望值（accel对应于角度控制模式，jerk对应于角速度控制模式）解算一个角度或角速度控制指令以及油门控制指令作为前馈，辅以你发送的控制量中的低阶导的期望值做串级反馈补偿.目前只用了PID
中的P. 串级PID参数的调试要从底层下层往上层调.

**相关参数**:   
gain:  
&emsp; Kp0 ~ Kp2: 位置误差比例增益;  
&emsp; Kv0 ～ Kv2: 速度误差比例增益;  
&emsp; Kvi0 ～ Kvd2: 速度积分微分增益，实际没用上，都给0即可;  
&emsp; KAngR ～ KAngY: **角速度模式下**的角度误差比例增益，角度模式没用;  

### 简易/精确油门推力模型 
这是由期望机体z轴加速度算出期望油门值（0~1）的映射关系. 两种模型都有部分参数是在线估计的，以更鲁棒. <br/><br/>

**简易推力模型**是一个线性模型，认为油门值和产生的加速度是一个线性关系， 会在线根据期望机体z轴加速度和实际机体z轴加速度估计线性模型的斜率.  
该模型下只有一个参数要给，为 hover_percentage（范围0~1），表示飞机在手飞角速度（Acro）模式下的大致悬停油门（即大致悬停时油门摇杆的杆量）.该参数用作计算简易线性模型的斜率初值，会在切换到悬停时自动估计，因此其不必特别精确（误差30%以内均可），因此可以这样试着给: 250mm穿越机初始给0.2，大的机架可以适当增加到0.5， 首先把 print_value 设置为true， 随后拨5通道切换到定点模式， 即可看到控制器会在油门估计运行时不断输出估计的悬停油门值，记录下稳定值，将该值填入参数文件的 hover_percentage 即可。注意日常使用时关闭该参数，因为高频率打印数据很占CPU和IO。这个参数在`/debugPx4ctrl`也是会录的，所以接收topic来看也行。  
关于悬停油门给的不精确的后果，如果会往上略微冲一下（冲几厘米），说明该参数给大了，调小一些，往下掉一下，则说明给小了，理想情况是切到定点的时候不会有任何上窜或下落. <br/><br/>

**精确推力模型**是非线性的，有三个参数 K1，K2，K3，表达式为 F = K1 * Voltage^K2 * (K3*u^2 + (1-K3)*u). F是推力（N），Voltage是电压（V）,u是油门信号值（0~1）. 这是我参考PX4飞控的所用的油门模型和实际测试数据整出来的，缺少理论依据，但估计的比较准确.注意该模型需要mavros提供电压输入，对应topic为`/mavros/battery`，频率应为100Hz.  

这一模型同简易模型一样，也有一个在线辨识的参数，叫debug.thr_scale_compensate，不过该参数是辨识模型的准确度，理论上应该稳定在1.0附近的某个值上，在飞行过程中，如果该值在它的稳定值（不一定是1.0但应靠近1.0）附近波动量应小于3%，否则认为精确模型参数有误，已经不精确了，需要重新校准，可以接收`/debugPx4ctrl`的消息来看这个值.  

该模型的辨识方法需要想一想的，软件包里有一个thrust_calibrate.py 文件，可以用来从topic中读取电压和油门量，方便辨识用.如果有比较准确的螺旋桨拉力座，这一辨识会很方便，但我没有，所以我用的辨识方法是: 分别给飞机绑上不同重物，每次从满电悬停到没电，用thrust_calibrate.py记录期间的油门和电压关系，且记录飞机的重量来作为这段时间的实际推力.随后用matlab的非线性拟合函数nlinfit来拟合K1～K3，实际拟合精度很高.

**相关参数**  
print_value: 显示飞行过程中 “hover_percentage”（简易推力模型下）或者 “thr_scale_compensate”（精确推力模型下）的值.
accurate_thrust_model: 使能或失能精确推力模型;  
hover_percentage: 简易推力模型的参数  
mass: 只在精确推力模型中用  
low_voltage: 低电压阈值，建议设置为电芯数*3.3V，该参数只在精确推力模型中使用;  
K1～K3: 只在精确推力模型中用，需要精确辨识  

### 提高精度总结
1. 角速度控制对控制精度影响很明显，如果对状态反馈的稳定性有足够的信心，推荐使用;
2. 精确的油门模型只在油门变化很大的大机动飞行中效果显著，考虑到其比较复杂的辨识过程，普通飞行中不推荐使用;
3. 空气阻力不要随便加，小范围、低速、大机动下空气阻力系数会降低控制精度.


