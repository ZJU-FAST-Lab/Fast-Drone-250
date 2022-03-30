<font size=6> **从零制作自主空中机器人** </font>

本文档是视频教程[从零制作自主空中机器人](https://www.bilibili.com/video/BV1WZ4y167me?p=1)的配套文档，四旋翼无人机具有一定的安全风险，请同学们严格遵守安全规范，对自己的安全负责。

[TOC]

## 第一章：课程介绍
  本次课程是一套面向对自主空中机器人感兴趣的学生、爱好者、相关从业人员的免费课程，包含了从硬件组装、机载电脑环境设置、代码部署、实机实验等全套详细流程，带你从0开始，组装属于自己的自主无人机，并让它可以在未知的环境中自由避障穿行。本次课程所涉及的所有代码、硬件设计全部开源，<font color="#dd0000">严禁商用与转载，版权与最终解释权由浙江大学FASTLAB实验室所有。</font>
  本次课程的重心主要落在自主空中机器人的搭建、代码部署及调试上，关于自主空中机器人的一些理论基础，例如动力学模型，路径搜索，轨迹规划，地图构建等内容，高飞老师在深蓝学院有非常详尽而深入浅出的[课程](https://www.shenlanxueyuan.com/course/385?source=1)，本次课程就不再赘述。

## 第二章：动力套焊接
  机器人本体相关配件及焊接用工具详见[purchase_list.xlsx](purchase_list.xlsx),对硬件选型有相关疑问请看[番外一：硬件选型](# 番外一：硬件选型)
## 第三章：飞控的安装与接线
* 一定要注意电调信号线顺序！！！
  <img src="images\电机方向.jpg" alt="电机方向" style="zoom: 15%;" />
* 飞控箭头与机头同向为正向，任意方向旋转90°的倍数也可以，后续可以在飞控设置内调整，推荐和视频内相同朝向摆放。
* <font color="#dd0000">强烈推荐使用硅胶杜邦线，常规杜邦线线材过硬，容易出现接触不良。</font>
* 5V稳压模块注意贴黑胶带绝缘，周围注意贴一圈厚的海绵胶带来防止飞机降落时损坏5V模块，也可以考虑把5V模块用扎带扎在机臂旁边

## 第四章：飞控设置与试飞

* 请烧录本git项目下的`/firmware/px4_fmu-v5_default.px4`固件，这个固件是官方1.11.0版本固件编译而来，如有需要可以自行编译。实测1.13版本固件存在BUG，不建议使用，更老的固件版本未经测试。

* 在飞控的sd卡的根目录下创建`/etc/extras.txt`，写入

  ```
  mavlink stream -d /dev/ttyACM0 -s ATTITUDE_QUATERNION -r 200
  mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 200
  ```
  
  以提高imu发布频率
  
* 修改机架类型为 `Generic 250 Racer`，代指250mm轴距机型。如果是其他尺寸的机架，请根据实际轴距选择机架类型

* 修改`dshot_config`为dshot600

* 修改`CBRK_SUPPLY_CHK`为894281

* 修改`CBRK_USB_CHK`为197848

* 修改`CBRK_IO_SAFETY`为22027

* 修改`SER_TEL1_BAUD`为921600

* 修改`SYS_USE_IO`为0（搜索不到则不用管）

* <font color="#dd0000">检测电机转向前确保没有安装螺旋桨！！！！</font>

* 修改电机转向：进入mavlink控制台

  ```
  dshot reverse -m 1
  dshot save -m 1
  ```

  修改`1`为需要反向的电机序号
  
* <font color="#dd0000">第一次试飞请务必找有自稳模式下飞行经验的飞手协助，只飞过大疆无人机的飞手99%无法飞好！</font>

## 第五章：机载电脑与传感器的安装

* 碳板已经预留了拆壳NUC的安装空位。如果想拆壳安装NUC，需要额外购买USB网卡，或者拆下自带的网卡天线找地方固定住，并且由于碳纤维板导电，请务必用尼龙柱把NUC支起来，相关资料请自行查阅。
* 机载电脑使用4S航模电池直接供电，正常情况下没有问题。但理论上最好接一个稳压模块，否则在无人机炸机/电池几乎耗尽时会出现机载电脑关机的情况。但由于符合NUC功率的稳压模块比较大，请同学们酌情选用。

## 第六章：Ubuntu20.04的安装

* 镜像站地址：`http://mirrors.aliyun.com/ubuntu-releases/20.04/` 下载 `ubuntu-20.04.4-desktop-amd64.iso`
* 烧录软件UltraISO官网：`https://cn.ultraiso.net/`
* 分区设置：
  * EFI系统分区（主分区）512M
  * 交换空间（逻辑分区）16000M（内存大小的两倍）
  * 挂载点`/`（主分区）剩余所有容量
  * <font color="#dd0000">笔记本上也需要安装ubuntu，推荐装20.04版本。虚拟机或双系统都可以，如果有长期学习打算推荐双系统</font>
## 第七章：机载电脑的环境配置

* ROS安装
  * `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
  * `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'`
  * `sudo apt update`
  * `sudo apt install ros-noetic-desktop-full`
  * `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`
  * <font color="#dd0000">建议没有ROS基础的同学先去B站学习古月老师的ROS入门教程</font>
* 测试ROS
  * 打开三个终端，分别输入
  * `roscore`
  * `rosrun turtlesim turtlesim_node`
  * `rosrun turtlesim turtle_teleop_key`
* realsense驱动安装
  * `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE`
  * `sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u`
  * `sudo apt-get install librealsense2-dkms`
  * `sudo apt-get install librealsense2-utils`
  * `sudo apt-get install librealsense2-dev`
  * `sudo apt-get install librealsense2-dbg`
  * 测试：`realsense-viewer`
  * <font color="#dd0000">注意测试时左上角显示的USB必须是3.x，如果是2.x，可能是USB线是2.0的，或者插在了2.0的USB口上（3.0的线和口都是蓝色的）</font>
* 安装mavros
  * `sudo apt-get install ros-noetic-mavros`
  * `cd /opt/ros/noetic/lib/mavros`
  * `sudo ./install_geographiclib_datasets.sh`
* 安装ceres与glog与ddyanmic-reconfigure
  * 解压`3rd_party.zip`压缩包
  * 进入glog文件夹打开终端
  * `./autogen.sh && ./configure && make && sudo make install`
  * `sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev`
  * 进入ceres文件夹打开终端
  * `mkdir build`
  * `cd build`
  * `cmake ..`
  * `sudo make -j4`
  * `sudo make install`
  * `sudo apt-get install ros-noetic-ddynamic-reconfigure`
* 下载ego-planner源码并编译
  * `git clone https://github.com/ZJU-FAST-Lab/Fast-Drone-250`
  * `cd Fast-Drone-250`
  * `catkin_make`
  * `source devel/setup.bash`
  * `roslaunch ego_planner sing_run_in_sim.launch`
  * 在Rviz内按下键盘G键，再单击鼠标左键以点选无人机目标点

## 第八章：常用实验与调试软件的安装与使用

* VScode：`sudo dpkg --i ***.deb`
* Terminator：`sudo apt install terminator`
* Plotjuggler：
  * `sudo apt install ros-noetic-plotjuggler`
  * `sudo apt install ros-noetic-plotjuggler-ros`
  * `rosrun plotjuggler plotjugller`
* Net-tools：
  * `sudo apt install net-tools`
  * `ifconfig`
* ssh：
  * `sudo apt install openssh-server`
  * 在笔记本上：`ping 192.168.**.**`
  * `sudo gedit /etc/hosts`
  * 加上一行：`192.168.**.** fast-drone`
  * `ping fast-drone`
  * `ssh fast-drone@fast-drone`(`ssh 用户名@别名`)

待更新………

## 第九章：Ego-Planner代码框架与参数介绍

## 第十章：VINS的参数设置与外参标定
## 第十一章：Ego-Planner的实验
## 番外一：硬件选型

