<font size=6> **Build an autonomous aerial robot from scratch** </font>

This is a companion document to the video [Build an autonomous aerial robot from scratch](https://www.bilibili.com/video/BV1WZ4y167me?p=1). 
Operating aerial robots is risky! Please strictly abide by the safety regulations! ! !

[TOC]

## Chapter 1: Course Introduction
  This course is a set of free courses for students, enthusiasts, and related practitioners who are interested in autonomous aerial robots. It includes a complete set of detailed procedures from hardware assembly, airborne computer environment setting, code deployment, and real machine experiments. Assemble your own autonomous drone from, and make it explore autonomously and keep away from obstacles in an unknown environment. All code and hardware design involved in this course are all open source,<font color="#dd0000">Commercial use and reprinting are strictly prohibited. The copyright and final interpretation rights are reserved by FAST-LAB, Zhejiang University.</font>
  The focus of this course is mainly on the assembly, code deployment and debugging of autonomous aerial robots. Some theoretical foundations of autonomous aerial robots, such as dynamic model, path search, trajectory planning, mapping, etc., are taught by Fei Gao in Shenlan College [course] (https://www.shenlanxueyuan.com/course/385?source=1), which will not be repeated in this course.

## Chapter 2: Drone Assembly
  For details about the accessories and welding tools of the aerial body, please refer to [purchase_list.xlsx](purchase_list.xlsx). If you have any questions about hardware, please refer to Extra 1: Hardware Selection.

## Chapter 3: Installation and wiring of flight controller
* Be sure to pay attention to the order of the ESC signal lines! ! !
  <img src="images\电机方向.jpg" alt="电机方向" style="zoom: 15%;" />
* If the arrow of the flight controller is in the same direction as the aircraft, the flight controller is forward. If rotating a multiple of 90° in any direction, you can also adjust the parameters in the flight control settings later. It is recommended to place the controller in the same orientation as in the video.
* <font color="#dd0000">It is strongly recommended to use silicone DuPont wire. Conventional DuPont wire is too hard and is prone to poor contact.</font>
* Note that the surface of the 5V voltage regulator module should be covered with black tape for insulation. Stick a thick sponge tape around it to prevent damage to the 5V module when landing. You can also consider sticking it to the arm or else places. 

## Chapter 4: Setup and flight test of flight controller

* Please burn the firmware under this git project `/firmware/px4_fmu-v5_default.px4`，This firmware is compiled from the official v1.11.0 px4 firmware. You can compile it yourself if necessary. Firmware v1.13 is not suitable for this project. The older firmware version has not been tested.

* Create `/etc/extras.txt` in the root directory of the flight controller's sd card. Write in

  ```
  mavlink stream -d /dev/ttyACM0 -s ATTITUDE_QUATERNION -r 200
  mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 200
  ```
  
  to increase the frequency of IMU.
  
* Modify the Airframe to `Generic 250 Racer`，Refers to the 250mm wheelbase model. Please select the type according to the actual wheelbase.

  Modify the following parameters.

* `dshot_config`: dshot600

* `CBRK_SUPPLY_CHK`: 894281

* `CBRK_USB_CHK`: 197848

* `CBRK_IO_SAFETY`: 22027

* `SER_TEL1_BAUD`: 921600

* `SYS_USE_IO`: 0（No set if not found）

* <font color="#dd0000">Make sure the propeller is not installed before checking the rotation of the motor! ! !</font>

* Modify the rotation direction of the motor. Enter the mavlink console.

  ```
  dshot reverse -m 1
  dshot save -m 1
  ```

  `1` is the number of the motor that needs to be reversed
  
* <font color="#dd0000">For the first test flight, please be sure to seek the assistance of a pilot who has flying experience in the self-stabilizing mode. 99% of pilots who have only flown DJI cannot fly well!</font>

## Chapter 5: Assembly of Onboard Computer and Sensors

* The carbon plate has reserved the installation hole for NUC that has removed the shell . If you want to remove the shell, you need to buy an additional USB network card, or remove the network card antenna and find a place to fix it. Because the carbon fiber board is conductive, please be sure to support the NUC with nylon column.
* The NUC uses the 4S aircraft battery to directly supply power, and there is no problem under normal circumstances. But it is better to connect a voltage stabilizing module. Please choose it as appropriate.

## Chapter 6: Installation of ubuntu20 04 

* Address of the mirror station：`http://mirrors.aliyun.com/ubuntu-releases/20.04/`. Download  `ubuntu-20.04.4-desktop-amd64.iso`
* UltraISO for burning：`https://cn.ultraiso.net/`
* Disk Partitioning：
  * EFI 512M
  * swap area 16000M（twice the memory size）
  * mount point `/` all capacity remaining
  * <font color="#dd0000">Ubuntu also needs to be installed on the PC or notebook. It is recommended to install ubuntu 20.04 virtual machine or dual system. Dual system is better if there is a long-term learning plan</font>

## Chapter 7: Environmental Configuration of the Airborne Computer

* ROS installation
  * `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
  * `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'`
  * `sudo apt update`
  * `sudo apt install ros-noetic-desktop-full`
  * `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`
  * <font color="#dd0000">It is recommended that students who do not know ROS first learn the ROS introductory tutorial by Guyueju in Bilibili.</font>
* ROS test
  * Open three terminals and enter
  * `roscore`
  * `rosrun turtlesim turtlesim_node`
  * `rosrun turtlesim turtle_teleop_key`
* realsense driver installation
  * `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE`
  * `sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u`
  * `sudo apt-get install librealsense2-dkms`
  * `sudo apt-get install librealsense2-utils`
  * `sudo apt-get install librealsense2-dev`
  * `sudo apt-get install librealsense2-dbg`
  * test：`realsense-viewer`
  * <font color="#dd0000">Note that the connected USB port must be 3.x (blue).</font>
* mavros installation
  * `sudo apt-get install ros-noetic-mavros`
  * `cd /opt/ros/noetic/lib/mavros`
  * `sudo ./install_geographiclib_datasets.sh`
* installation of ceres, glog and ddyanmic-reconfigure
  * unzip`3rd_party.zip`
  * Open the terminal and enter the ./glog
  * `./autogen.sh && ./configure && make && sudo make install`
  * `sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev`
  * Open the terminal and enter the ./ceres
  * `mkdir build`
  * `cd build`
  * `cmake ..`
  * `sudo make -j4`
  * `sudo make install`
  * `sudo apt-get install ros-noetic-ddynamic-reconfigure`
* Download ego-planner code and compile
  * `git clone https://github.com/ZJU-FAST-Lab/Fast-Drone-250`
  * `cd Fast-Drone-250`
  * `catkin_make`
  * `source devel/setup.bash`
  * `roslaunch ego_planner single_run_in_sim.launch`
  * Press the G key on the keyboard in Rviz, then click the left button of mouse to select the target point of the drone

## Chapter 8: Installation and Instructions of Experiment and Debugging Software 

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
  * PC/notebook：`ping 192.168.**.**`
  * `sudo gedit /etc/hosts`
  * add at the end：`192.168.**.** fast-drone`
  * `ping fast-drone`
  * `ssh fast-drone@fast-drone`(`ssh username@alias`)

## Chapter 9: Instructions of Ego-Planner
* `src/planner/plan_manage/launch/single_run_in_exp.launch`：
  * `map_size`：When your map is large, it needs to be modified. Note that the target point should not exceed the map_ size/2
  * `fx/fy/cx/cy`：Actual internal parameters of your depth camera
  * `max_vel/max_acc`：Max speed and acceleration. It is recommended to use 0.5 for test first. The maximum speed should not exceed 2.5. The acceleration should not exceed 6
  * `flight_type`：1 represents rviz point selection mode, 2 represents waypoints tracking mode.
* `src/planner/plan_manage/launch/advanced_param_exp.xml`下的：
  * `resolution`：Represents the resolution of the grid points of the raster map, in meters. The smaller the map, the finer the map, but the more memory it takes up. The minimum should not be lower than 0.1
  * `obstacles_inflation`：Represents the expansion size of the obstacle, in meters. It is recommended to set at least 1.5 times the radius of the aircraft (including the propeller), but not more than 4 times the `resolution`. If the wheelbase of the aircraft is larger, please increase the `resolution` 
* `src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml`：
  * `mass`：Actual weight of the drone
  * `hover_percent`：The hovering throttle of the drone. It can be viewed through px4log. For details, please refer to [document](https://www.bookstack.cn/read/px4-user-guide/zh-log-flight_review.md) If your drone is exactly the same as the course, keep this at 0.3. If the power configuration, or the weight, or the wheelbase is changed, please adjust this item. Otherwise the automatic takeoff will fail to take off or the overshoot will be serious.
  * `gain/Kp,Kv`：PID's P and I. Generally, no major changes are required. If overshoot occurs, please adjust it appropriately. If the drone is slow to respond, please adjust it appropriately
  * `rc_reverse`：No changes required if using AT9S。If it is found that the flight direction of the aircraft is opposite to the direction of the joystick, it is necessary to modify this item. Change the value corresponding to the opposite channel to `true`. If the throttle is reversed, the experiment will be very dangerous. It is recommended to confirm it before taking off：
    * `roslaunch mavros px4.launch`
    * `rostopic echo /mavros/rc/in`
    * Turn on the remote control and turn the remote control throttle from the lowest to the highest
    * See which item in the echo message is changing slowly (this item is the throttle channel value) and observe whether it changes from small to large
    * If it changes from small to large, there is no need to modify the rc_reverse of the throttle, otherwise change to `true`
    * The same for other channels
  
## Chapter 10: setting of VINS 
* Check the connection is normal
  * `ls /dev/tty*`，confirm that the serial port connection of the flight controller is normal. Generally is `/dev/ttyACM0`
  * `sudo chmod 777 /dev/ttyACM0`，give serial port permissions
  * `roslaunch mavros px4.launch`
  * `rostopic hz /mavros/imu/data_raw`，confirm that the imu frequency transmitted by the flight control is around 200hz
* Check that the realsense driver is normal
  * `roslaunch realsense2_camera rs_camera.launch`
  * Enter remote desktop, `rqt_image_view`
  * Check `/camera/infra1/image_rect_raw`,`/camera/infra2/image_rect_raw`,`/camera/depth/image_rect_raw` is normal
* VINS parameter settings
  * Check `realflight_modules/VINS_Fusion/config/`
  
  * Drive realsense，`rostopic echo /camera/infra1/camera_info`，fill in `left.yaml` and `right.yaml` with fx, fy, cx, cy in the K matrix
  
  * Create a `vins_output` folder in the home directory
  
  * Modify in `fast-drone-250.yaml`, for `body_T_cam0` and `body_T_cam1`, `data` 's fourth column to the actual extrinsic parameters of the camera on your drone relative to the flight control，in meters. The order is x/y/z, the fourth item is 1, no need to change
  
* Accurate self-calibration of VINS external parameters  
  * `sh shfiles/rspx4.sh`
  * `rostopic echo /vins_estimator/imu_propagate`
  * Pick up the robot and walk <font color="#dd0000">slowly</font>in the field. The lighting in the venue should not change too much.<font color="#dd0000">Do not use light sources that flicker</font>. Put as many clutter as possible to increase the feature points that VINS uses for matching
  * Replace the content in `vins_output/extrinsic_parameter.txt` to `body_T_cam0` and `body_T_cam1` of `fast-drone-250.yaml`
  * Repeat the above operation until the odometer data deviation of VINS converges to a satisfactory value after a few laps (usually within 0.3 meters)
* Test of mapping
  * `sh shfiles/rspx4.sh`
  * `roslaunch ego_planner single_run_in_exp.launch`
  * Enter remote desktop, `roslaunch ego_planner rviz.launch`

## Chapter 11: Experiments of Ego-Planner
* Automatic takeoff
  * `sh shfiles/rspx4.sh`
  * `rostopic echo /vins_estimator/imu_propagate`
  * Pick up the robot and remove it slowly in a small range. After putting it back in place, make sure VINS in a small deviation.
  * Channel 5 of the RC is dialed to the inside. Channel 6 is dialed to the lower side. The throttle is set to center.
  * `roslaunch px4ctrl run_ctrl.launch`
  * `sh shfiles/takeoff.sh`, If the propeller of the aircraft starts to rotate, but cannot take off, the `hover_percent` is too small. If the aircraft flies over 1 meter before descending, the `hover_percent` parameter is too big.
  * You can control the position of the drone, just like controlling the DJI
  * When landing, hit the throttle to the lowest level. After the drone is on the ground, set the channel 5 to the middle, and hit the left stick to the lower left corner to lock it.
  
* Experiment
  * Automatic takeoff
  * `roslaunch ego-planner single_run_in_exp.launch`
  * `sh shfiles/record.sh`
  * Enter remote desktop, `roslaunch ego_planner rviz.launch`
  * Press the G key and the left mouse button to click the target point to make the drone fly
  
* <font color="#dd0000">What to do if you encounter an accident during the experiment! ! !</font>
  * `case 1`: There is no problem with VINS positioning, but the untimely planning/inaccurate mapping causes the drone to plan a trajectory that may crash into an obstacle. If the pilot finds that the drone may hit an obstacle during the flight, turn the channel 6 back to the upper side before the collision. The drone will exit the trajectory following mode and enter the VINS hovering mode. Then land the drone safely
  * `case 2`: VINS positioning inaccurately，It is manifested as a large tremor of the aircraft/obviously not following the normal trajectory or fast ascent or rapid descent, etc. At this time, it is useless to dial channel 6. You must turn channel 5 back to the middle position. Make the robot completely exit the program control and return to the stabilized mode of the RC. Then land
  * `case 3`: The drone has hit an obstacle and hasn't fallen to the ground yet. At this time, dial channel 6 to see if the plane can stabilize. If not, dial channel 5 to land manually
  * `case 4`: The drone hit an obstacle and exploded to the ground. Dial channel 5 to lock it immediately to reduce property damage
  * `case 5`: **last resort** If you can't figure out what kind of case, or the plane is flying towards a very dangerous area, dial channel 7 to stop the propellers directly. In this way, the aircraft will directly lose power and fall down, which will cause great damage to the fuselage of the aircraft. Generally, it is not recommended under slow speed conditions.



