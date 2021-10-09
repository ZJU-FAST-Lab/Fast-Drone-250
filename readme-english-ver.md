<<<<<<< HEAD
### 更新!

中文版readme请移步 [readme.md](readme.md).



# Hardware Design of the FAST-LAB Autonomous Drone

## 0.Preview

​	This project introduces the hardware design and assembly process of the FAST-LAB autonomous drone. This drone can be applied in autonomous flight in unknown environment, swarm flight, for example. The following projects are on based on the introduced drone:[Ego-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) [Ego-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner) [CMPCC](https://github.com/ZJU-FAST-Lab/CMPCC) [Fast-Tracker](https://github.com/ZJU-FAST-Lab/Fast-tracker) [Fast-Racing](https://github.com/ZJU-FAST-Lab/Fast-Racing)

## 1. Basic Hardware Formation

<img src="images\1.png" alt="1.jpg" style="zoom:50%;" />

+ The basic hardware of a drone that can be controlled by a remote controller are: frame, motor, ESC, flight controller, receiver, remote controller, li-po battery, voltmeter, etc.

+ Based on the basic hardware, an autonomous drone need two more parts: a stereo camera for visual localization, an onboard computer for running navigation algorithms.

+  The following sheet contains the model, quantity, and purchasing link(we only offer the taobao link,and developer overseas may have to look up the device in other purchasing channel. In theory, every similar model works.)

  | Device                     | Model                | Quantity | Unit Price | Total Price | 淘宝链接                                                     |
  | -------------------------- | -------------------- | -------- | ---------- | ----------- | ------------------------------------------------------------ |
  | Frame                      | QAV250               | 1        | 38         | 38          | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=520738516076&_u=l32egecqf290 |
  | ESC                        | EMAX 45A             | 4        | 89         | 356         | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=627250691828&_u=l32egecq104a |
  | Motor                      | T-MOTOR F60 KV2550   | 4        | 159        | 636         | https://item.taobao.com/item.htm?spm=a230r.1.14.6.58866126r2mH5j&id=612118488792&ns=1&abbucket=3#detail |
  | Flight Controller          | CUAV NORA            | 1        | 1669       | 1669        | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=618340579779&_u=l32egecq6321 |
  |                            | V5+                  | 1        | 1499       | 1499        | https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-22188405087.10.39df7ad6BKLAJz&id=594262853015 |
  |                            | X7                   | 1        | 1999       | 1999        | https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-22188405087.22.39df7ad6BKLAJz&id=617384615131 |
  | Onboard Computer           | DJI MANIFOLD2-C      | 1        | 8799       | 8799        | https://m.dji.com/cn/product/manifold-2                      |
  |                            | JETSON XAVIER NX     | 1        | 6800       | 6800        | https://detail.tmall.com/item.htm?spm=a230r.1.14.33.351a587bMPOWBh&id=619740546745&ns=1&abbucket=3&skuId=4573153270812 |
  |                            | NX carrier           | 1        | 889        | 889         | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.40df2e8dWJlaLW&id=613984388047&_u=s32egecqa8ff |
  | Stereo Camera              | INTEL REALSENSE D435 | 1        | 1590       | 1590        | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=638877621060&_u=l32egecq42d1 |
  | Remote Controller          | RadioLink AT9S PRO   | 1        | 580        | 580         | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=533085053894&_u=l32egecq481a |
  | Receiver                   | RadioLink  R12DSM    | 1        | 94         | 94          | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=541658831753&_u=l32egecq5116 |
  | Carbon Fiber Board         | Cumtomized           | 1        | 80         | 80          | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=628187754851&_u=l32egecq8290 |
  | USB Network Card           | EDIMAX EW7822ULC     | 1        | 188        | 188         | https://item.jd.com/10022884495770.html                      |
  | 3D Print Part              | Cumtomized           | 1        | 100        | 100         | https://wenext.cn/                                           |
  | Battery                    | Tattu 2300mAh 4S     | 4        | 125        | 500         | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=583311920871&_u=l32egecq9cf8 |
  | TYPEC cable                | 30cm                 | 2        | 27         | 54          | https://detail.tmall.com/item.htm?id=617461584216&spm=a1z09.2.0.0.2abd2e8da0sHzh&_u=l32egecq0e19 |
  | Paddle                     | GEMFAN 51477         | 10       | 14         | 140         | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=627007813072&_u=l32egecqfabf |
  | Tracking Camera (optional) | INTEL REALSENSE T265 | 1        | 1600       | 1600        | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=638877621060&_u=l32egecq42d1 |
  |                            |                      |          |            |             |                                                              |
  | Silicone Wire              |                      | some     |            |             |                                                              |
  | Voltmeter                  |                      | 1        |            |             |                                                              |
  | Dupont Thread              |                      | some     |            |             |                                                              |
  | XT60 Plug                  |                      | 1        |            |             |                                                              |

  It should be noted that, the model selection of flight controller is CUAV NORA, V5+, X7, while any other FCU that supports PX4 firmware works, too. As for the onboard computer, you can choose DJI MANIFOLD 2-C or JETSON XAVIER NX.  The STL model of the 3D print part and the carbon fiber board can be found at catalog "model".

  ## 2. Disassembly of 3D model

  + MANIFOLD 2-C version：

  <img src="images\2.png" style="zoom:50%;" />

+ XAVIER NX version：to be updated.

## 3. Environment of Onboard Computer

This drone runs at Ubuntu 16.04/18.04 and ROS Kinetic/Melodic, and correlating installation can be found at  [ROS Installation](http://wiki.ros.org/ROS/Installation)



## 4. Assembly

The main difficulty of the assembly lies at the weld of the power suit and the configuration and tuning of the FCU. Please refer to [assemble.pdf](assemble.pdf). 

=======
# Hardware Design of the FAST-LAB Autonomous Drone

## 0.Preview

​	This project introduces the hardware design and assembly process of the FAST-LAB autonomous drone. This drone can be applied in autonomous flight in unknown environment, swarm flight, for example. The following projects are on based on the introduced drone:[Ego-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) [Ego-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner) [CMPCC](https://github.com/ZJU-FAST-Lab/CMPCC) [Fast-Tracker](https://github.com/ZJU-FAST-Lab/Fast-tracker) [Fast-Racing](https://github.com/ZJU-FAST-Lab/Fast-Racing)

## 1. Basic Hardware Formation

<img src="images\1.png" alt="1.jpg" style="zoom:50%;" />

+ The basic hardware of a drone that can be controlled by a remote controller are: frame, motor, ESC, flight controller, receiver, remote controller, li-po battery, voltmeter, etc.

+ Based on the basic hardware, an autonomous drone need two more parts: a stereo camera for visual localization, an onboard computer for running navigation algorithms.

+  The following sheet contains the model, quantity, and purchasing link(we only offer the taobao link,and developer overseas may have to look up the device in other purchasing channel. In theory, every similar model works.)

  | Device                     | Model                | Quantity | Unit Price | Total Price | 淘宝链接                                                     |
  | -------------------------- | -------------------- | -------- | ---------- | ----------- | ------------------------------------------------------------ |
  | Frame                      | QAV250               | 1        | 38         | 38          | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=520738516076&_u=l32egecqf290 |
  | ESC                        | EMAX 45A             | 4        | 89         | 356         | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=627250691828&_u=l32egecq104a |
  | Motor                      | T-MOTOR F60 KV2550   | 4        | 159        | 636         | https://item.taobao.com/item.htm?spm=a230r.1.14.6.58866126r2mH5j&id=612118488792&ns=1&abbucket=3#detail |
  | Flight Controller          | CUAV NORA            | 1        | 1669       | 1669        | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=618340579779&_u=l32egecq6321 |
  |                            | V5+                  | 1        | 1499       | 1499        | https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-22188405087.10.39df7ad6BKLAJz&id=594262853015 |
  |                            | X7                   | 1        | 1999       | 1999        | https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-22188405087.22.39df7ad6BKLAJz&id=617384615131 |
  | Onboard Computer           | DJI MANIFOLD2-C      | 1        | 8799       | 8799        | https://m.dji.com/cn/product/manifold-2                      |
  |                            | JETSON XAVIER NX     | 1        | 6800       | 6800        | https://detail.tmall.com/item.htm?spm=a230r.1.14.33.351a587bMPOWBh&id=619740546745&ns=1&abbucket=3&skuId=4573153270812 |
  |                            | NX carrier           | 1        | 889        | 889         | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.40df2e8dWJlaLW&id=613984388047&_u=s32egecqa8ff |
  | Stereo Camera              | INTEL REALSENSE D435 | 1        | 1590       | 1590        | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=638877621060&_u=l32egecq42d1 |
  | Remote Controller          | RadioLink AT9S PRO   | 1        | 580        | 580         | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=533085053894&_u=l32egecq481a |
  | Receiver                   | RadioLink  R12DSM    | 1        | 94         | 94          | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=541658831753&_u=l32egecq5116 |
  | Carbon Fiber Board         | Cumtomized           | 1        | 80         | 80          | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=628187754851&_u=l32egecq8290 |
  | USB Network Card           | EDIMAX EW7822ULC     | 1        | 188        | 188         | https://item.jd.com/10022884495770.html                      |
  | 3D Print Part              | Cumtomized           | 1        | 100        | 100         | https://wenext.cn/                                           |
  | Battery                    | Tattu 2300mAh 4S     | 4        | 125        | 500         | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=583311920871&_u=l32egecq9cf8 |
  | TYPEC cable                | 30cm                 | 2        | 27         | 54          | https://detail.tmall.com/item.htm?id=617461584216&spm=a1z09.2.0.0.2abd2e8da0sHzh&_u=l32egecq0e19 |
  | Paddle                     | GEMFAN 51477         | 10       | 14         | 140         | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=627007813072&_u=l32egecqfabf |
  | Tracking Camera (optional) | INTEL REALSENSE T265 | 1        | 1600       | 1600        | https://item.taobao.com/item.htm?spm=a1z09.2.0.0.2abd2e8da0sHzh&id=638877621060&_u=l32egecq42d1 |
  |                            |                      |          |            |             |                                                              |
  | Silicone Wire              |                      | some     |            |             |                                                              |
  | Voltmeter                  |                      | 1        |            |             |                                                              |
  | Dupont Thread              |                      | some     |            |             |                                                              |
  | XT60 Plug                  |                      | 1        |            |             |                                                              |

  It should be noted that, the model selection of flight controller is CUAV NORA, V5+, X7, while any other FCU that supports PX4 firmware works, too. As for the onboard computer, you can choose DJI MANIFOLD 2-C or JETSON XAVIER NX.  The STL model of the 3D print part and the carbon fiber board can be found at catalog "model".

  ## 2. Disassembly of 3D model

  + MANIFOLD 2-C version：

  <img src="images\2.png" style="zoom:50%;" />

+ XAVIER NX version：to be updated.

## 3. Environment of Onboard Computer

This drone runs at Ubuntu 16.04/18.04 and ROS Kinetic/Melodic, and correlating installation can be found at  [ROS Installation](http://wiki.ros.org/ROS/Installation)



## 4. Assembly

The main difficulty of the assembly lies at the weld of the power suit and the configuration and tuning of the FCU. Please refer to [assemble.pdf](assemble.pdf). 

>>>>>>> ea6f59038fc6e77a40586593a6e9ddbc699b6fb7
The installation of the onboard computer can be done by yourself.