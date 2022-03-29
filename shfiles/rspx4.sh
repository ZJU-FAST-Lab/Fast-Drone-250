sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch realsense2_camera rs_camera.launch & sleep 10;
roslaunch mavros px4.launch & sleep 10;
roslaunch vins fast_drone_250.launch
wait;
