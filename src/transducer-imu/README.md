# tm_imu
This package implements a ROS2 node of TransducerM AHRS/IMU from SYD Dynamics. 

# Acknowledge
New updates: The package converts the original ROS1 node to work in ROS2. The function `onSerialRX` that reads the values from the sensor is called in a timer callback function. Special thanks to Antonis and fellow colleagues from ROICO Solutions ApS for providing value contributions for the conversion.

# Dependencies
All the required library are contained inside the `src/` folder and the 'external/' folder.
The 'external/EasyProfile' folder includes the TransducerM communication library from SYD Dynamics.
The 'external/seriallib'   folder includes the [seriallib](https://github.com/imabot2/serialib?tab=MIT-1-ov-file), version 1.2 (from april 2011).

# Messages
This package publishes message types:
- sensor_msgs/Imu :            The standard Imu message from ROS
- sensor_msgs/MagneticField:   The standard Imu mag message from ROS

# Topics
This package publishes three topics:
-imu_data     : Standard ROS imu data package, including quaternion data, acceleration data, and gyroscope data
-imu_data_mag : Standard ROS magneticfield data package
-imu_data_rpy : This topic shares the same data structure as the standard ROS imu_data_mag topic, while it is used for publishing Roll, Pitch and Yaw output from TransducerM. The definition of data fields shown as below:
                imu_data_raw_rpy.magnetic_field.x =  roll                    
                imu_data_raw_rpy.magnetic_field.y =  pitch
                imu_data_raw_rpy.magnetic_field.z =  yaw
                
# How to use it
1. Create a work folder:
       $mkdir -p ~/imu_ws/src
2. Copy all source code of this ROS2 package into the src folder created above
3. Setup parameters in ./config/params.yaml
       For example: change imu_port value to your actual serial port name and setup baudrate
4. Compile 
       $cd .. 
           Then if you run $pwd the path should be similar as below:
           /home/your_user_name/imu_ws
       $colcon build 
       $cd ~
       $source ~/imu_ws/install/setup.sh 
       $ros2 launch tm_imu imu.launch.py
5. View topics
       $ros2 topic list 
       $ros2 topic echo /imu_data
6. View topics in RVIZ
       $ros2 run rviz2 rviz2
       Add tf display, Set fixed axis to 'world'.
       Please also refer to the screenshot of an example setup ./screenshot_rviz2.png
   
# Parameters
./config params.yaml includes the following parameters:
- `imu_baudrate` default value `115200` :        Set the communication speed between the IMU and the computer.
- `imu_port`     default value `"/dev/ttyUSB0"`: Specify the serial port where the sensor is connected.
- `imu_frame_id` default value `"imu"`  :        Specify the frame id of the sensor. 
- `timer_period` default value `5`      :        Set the timer period reading TransducerM serial port data in ms (Please rebuild this package after changing this value for it to take effect).


If you find any issue please leave a message at [our website](https://www.syd-dynamics.com/contact-us/)

Last updated on Aug 1, 2024




