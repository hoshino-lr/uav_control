# Control UAV

------

px4 入门
-------
   1. 阅读相关文档
      1. [PX4官方文档](https://docs.px4.io/)
      2. [mavros官方文档](http://wiki.ros.org/mavros)
      3. [mavlink官方文档]()


-----

# tips
1. 使用MAVLINK提升Px4飞控的数据采样率
   1. 在qgc中使用
       ```` Shell
       mavlink stream -d /dev/ttyS1 -s SERVO_OUTPUT_RAW_0 -r 50
       ````
   2. 在gazebo仿真中使用
      ```` Shell
      mavlink stream -u 14557 -s SERVO_OUTPUT_RAW_0 -r 50
      ````
2. launch文件使用
   1. mavros.launch 
      > 作为最终实机程序运行的文件
   2. motion_capture
      > 启动动捕文件
   
3. record data
   ```` Shell
   rosbag record /mavros/imu/data /mavros/setpoint_raw/attitude /px4/vision_odom /mavros/battery /mavros/servo_output_raw -o uav_t2.bag
   ````
4. 正确通过wifi连接飞控的方式
   ```` Shell
   roslaunch mavros px4.launch fcu_url:="udp://:14540@{无人机的ip:port}" gcs_url:=udp://@{地面站的ip}
   roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.1:14557" gcs_url:=udp://@192.168.1.2
   ````
