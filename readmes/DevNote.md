
## 硬件链路

- Companion Computer: 
  - USB - Telem2 (serial) connect
    parammeters setting: https://docs.px4.io/master/en/peripherals/mavlink_peripherals.html#example
    on Jetson: SER_TEL2_BAUD = 460800, MAV_2_RATE = 40000 B/s
    ref:
    [通过MAVROS连接机载电脑（NANO/TX2/NX）与Pixhawk - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/364390798) 
    [Companion Computer for Pixhawk Series | PX4 User Guide](https://docs.px4.io/master/en/companion_computer/pixhawk_companion.html) 
    [Serial Port Configuration | PX4 User Guide](https://docs.px4.io/master/en/peripherals/serial_configuration.html) 
    check: rostopic echo /mavros/state
    get: connected: True
  - Wifi module wiring: 
    <img src="DevNote.assets/2017011714481954-977597-16516703043761.jpg" alt="2017011714481954-977597" style="zoom: 50%;" />
    - Cube Orange:
      [Cube Wiring Quick Start | PX4 User Guide](https://docs.px4.io/v1.12/en/assembly/quick_start_cube.html) 
      [Cube Orange Flight Controller | PX4 User Guide](https://docs.px4.io/master/en/flight_controller/cubepilot_cube_orange.html) 
      pinout: [The Cube Module Overview - CubePilot](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview) 

## PX4二次开发

- 外设兼容：要求1.12以上（基于1.12.3开发）

- PX4 setup (Ubuntu 20)
  git repository: https://github.com/ErcBunny/DRL-Autopilot 
  同时推进的项目: [OmniHex: Omnidirectional Hexacopter (notion.site)](https://ryan-yql.notion.site/OmniHex-Omnidirectional-Hexacopter-2dcce2cdaf9e43e4810e976ceec3730f) 

  - clone -r
    submodule update --init --recursive

  - switch to ROS1 branch

  - setup: `Ubuntu.sh`

  - build ros1 workspace
    note: `catkin bulid`, not `catkin_make` 

    - Error of installing GeographicLib while installing MAVROS: 
      [GeographicLib: install_geographiclib_datasets.sh需要下载的内容 (gitee.com)](https://gitee.com/MrZhaosx/geographic-lib) 
      [GeographicLib: Installing GeographicLib (sourceforge.io)](https://geographiclib.sourceforge.io/html/install.html) 
      [执行 install_geographiclib_datasets.sh 错误--原因被墙---解决--_飞同学的博客-CSDN博客](https://blog.csdn.net/z1872385/article/details/122636411) 
    
  - run simulation of PX4 in Gazebo: `roslaunch px4 posix_sitl_customized.launch`
    http://docs.px4.io/master/en/simulation/ros_interface.html
    make and source before the first run

    ```sh
    cd <PX4-Autopilot_clone>
    DONT_RUN=1 make px4_sitl_default gazebo
    
    source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
    ```

    - `rospy.exceptions.ROSTimeMovedBackwardsException: ROS time moved backwards`
      set `use_sim_time` false in `posix_sitl.launch`, hence we have `posix_sitl_customized.launch` 
    - `make px4_sitl_default gazebo` is not recommended
      which use Gazebo just as a debugger, and will not publish `gazebo/state` in ros
      `make` command: http://docs.px4.io/master/en/dev_setup/building_px4.html#px4-make-build-targets 

- run QGC and MAVROS
  MAVROS: 

  - `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"` (SITL)
  - `roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0:57600" gcs_url:="udp://@localhost"` (Real-USB) 
    note: disable `AutoConnect` to `SiK Radio` in QGC, otherwise mavros can not access `ttyUSBx` 
  - `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" gcs_url:="udp://@localhost"`
    (Real-WiFi)
    wifi module: set `UgCS Port` to `14540` (default port of mavros)

- test: 

  - takeoff in offboard mode
    `rosrun motion_capture_fake offb_node` 
  - follow a trajectory
    `rosrun px4 mavros_offboard_posctl_test_py3.py` (GPS is required for global_pos_lock)

- override and save log data
  `rosrun motion_capture_fake data_logger.py`
  (depend on package mavros_msg in ros1_workspace, source it before making)

- 控制输出覆写：mavros & uorb 
  https://docs.px4.io/v1.12/en/ros/mavros_custom_messages.html
  https://zhuanlan.zhihu.com/p/409410385 
  在MAVROS下添加自定义插件，飞控上添加自定义模块
- 
- PX4 loading firmware and sensor colibration 

  - https://docs.px4.io/v1.12/en/config/firmware.html
    https://docs.px4.io/v1.12/en/dev_setup/building_px4.html#nuttx-pixhawk-based-boards 
    https://docs.px4.io/v1.12/en/flight_controller/pixhawk-2.html 
    [2020-04-27 20分钟入门，利用QGC下载px4固件、校准传感器，注意事项](https://www.bilibili.com/video/BV1M54y1R71u) 
    Hex Cube Orange build command: `make cubepilot_cubeorange` [cubeorange](https://docs.px4.io/v1.12/en/flight_controller/cubepilot_cube_orange.html) 
    file dir: `PX4-Autopilot/build/cubepilot_cubeorange_default/cubepilot_cubeorange_default.px4` 
    Note: build command of Cube Orange is different to Cube Black
    
  - compass calibration: 
    do not wait for the prompt, QGC prompt depend on your motion
    waiting for prompt will cause failure: `calibration failed: timeout: no motion` 

  - px4 does not have 'loiter' mode, but it has 'position' instead
    - successfully saved via QGC

  - publisher mavros topic: vision_pose_estimate 
    http://wiki.ros.org/mavros_extras#vision_pose_estimate
      
  - parameter setting: 
    https://docs.px4.io/master/en/ros/external_position_estimation.html
    set EKF2_AID_MASK, EKF2_HGT_MODE as docs
    then we will get: 
    `INFO  [ecl/EKF] starting vision pos fusion`
    `INFO  [ecl/EKF] starting vision yaw fusion`
    if we uncheck GPS in EKF2_AID_MASK, no `global_position` will be published until
    the origin/home are manually setted
    
  - check:
    ```bash
    rostopic echo /mavros/local_position/pose
    ```
  
  

