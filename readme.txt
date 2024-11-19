编译时可能遇到的报错：
1、Could not find a package configuration file provided by "catkin_virtualenv"

解决办法：sudo apt install ros-noetic-catkin-virtualenv

2、ERROR: launchpadlib 1.10.13 requires testresources, which is not installed.
解决办法：sudo apt install python3-testresources

3、ERROR: ModuleNotFoundError: No module named 'serial'.
解决办法：sudo apt install python3-pip
pip3 install serial
pip3 install pyserial

安装依赖：
sudo pip3 install transforms3d
sudo apt install ros-noetic-tf-transformations
或一键安装依赖：
rosdep install --from-paths src --ignore-src -r -y


运行设备别名规则文件:
sudo sh wheeltec_gnss.sh

使用NMEA协议解析WHEELTEC G60/G70：
roslaunch wheeltec_gps_driver wheeltec_nmea_driver.launch

--gps topic: /gps/fix
--gps frame_id: navsat_link

在RVIZ中记录经纬度轨迹(WHEELTEC G60/G70)：
roslaunch wheeltec_gps_driver nmea_gps_path.launch

使用UBLOX协议解析WHEELTEC G70模块(仅WHEELTEC G70)：：
roslaunch wheeltec_gps_driver wheeltec_ublox_driver.launch

--gps topic: /ublox_gps_node/fix
--gps frame_id: navsat_link

在RVIZ中记录经纬度轨迹(仅WHEELTEC G70)：
roslaunch wheeltec_gps_driver ublox_gps_path.launch
