
# Pepperl+Fuchs SmartRunner Explorer 3-D Driver

This is the documentation of a driver for the Pepperl+Fuchs SmartRunner Explorer 3-D.
The driver has a ROS-Node interface to the Robot Operating System (<http://www.ros.org>).


## Sensor information

The SmartRunner Explorer 3-D generates high-precision 3-D point cloud images in addition to 2-D images. 
It is optionally available with stereo vision technology or time-of-flight (ToF) technology. 
As a raw data sensor, the 3-D vision sensor is suitable for a wide variety of applications.

Official Website: https://www.pepperl-fuchs.com/global/en/smartrunner_3-d.htm

Datasheet (en): https://files.pepperl-fuchs.com/webcat/navi/productInfo/doct/tdoct7242a_eng.pdf?v=20220207113707


## Usage with ROS
The ROS package `sr3d_driver` consists of the driver library and a node named `sr3D_node`.


#### Supported platforms:
- Ubuntu 20.04 / ROS Noetic
- Ubuntu 18.04 / ROS Melodic

#### Published topics
The following standard ROS messages are supported. The messages contain the measured data.
- `scan` (`sensor_msgs/PointCloud`) 
- `scan` (`sensor_msgs/PointCloud2`) 

#### Parameters

- `frame_id` - The frame-ID in the header of the published `sensor_msgs/PointCloud` messages
- `scanner_ip` - IP address or hostname of the sensor
- `message_type` - The type of the PointCloud-Message (PointCloud or PointCloud2)

#### Installation guide
1. Clone the repository in the src folder of your ROS workspace.  
```git clone https://github.com/PepperlFuchs/pf_smartrunner3d_ros_driver.git```
2. Download the library from the Pepperl+Fuchs site.  
   (https://www.pepperl-fuchs.com/global/en/classid_9866.htm?view=productdetails&prodid=117291#software)
3. Unzip the downloaded file and copy the folder "VsxSdk" in to the folder:  
```<home>/catkin_ws/src/pf_sr3d_ros_driver/sr3d_driver/lib/```
4. Install .NET SDK.  
   (https://learn.microsoft.com/en-us/dotnet/core/install/linux-ubuntu-2004)
5. Change to the workspace directory.  
```<home>/catkin_ws/```
6. Run the command.   
```source devel/setup.bash```
7. Start the compilation.  
```catkin_make install```

#### Launch the driver
1. Launch the ros system. ```roscore```
2. Set the IP-Address (default 192.168.2.4) of the sensor in (for PointCloud):
```<home>/catkin_ws/src/pf_sr3d_ros_driver/sr3d_driver/launch/sr3d.launch```
or (for PointCloud2)
```<home>/catkin_ws/src/pf_sr3d_ros_driver/sr3d_driver/launch/sr3d_2.launch```
3. Run the command (for PointCloud):
```roslaunch pepperl_fuchs_sr3d sr3d.launch```
or (for PointCloud2)
```roslaunch pepperl_fuchs_sr3d sr3d_2.launch```
4. This starts `RViz` (http://wiki.ros.org/rviz) and the driver.
5. The 3-D measure points coming from the sensor are shown in the window.


