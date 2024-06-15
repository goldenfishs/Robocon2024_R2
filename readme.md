# 导航代码使用文档

## 环境配置

### 系统配置

- Ubuntu22.04
- ROS Humble (desktop-full)

### 库依赖

- LIVOX-SDK2
- Libpcl-ros-dev
- eigen、pcl、opencv、ceres等 

### 硬件配置

- 3D雷达（mid360）

### 安装依赖

```shell
sudo apt-get update
sudo apt install ros-humble-serial-driver
sudo apt install ros-humble-urdf-tutorial
sudo apt-get install libeigen3-dev libpcl-ros-dev
sudo apt install ros-humble-navigation2 ros-humble-nav2-*
sudo apt install -y ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-tf2* libgoogle-glog-dev ros-humble-libpointmatcher

sudo apt install -y ros-humble-gazebo-*
sudo apt install -y ros-humble-xacro
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-rviz2
sudo apt install -y ros-humble-nav2*
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-pcl-ros
sudo apt install -y ros-humble-pcl-conversions
sudo apt install -y ros-humble-libpointmatcher
sudo apt install -y ros-humble-tf2-geometry-msgs
sudo apt install -y libboost-all-dev
sudo apt install -y libgoogle-glog-dev
```

### 安装Livox-SDK2

```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 编译

```shell
colcon build
```

## 配置参数

### mid360配置

- 修改rm_sensors/livox_ros_driver2/config/MID360_config.json
将雷达ip改成192.168.1.1xx   （xx为雷达广播码后两位）

```json
"host_net_info" : {
      "cmd_data_ip" : "192.168.1.50",  # host ip
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.50",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.50",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.106",  # 修改为192.168.1.1+广播码后两位
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
```

- 修改有线连接ip
（修改ubuntu有线连接IPv4，修改如下图，地址与1.1中用户IP相同）
![Alt text](doc/3.png)

## 运行

- 修改文件参数
修改rm_perception/icp_localization_ros2/config/node_params.yaml中的点云图路径

```yaml
/icp_localization:
  ros__parameters:
    pcd_file_path: "/home/ubuntu/rc2024/nav/test.pcd"  # 需要修改点云图路径
```

- 修改rm_navigation/launch/bringup_launch.py中的地图文件路径

```py
# 需要修改的地图文件
declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value= os.path.join(bringup_dir,'map', 'map.yaml'), # 需要修改yaml文件
        description='Full path to map yaml file to load')
```

## 建图

```shell
. map.sh
```

- 保存地图，运行rqt，选择/map_save服务，点击call保存点云pcd图

```shell
source install/setup.bash
rqt
```

![Alt text](doc/2.png)

- 再运行nav2_map_server功能包的map_saver_cli节点保存，加上-f参数，保存在当前运行命令的文件夹下，map为保存的地图名字

```shell
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f map
```

## 修改代码参数

### 雷达外参

- 若3d雷达有倒置、倾斜等放置姿态需求，需要修改rm_sensors/livox_ros_driver2/config/MID360_config.json中的extrinsic_parameter

```js
"lidar_configs" : [
    {
      "ip" : "192.168.1.176",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 180.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
```

- 需要修改3d雷达的离地高度，在rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/launch/segmentation_params.yaml 中修改

```js
ground_segmentation:
  ros__parameters:
    sensor_height: 0.23         # sensor height above ground.

```

- 机器人外参
在rm_navigation/params/nav2_params.yaml 中修改local_costmap和global_costmap的机器人半径

```js
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.2
      
      
global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.20
```

## 脚本使用和常用命令

快速发布目标点命令：ros2 run rc_decision goal_id_pub_node
动态调节预设参数：ros2 run rc_decision param_id_pub_node
测试机器人状态：ros2 topic pub /robot_mode std_msgs/msg/Int64 "data: 2"
比赛启动指令：ros2 topic pub /game std_msgs/msg/String "data: 'start'"
查看当前坐标：ros2 run tf2_ros tf2_echo map base_link