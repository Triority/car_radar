# 简介
由于原有文档不是错误百出(底盘)就是啥都没有(雷达)，在这里重写一份能看的。

毫米波雷达必须使用ros2 humble ubuntu22.04才能驱动，底盘有ros1和ros2两个版本。由于计划未来使用ros2开发因此底盘驱动包含在ros2系统中了。

# ip地址
+ can转以太网：192.168.0.10
+ ros1上位机：192.168.0.190
+ ros2上位机：192.168.0.191
+ 毫米波雷达：192.168.0.40
+ 固态激光雷达：192.168.0.200

# 底盘
## 启动
编译和底盘节点启动命令：
```
colcon build --symlink-install
colcon build --symlink-install --parallel-workers 1

source ./install/setup.bash

sudo cp ./src/chassis_driver/lib/x86/linux_64/libcontrolcan.so /usr/lib

ros2 launch chassis_driver chassis_driver.launch.py
```
编译依赖报错处理：
```
None of the required 'gstreamer-1.0>=1.4' found
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

None of the required 'gtkmm-3.0>=0.29.2' found
sudo apt install libgtkmm-3.0-dev
```
## launch配置
|       名称       |                可选值                 |            释义            |
| :--------------: | :-----------------------------------: | :------------------------: |
|     can_type     |                0 或 1                 |  默认为1,以太网的udp模式   |
|   main_can_id    |                1 或 2                 | 默认为2, 控制底盘的can的id |
|   can_eth_card   |         根据本机网口名称设置          |    网口名称查看ifconfig     |
|  can1_remote_ip  | 默认为192.168.1.10, 可在CAN上位机设置 |        CAN1的远程IP        |
| can1_remote_port |     默认为4001, 可在CAN上位机设置     |     CAN1的远程IP端口号     |
|  can2_remote_ip  | 默认为192.168.1.10, 可在CAN上位机设置 |        CAN2的远程IP        |
| can2_remote_port |     默认为4002, 可在CAN上位机设置     |     CAN2的远程IP端口号     |
|     local_ip     |       根据本机网口的IP进行设置        |   **本机连接CAN盒的IP**    |
|    local_port    |       根据本机网口的端口号设置        |        本机IP端口号        |
|    debug_mode    |             True 或 False             |      是否为debug模式       |
| show_sending_msg |             True 或 False             |      是否需要输出指令      |
|     car_type     |            NWD, JD01, JD03,TD         |        线控底盘型号        |
|  vehicle_weight  |                   /                   |             /              |

## 通信格式
+ **ecu**数据格式:

|    名称     | 数据类型 |                    释义                     |
| :---------: | :------: | :-----------------------------------------: |
|    motor    | float32  |  目标速度, 下发消息时需要设置, 单位: km/h   |
|    steer    | float32  |  目标转向, 下发消息时需要设置, 单位为角度   |
|    brake    |   bool   |            紧急停车, true为使能             |
|    shift    |  uint8   | 档位设置, 可设置为1--前进, 2--空档(请勿空挡使用), 3--后退 |

+ **vehicle_status**数据格式

|      名称       | 数据类型 |                      释义                      |
| :-------------: | :------: | :--------------------------------------------: |
|    cur_speed    | float32  |              当前车速, 单位: m/s               |
|    cur_steer    | float32  |           当前转向角度, 单位: 度(°)            |
| wheel_direction |  uint8   |           车轮转向方向，左转为1，右转为0 |
| left_wheel_speed | float32  |           左轮转速，单位: m/s           |
| right_wheel_speed | float32  |           右轮转速，单位: m/s           |
|   shift_level   |  uint8   |       当前档位 1--前进, 2--停止, 3--后退       |
|   drive_mode    |  uint8   | 驾驶模式, 1--遥控, 2--Reservation, 3--自动驾驶模式 |
|   is_autodrive  |  bool  | 自动驾驶按钮状态, 1--按钮已按下, 0--按钮未按下 |
|   p_status      |  bool  | 底盘刹车状态, 1--底盘刹车, 0--底盘未刹车 |

+ **battery**数据格式

|     名称      | 数据类型 |            释义            |
| :-----------: | :------: | :------------------------: |
|    voltage    | float32  |   动力电池 电压 单位：V    |
|    ampere     | float32  |   动力电池 电流 单位：A    |
|   capacity    | float32  | 动力电池 电量 单位：百分比 |
| bmusys_status |  uint8   |      电源管理系统状态      |
| charge_status |  uint8   |        电池充电状态        |


```
root@orangepi5:/car_radar# ros2 topic list
/battery_status
/ecu
/imu_raw
/parameter_events
/rosout
/vehicle_status
```
```
root@orangepi5:/car_radar# ros2 topic info /ecu
Type: yunle_msgs/msg/Ecu
Publisher count: 0
Subscription count: 1
```
```
root@orangepi5:/car_radar# ros2 interface show yunle_msgs/msg/Ecu
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

float32 motor # 目标速度
float32 steer # 转向
bool brake # 紧急停车
bool set_torque # 设置扭矩值(测试用，使用时不设置)
bool rear_wheel_flag # 后轮转向flag

uint8 shift # 档位
uint8 SHIFT_UNKNOWN = 0
uint8 SHIFT_D = 1 #前进档位
uint8 SHIFT_N = 2 #停止档位
uint8 SHIFT_R = 3 #后退档位
```
```
ros2 topic pub -1 /ecu yunle_msgs/msg/Ecu "{header: 'auto', motor: 0.0 ,steer: 30.0, brake: False, shift: 1}"

```
# 毫米波雷达
## 启动
```
source ./install/setup.bash
ros2 launch launch/standard.py
```
启动后发布雷达点云的话题名称为`/oden_1/extended_point_cloud`

## 数据转发到其他设备
为了使用[4DRadarSLAM](https://github.com/zhuge2333/4DRadarSLAM)算法，需要把数据转发到ros1。在ros2和ros1下分别新建一个包，雷达数据解析后使用tcp发送到ros1地址的12345端口并接收。

```bash
# 如果需要单独编译转发的包
colcon build --packages-select pointcloud_sender

ros2 run pointcloud_sender pointcloud_sender_node --ros-args -p dest_ip:="192.168.0.190" -p dest_port:=12345
```

在ros1系统内开启端口监听并接收数据：
```bash
rosrun pointcloud_receiver pointcloud_receiver_node _listen_port:=12345
# 如果需要临时发布tf变换
rosrun tf static_transform_publisher 0 0 0 0 0 0 map laser_frame 100
```

# 固态激光雷达
```bash
ros2 launch rslidar_sdk start.py
```
