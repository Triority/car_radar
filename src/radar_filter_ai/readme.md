# 毫米波雷达点云匹配与滤波
## build
```
colcon build --packages-select radar_filter_ai
```
## 启动命令(path:~/Desktop/car_radar)
+ rviz
    ```
    rviz2 -d bag/label.rviz --ros-args -p use_sim_time:=true
    ```
+ bag
    ```
    ros2 bag play bag/test_20260121/. --clock -l
    ```
+ TF:
    ```
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw -1.5708 --pitch 0 --roll 0 --frame-id base_link --child-frame-id radar_1 --ros-args -p use_sim_time:=true
    ```
    ```
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.1 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id rslidar --ros-args -p use_sim_time:=true
    ```
+ radar_filter_node
    ```
    ros2 run radar_filter_ai radar_filter_node --ros-args -p use_sim_time:=true
    ```

## 调试命令
