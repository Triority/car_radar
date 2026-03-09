# 毫米波雷达点云匹配与滤波
## 启动命令(path:~/Desktop/car_radar)
+ rviz
    ```
    rviz2 bag/rviz.rviz
    ```
+ bag
    ```
    ros2 bag play bag/test_20260121/. --clock -l
    ```
+ TF:
    ```
    ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 0 base_link map
    ```
    ```
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 lidar_link rslidar
    ```
+ radar_filter_node
    ```
    ros2 run radar_filter_ai radar_filter_node --ros-args -p use_sim_time:=true
    ```

## 调试命令
