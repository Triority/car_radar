import rclpy
from rclpy.node import Node
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud  # 核心工具：点云变换

class RadarFilter4DNode(Node):
    def __init__(self):
        super().__init__('radar_filter_node')

        # --- 1. 参数配置 ---
        self.base_dist_threshold = 0.15  # 基础距离误差范围
        self.dist_coeff = 0.05           # 距离动态系数（远处的点允许更大的误差）
        self.angle_threshold_rad = np.radians(5.0) # 角度搜索锥半角

        # --- 2. TF 监听基础设施 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_lidar_msg = None

        # --- 3. QoS 配置 ---
        # 使用 RELIABLE 确保在网络负载高时不丢弃关键包
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.lidar_sub = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_callback, qos)
        self.radar_sub = self.create_subscription(PointCloud2, '/oden_1/extended_point_cloud', self.radar_callback, qos)
        self.publisher = self.create_publisher(PointCloud2, '/radar/filtered_points', qos)

        self.get_logger().info("Radar Filter Node Started (Using do_transform_cloud)")

    def lidar_callback(self, msg):
        # 简单缓存最新的激光点云
        self.latest_lidar_msg = msg

    def radar_callback(self, radar_msg):
        if self.latest_lidar_msg is None:
            return

        # --- A. 坐标对齐 (TF) ---
        try:
            # 查找从 Lidar 到 Radar 的变换矩阵
            # 关键：使用 radar_msg.header.stamp 进行时间同步对齐
            trans = self.tf_buffer.lookup_transform(
                radar_msg.header.frame_id,      # 目标坐标系 (Radar)
                self.latest_lidar_msg.header.frame_id, # 源坐标系 (Lidar)
                radar_msg.header.stamp,         # 匹配毫米波的时间戳
                timeout=rclpy.duration.Duration(seconds=0.1))
            
            # 使用 ROS 2 官方工具直接变换整个激光点云消息
            # 这一步之后，aligned_lidar_msg 的坐标系已经和 radar 坐标系完全重合
            aligned_lidar_msg = do_transform_cloud(self.latest_lidar_msg, trans)

        except TransformException as ex:
            # 过滤掉启动初期的 TF 未就绪错误
            return

        # --- B. 数据提取与 NumPy 化 ---
        # 激光：提取 (N, 3) 的 float32 矩阵。read_points_numpy 速度极快
        l_pts = pc2.read_points_numpy(aligned_lidar_msg, field_names=("x", "y", "z"), skip_nans=True)
        if l_pts.size == 0: return
        
        # 毫米波：先转成 list 方便索引，再提数值用于计算
        radar_raw_list = list(pc2.read_points(radar_msg, skip_nans=True))
        r_pts = np.array([[p[0], p[1], p[2]] for p in radar_raw_list])
        total_radar_count = len(radar_raw_list)
        if total_radar_count == 0: return

        # --- C. 坐标转换：笛卡尔 (x,y,z) -> 球坐标 (r, az, el) ---
        # 激光点极坐标化
        l_r = np.linalg.norm(l_pts, axis=1)
        l_azimuth = np.arctan2(l_pts[:, 1], l_pts[:, 0])
        l_elevation = np.arctan2(l_pts[:, 2], np.linalg.norm(l_pts[:, :2], axis=1))

        # 毫米波点极坐标化
        r_r = np.linalg.norm(r_pts, axis=1)
        r_azimuth = np.arctan2(r_pts[:, 1], r_pts[:, 0])
        r_elevation = np.arctan2(r_pts[:, 2], np.linalg.norm(r_pts[:, :2], axis=1))

        # --- D. 空间交叉验证过滤 ---
        verified_indices = []
        for i in range(total_radar_count):
            # 动态距离阈值：目标越远，允许的径向误差越大
            curr_th = self.base_dist_threshold + (r_r[i] * self.dist_coeff)
            
            # 1. 距离粗筛：找出所有到原点距离相近的激光点
            dist_mask = np.abs(l_r - r_r[i]) < curr_th
            
            if np.any(dist_mask):
                # 2. 角度精筛：在距离接近的点中，计算方位角和俯仰角偏差
                d_az = np.abs(l_azimuth[dist_mask] - r_azimuth[i])
                # 处理角度环绕 (π 和 -π 是同一个方向)
                d_az = np.where(d_az > np.pi, 2*np.pi - d_az, d_az)
                
                d_el = np.abs(l_elevation[dist_mask] - r_elevation[i])
                
                # 如果存在激光点在毫米波点的“搜索锥”内，则认为该点是真实的
                if np.any((d_az < self.angle_threshold_rad) & (d_el < self.angle_threshold_rad)):
                    verified_indices.append(i)

        # --- E. 统计、发布与打印 ---
        verified_count = len(verified_indices)
        if verified_count > 0:
            # 根据索引提取原始毫米波数据（包含 4D 速度、强度等所有字段）
            verified_points_raw = [radar_raw_list[idx] for idx in verified_indices]
            filtered_msg = pc2.create_cloud(radar_msg.header, radar_msg.fields, verified_points_raw)
            self.publisher.publish(filtered_msg)

        # 打印统计信息
        percentage = (verified_count / total_radar_count) * 100 if total_radar_count > 0 else 0.0
        self.get_logger().info(
            f"Verified: {verified_count}/{total_radar_count} ({percentage:.1f}%)"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RadarFilter4DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    