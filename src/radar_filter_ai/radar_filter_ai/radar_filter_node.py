import rclpy
from rclpy.node import Node
import numpy as np
import os
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation as R

class RadarFilter4DNode(Node):
    def __init__(self):
        super().__init__('radar_filter_node')

        # --- 1. 参数配置 ---
        self.declare_parameter('save_data', True)
        self.declare_parameter('save_path', './radar_dataset')
        self.declare_parameter('file_prefix', 'run1')
        
        self.save_data = self.get_parameter('save_data').value
        self.save_path = self.get_parameter('save_path').value
        self.file_prefix = self.get_parameter('file_prefix').value

        # 匹配算法阈值
        self.base_dist_threshold = 0.2
        self.dist_coeff = 0.05
        self.angle_threshold_rad = np.radians(3.0)

        # --- 2. 存储路径初始化 ---
        if self.save_data:
            self.points_dir = os.path.join(self.save_path, 'points')
            self.labels_dir = os.path.join(self.save_path, 'labels')
            os.makedirs(self.points_dir, exist_ok=True)
            os.makedirs(self.labels_dir, exist_ok=True)
            
        self.frame_points_buffer = [] # 暂存点云特征数据
        self.frame_labels_buffer = [] # 暂存标签数据
        self.file_count = 0

        # --- 3. TF 与 订阅 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_lidar_msg = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        self.lidar_sub = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_callback, qos)
        self.radar_sub = self.create_subscription(PointCloud2, '/oden_1/extended_point_cloud', self.radar_callback, qos)
        self.publisher = self.create_publisher(PointCloud2, '/radar/labeled_points', qos)

        self.get_logger().info(f"Node Started. Storage: {self.save_path}")

    def lidar_callback(self, msg):
        self.latest_lidar_msg = msg

    def save_to_disk(self):
        """执行保存逻辑：每100帧一个文件，不满20帧丢弃"""
        num_frames = len(self.frame_points_buffer)
        if num_frames < 20:
            self.get_logger().warn(f"Buffer has only {num_frames} frames (<20), discarding.")
            self.frame_points_buffer.clear()
            self.frame_labels_buffer.clear()
            return

        filename = f"{self.file_prefix}_{self.file_count:03d}.npy"
        
        # 保存点云特征 [N, 8] 的对象数组
        np.save(os.path.join(self.points_dir, filename), 
                np.array(self.frame_points_buffer, dtype=object))
        
        # 保存标签 [N, 1] 的对象数组
        np.save(os.path.join(self.labels_dir, filename), 
                np.array(self.frame_labels_buffer, dtype=object))

        self.get_logger().info(f"Disk Write: Saved {num_frames} frames to {filename}")
        
        # 重置
        self.frame_points_buffer.clear()
        self.frame_labels_buffer.clear()
        self.file_count += 1

    def radar_callback(self, radar_msg):
        if self.latest_lidar_msg is None:
            return

        # 1. 坐标变换获取 (Lidar -> Radar)
        try:
            trans = self.tf_buffer.lookup_transform(
                radar_msg.header.frame_id,      
                self.latest_lidar_msg.header.frame_id, 
                rclpy.time.Time()) 
            
            l_pts_raw = pc2.read_points_numpy(self.latest_lidar_msg, field_names=("x", "y", "z"), skip_nans=True)
            q = trans.transform.rotation
            rotation = R.from_quat([q.x, q.y, q.z, q.w])
            t_vec = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            l_pts_in_radar = rotation.apply(l_pts_raw) + t_vec
        except Exception:
            return

        # 2. 提取激光极坐标用于匹配
        l_r = np.linalg.norm(l_pts_in_radar, axis=1)
        l_az = np.arctan2(l_pts_in_radar[:, 1], l_pts_in_radar[:, 0])
        l_el = np.arctan2(l_pts_in_radar[:, 2], np.linalg.norm(l_pts_in_radar[:, :2], axis=1))

        # 3. 提取毫米波雷达所有点及指定特征
        # 对应 Channels: x, y, z, power, doppler, range (由 topic echo 确认)
        gen = pc2.read_points(radar_msg, field_names=("x", "y", "z", "power", "doppler", "range"), skip_nans=True)
        radar_data_list = list(gen)
        total_radar_count = len(radar_data_list)
        if total_radar_count == 0: return

        # 预计算毫米波极坐标
        r_coords = np.array([[p[0], p[1], p[2]] for p in radar_data_list]) # x, y, z
        r_r = np.linalg.norm(r_coords, axis=1)
        r_az = np.arctan2(r_coords[:, 1], r_coords[:, 0])
        r_el = np.arctan2(r_coords[:, 2], np.linalg.norm(r_coords[:, :2], axis=1))

        # 4. 匹配逻辑
        verified_indices = set()
        for i in range(total_radar_count):
            curr_th = self.base_dist_threshold + (r_r[i] * self.dist_coeff)
            dist_mask = np.abs(l_r - r_r[i]) < curr_th
            if np.any(dist_mask):
                d_az = np.abs(l_az[dist_mask] - r_az[i])
                d_az = np.where(d_az > np.pi, 2*np.pi - d_az, d_az)
                d_el = np.abs(l_el[dist_mask] - r_el[i])
                if np.any((d_az < self.angle_threshold_rad) & (d_el < self.angle_threshold_rad)):
                    verified_indices.add(i)

        # 5. 构建保存用的数据结构 [N, 8] 和 [N, 1]
        # Channels: [x, y, z, r, az, el, doppler, power]
        current_frame_points = []
        current_frame_labels = []
        
        # 用于 ROS 发布的数据
        labeled_ros_list = []

        for i, p in enumerate(radar_data_list):
            is_real = 1.0 if i in verified_indices else 0.0
            
            # 提取 8 通道特征
            # p 顺序: 0:x, 1:y, 2:z, 3:power, 4:doppler, 5:range
            pt_feature = [
                p[0], p[1], p[2],        # x, y, z
                r_r[i], r_az[i], r_el[i],# r, az, el
                p[4],                    # doppler
                p[3]                     # power
            ]
            current_frame_points.append(pt_feature)
            current_frame_labels.append([is_real])
            
            # 发布用数据 (保留所有原始字段 + label)
            # 这里需要读取全部原始字段，不仅是上面选出的6个
            # 为了效率，我们直接复用上面的计算结果
            labeled_ros_list.append(list(p) + [is_real])

        # 6. 数据暂存
        if self.save_data:
            self.frame_points_buffer.append(np.array(current_frame_points, dtype=np.float32))
            self.frame_labels_buffer.append(np.array(current_frame_labels, dtype=np.float32))
            
            # 每 100 帧分段保存
            if len(self.frame_points_buffer) >= 100:
                self.save_to_disk()

        # --- 7. 发布 labeled 点云 (优化版：统一使用 Float32 以确保 RViz 兼容) ---
        from sensor_msgs.msg import PointField
        
        # 重新定义发布给 RViz 的字段，全部设为 Float32 (datatype=7)
        # 这样可以避免原始点云中 1字节/2字节 字段导致的对齐错误
        rviz_fields = [
            PointField(name="x", offset=0, datatype=7, count=1),
            PointField(name="y", offset=4, datatype=7, count=1),
            PointField(name="z", offset=8, datatype=7, count=1),
            PointField(name="doppler", offset=12, datatype=7, count=1),
            PointField(name="power", offset=16, datatype=7, count=1),
            PointField(name="label", offset=20, datatype=7, count=1),
        ]

        rviz_points_list = []
        for i, p in enumerate(radar_data_list):
            # 这里的 p 是我们之前提取的 6 个字段: (x, y, z, power, doppler, range)
            is_real = 1.0 if i in verified_indices else 0.0
            # 构造匹配 rviz_fields 的列表: [x, y, z, doppler, power, label]
            rviz_points_list.append([
                float(p[0]), float(p[1]), float(p[2]), 
                float(p[4]), float(p[3]), is_real
            ])

        # 创建消息
        labeled_msg = pc2.create_cloud(radar_msg.header, rviz_fields, rviz_points_list)
        
        # 确保 header.frame_id 正确
        labeled_msg.header = radar_msg.header 
        
        self.publisher.publish(labeled_msg)

        # 终端显示
        percentage = (len(verified_indices) / total_radar_count) * 100 if total_radar_count > 0 else 0
        self.get_logger().info(
            f"Matching: {len(verified_indices)}/{total_radar_count} ({percentage:.1f}%) | "
            f"Seq: {len(self.frame_points_buffer)}/100"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RadarFilter4DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 程序结束时保存缓冲区剩余数据
        if node.save_data:
            node.save_to_disk()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()