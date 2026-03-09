import rclpy
from rclpy.node import Node
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException

class RadarFilter4DNode(Node):
    def __init__(self):
        super().__init__('radar_filter_node')

        # --- 1. 匹配参数配置 ---
        self.base_dist_threshold = 0.15  # 基础径向距离阈值 (米)
        self.dist_coeff = 0.05           # 距离系数
        self.angle_threshold_deg = 5.0   # 角度阈值 (3度锥角)
        self.angle_threshold_rad = np.radians(self.angle_threshold_deg)

        # --- 2. TF 与 缓存 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_lidar_msg = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.lidar_sub = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_callback, qos)
        self.radar_sub = self.create_subscription(PointCloud2, '/oden_1/extended_point_cloud', self.radar_callback, qos)
        self.publisher = self.create_publisher(PointCloud2, '/radar/filtered_points', qos)

        self.get_logger().info("4D Adaptive Radar Filter Node (3D Matching) Started...")

    def lidar_callback(self, msg):
        self.latest_lidar_msg = msg

    def quaternion_to_rotation_matrix(self, q):
        """将四元数转换为3x3旋转矩阵"""
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
        ])

    def radar_callback(self, radar_msg):
        if self.latest_lidar_msg is None:
            return

        # 1. 获取完整的 3D TF 变换
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                radar_msg.header.frame_id,      
                self.latest_lidar_msg.header.frame_id, 
                now)
        except TransformException as ex:
            self.get_logger().error(f"TF Error: {ex}")
            return

        # 2. 转换激光点到毫米波坐标系 (3D 处理)
        # 提取 x, y, z
        l_gen = list(pc2.read_points(self.latest_lidar_msg, field_names=("x", "y", "z"), skip_nans=True))
        l_pts = np.array([[p[0], p[1], p[2]] for p in l_gen])
        if l_pts.size == 0: return
        
        # 应用 3D 旋转和平移
        rot_mat = self.quaternion_to_rotation_matrix(trans.transform.rotation)
        t_vec = np.array([trans.transform.translation.x, 
                          trans.transform.translation.y, 
                          trans.transform.translation.z])
        
        # 变换公式: P_radar = R * P_lidar + T
        l_pts_trans = np.dot(l_pts, rot_mat.T) + t_vec
        
        # 计算激光点在球坐标系下的值
        l_r = np.linalg.norm(l_pts_trans, axis=1) # 径向距离
        l_azimuth = np.arctan2(l_pts_trans[:, 1], l_pts_trans[:, 0]) # 方位角
        l_elevation = np.arctan2(l_pts_trans[:, 2], np.linalg.norm(l_pts_trans[:, :2], axis=1)) # 俯仰角

        # 3. 处理毫米波点 (4D 数据)
        radar_gen = list(pc2.read_points(radar_msg, skip_nans=True))
        total_radar_count = len(radar_gen)
        if total_radar_count == 0: return
        
        r_pts = np.array([[p[0], p[1], p[2]] for p in radar_gen])
        r_r = np.linalg.norm(r_pts, axis=1)
        r_azimuth = np.arctan2(r_pts[:, 1], r_pts[:, 0])
        r_elevation = np.arctan2(r_pts[:, 2], np.linalg.norm(r_pts[:, :2], axis=1))

        real_points_list = []
        
        # --- 3D 空间过滤循环 ---
        for i in range(total_radar_count):
            # 动态径向距离阈值
            current_dist_threshold = self.base_dist_threshold + (r_r[i] * self.dist_coeff)
            
            # A. 径向距离过滤
            dist_mask = np.abs(l_r - r_r[i]) < current_dist_threshold
            
            if np.any(dist_mask):
                # B. 在距离相近的激光点中，进行角度过滤（方位角+俯仰角）
                # 计算方位角差
                d_azimuth = np.abs(l_azimuth[dist_mask] - r_azimuth[i])
                d_azimuth = np.where(d_azimuth > np.pi, 2*np.pi - d_azimuth, d_azimuth)
                
                # 计算俯仰角差
                d_elevation = np.abs(l_elevation[dist_mask] - r_elevation[i])
                
                # C. 锥形匹配判断：方位角误差和俯仰角误差均在阈值内
                # (也可以用矢量夹角公式计算更精确的3D角度，此处用分量判断速度更快)
                angle_mask = (d_azimuth < self.angle_threshold_rad) & (d_elevation < self.angle_threshold_rad)
                
                if np.any(angle_mask):
                    real_points_list.append(radar_gen[i])

        # 4. 发布与统计
        verified_count = len(real_points_list)
        if verified_count > 0:
            filtered_msg = pc2.create_cloud(radar_msg.header, radar_msg.fields, real_points_list)
            self.publisher.publish(filtered_msg)
        
        # 终端输出
        percentage = (verified_count / total_radar_count) * 100 if total_radar_count > 0 else 0
        self.get_logger().info(f"Published {verified_count} / {total_radar_count} ({percentage:.0f}%) points")

def main(args=None):
    rclpy.init(args=args)
    node = RadarFilter4DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    