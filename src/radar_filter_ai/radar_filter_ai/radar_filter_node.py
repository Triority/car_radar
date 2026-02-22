import rclpy
from rclpy.node import Node
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException

class RadarFilterAdaptiveNode(Node):
    def __init__(self):
        super().__init__('radar_filter_node')

        # --- 1. 匹配参数配置 ---
        self.base_dist_threshold = 0.3   # 基础距离阈值 (米)
        self.dist_coeff = 0.05           # 距离系数 (每米增加5cm容差)
        self.angle_threshold_deg = 5.0   # 角度阈值 (度)
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

        self.get_logger().info("Adaptive Radar Filter Node Started...")

    def lidar_callback(self, msg):
        self.latest_lidar_msg = msg

    def radar_callback(self, radar_msg):
        if self.latest_lidar_msg is None:
            return

        # 1. 获取 TF 变换
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                radar_msg.header.frame_id,      
                self.latest_lidar_msg.header.frame_id, 
                now)
        except TransformException as ex:
            self.get_logger().error(f"TF Error: {ex}")
            return

        # 2. 转换激光点到毫米波坐标系
        lidar_gen = list(pc2.read_points(self.latest_lidar_msg, field_names=("x", "y"), skip_nans=True))
        l_pts = np.array([[p[0], p[1]] for p in lidar_gen])
        if l_pts.size == 0: return
        
        t = transform.transform.translation
        q = transform.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        c, s = np.cos(yaw), np.sin(yaw)
        l_x_trans = (l_pts[:, 0] * c - l_pts[:, 1] * s) + t.x
        l_y_trans = (l_pts[:, 0] * s + l_pts[:, 1] * c) + t.y
        
        l_rho = np.sqrt(l_x_trans**2 + l_y_trans**2)
        l_theta = np.arctan2(l_y_trans, l_x_trans)

        # 3. 处理毫米波点
        radar_gen = list(pc2.read_points(radar_msg, skip_nans=True))
        total_radar_count = len(radar_gen)
        if total_radar_count == 0: return
        
        r_pts = np.array([[p[0], p[1]] for p in radar_gen])
        r_rho = np.sqrt(r_pts[:, 0]**2 + r_pts[:, 1]**2)
        r_theta = np.arctan2(r_pts[:, 1], r_pts[:, 0])

        real_points_list = []
        
        # --- 核心修改：动态阈值遍历 ---
        for i in range(total_radar_count):
            # 计算当前点对应的动态距离阈值
            # 举例：在50米处，阈值 = 0.3 + 50*0.05 = 2.8米
            current_dynamic_dist_threshold = self.base_dist_threshold + (r_rho[i] * self.dist_coeff)
            
            # 匹配距离
            dist_diff = np.abs(l_rho - r_rho[i])
            close_mask = dist_diff < current_dynamic_dist_threshold
            
            if np.any(close_mask):
                # 匹配角度
                angle_diff = np.abs(l_theta[close_mask] - r_theta[i])
                angle_diff = np.where(angle_diff > np.pi, 2*np.pi - angle_diff, angle_diff)
                
                if np.any(angle_diff < self.angle_threshold_rad):
                    real_points_list.append(radar_gen[i])

        # 4. 发布与日志输出
        verified_count = len(real_points_list)
        if verified_count > 0:
            filtered_msg = pc2.create_cloud(radar_msg.header, radar_msg.fields, real_points_list)
            self.publisher.publish(filtered_msg)
        
        # 修改后的日志输出：同时显示 验证通过数 / 总点数
        # 计算百分比并格式化输出
        if total_radar_count > 0:
            percentage = int((verified_count / total_radar_count) * 100)
            self.get_logger().info(f"Published {verified_count} / {total_radar_count} ({percentage}%) points")
        else:
            self.get_logger().info(f"Total radar count is 0")

def main(args=None):
    rclpy.init(args=args)
    node = RadarFilterAdaptiveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    