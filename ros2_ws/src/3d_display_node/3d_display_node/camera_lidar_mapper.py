import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import open3d as o3d
import torch
import struct
import sensor_msgs_py.point_cloud2 as pc2

class CameraLidarMapper(Node):
    def __init__(self):
        super().__init__('camera_lidar_mapper')
        self.bridge = CvBridge()
        self.fx = 320
        self.fy = 320
        self.cx = 160
        self.cy = 120

        # MiDaS derinlik modeli
        self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True)
        self.midas.eval()
        self.device = torch.device("cpu")
        self.midas.to(self.device)
        self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True).small_transform

        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, "/lidar/points", self.lidar_callback, 10)

        self.latest_lidar = np.empty((0, 3))
        self.pcd_total = o3d.geometry.PointCloud()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_resized = cv2.resize(cv_image, (320, 240))
            rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            input_tensor = self.midas_transforms(rgb).to(self.device)

            with torch.no_grad():
                prediction = self.midas(input_tensor)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=rgb.shape[:2],
                    mode="bicubic",
                    align_corners=False
                ).squeeze()
                depth_map = prediction.cpu().numpy()

            cam_points = self.depth_to_point_cloud(depth_map)
            pc_cam = o3d.geometry.PointCloud()
            pc_cam.points = o3d.utility.Vector3dVector(cam_points)

            if len(self.latest_lidar) > 0:
                pc_lidar = o3d.geometry.PointCloud()
                pc_lidar.points = o3d.utility.Vector3dVector(self.latest_lidar)

                reg = o3d.pipelines.registration.registration_icp(
                    pc_cam, pc_lidar, 1.0, np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPoint()
                )
                aligned = pc_cam.transform(reg.transformation)
                self.pcd_total += aligned + pc_lidar
            else:
                self.pcd_total += pc_cam

            # Görüntüleme (tek pencere)
            o3d.visualization.draw_geometries([self.pcd_total], window_name="Harita", width=800, height=600)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def lidar_callback(self, msg):
        try:
            points = []
            for p in pc2.read_points(msg, skip_nans=True):
                points.append([p[0], p[1], p[2]])
            self.latest_lidar = self.filter_lidar_fov(np.array(points))
        except Exception as e:
            self.get_logger().error(f"Lidar parsing error: {e}")

    def depth_to_point_cloud(self, depth_map):
        h, w = depth_map.shape
        i, j = np.meshgrid(np.arange(w), np.arange(h))
        z = depth_map
        x = (i - self.cx) * z / self.fx
        y = (j - self.cy) * z / self.fy
        return np.stack((x, y, z), axis=-1).reshape(-1, 3)

    def filter_lidar_fov(self, points, angle_deg=60):
        angles = np.arctan2(points[:, 1], points[:, 0]) * 180 / np.pi
        mask = (angles > -angle_deg/2) & (angles < angle_deg/2)
        return points[mask]

def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
