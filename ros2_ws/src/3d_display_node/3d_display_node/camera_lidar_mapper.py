#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
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

        # Kamera iç parametreleri
        self.fx, self.fy = 320, 320
        self.cx, self.cy = 160, 120

        # MiDaS derinlik modeli
        self.device = torch.device("cpu")
        self.midas = torch.hub.load("intel-isl/MiDaS",
                                    "MiDaS_small",
                                    trust_repo=True).to(self.device).eval()
        self.midas_tfms = torch.hub.load("intel-isl/MiDaS",
                                         "transforms",
                                         trust_repo=True).small_transform

        # ROS2 abone / yayımlayıcı
        self.image_sub = self.create_subscription(Image,
                                                  "/camera/image_raw",
                                                  self.image_cb, 10)
        self.lidar_sub = self.create_subscription(PointCloud2,
                                                  "/lidar/points",
                                                  self.lidar_cb, 10)
        self.cloud_pub = self.create_publisher(PointCloud2,
                                               "/fused_cloud", 10)

        self.latest_lidar = np.empty((0, 3), dtype=np.float32)
        self.pcd_total = o3d.geometry.PointCloud()

    # ---------- Call-back’ler ----------
    def image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb = cv2.cvtColor(cv2.resize(cv_img, (320, 240)),
                               cv2.COLOR_BGR2RGB)

            # Derinlik tahmini
            inp = self.midas_tfms(rgb).to(self.device)
            with torch.no_grad():
                pred = self.midas(inp)
                pred = torch.nn.functional.interpolate(
                    pred.unsqueeze(1), size=rgb.shape[:2],
                    mode="bicubic", align_corners=False
                ).squeeze()
                depth_map = pred.cpu().numpy()

            # Kameradan nokta bulutu
            pc_cam = o3d.geometry.PointCloud()
            pc_cam.points = o3d.utility.Vector3dVector(
                self.depth_to_pc(depth_map))

            # LIDAR ile hizalama (ICP)
            if len(self.latest_lidar):
                pc_lidar = o3d.geometry.PointCloud()
                pc_lidar.points = o3d.utility.Vector3dVector(
                    self.latest_lidar)
                reg = o3d.pipelines.registration.registration_icp(
                    pc_cam, pc_lidar, 1.0, np.eye(4),
                    o3d.pipelines.registration.
                    TransformationEstimationPointToPoint())
                pc_cam.transform(reg.transformation)
                self.pcd_total += pc_cam + pc_lidar
            else:
                self.pcd_total += pc_cam

            # RViz2’ye yayınla
            self.publish_cloud(self.pcd_total)

        except Exception as e:
            self.get_logger().error(f"Image error: {e}")

    def lidar_cb(self, msg: PointCloud2):
        try:
            pts = np.array([[p[0], p[1], p[2]]
                            for p in pc2.read_points(msg,
                                                     skip_nans=True)],
                           dtype=np.float32)
            self.latest_lidar = self.fov_filter(pts)
        except Exception as e:
            self.get_logger().error(f"Lidar error: {e}")

    # ---------- Yardımcı fonksiyonlar ----------
    def depth_to_pc(self, depth):
        h, w = depth.shape
        i, j = np.meshgrid(np.arange(w), np.arange(h))
        z = depth.flatten()
        x = (i.flatten() - self.cx) * z / self.fx
        y = (j.flatten() - self.cy) * z / self.fy
        return np.stack((x, y, z), axis=-1)

    def fov_filter(self, pts, angle=60):
        ang = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        mask = (-angle/2 < ang) & (ang < angle/2)
        return pts[mask]

    def publish_cloud(self, pcd: o3d.geometry.PointCloud):
        pts_np = np.asarray(pcd.points, dtype=np.float32)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"                 # RViz2’de TF’nize göre değiştirin
        cloud_msg = pc2.create_cloud_xyz32(header, pts_np.tolist())
        self.cloud_pub.publish(cloud_msg)

# ---------- main ----------
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

if __name__ == "__main__":
    main()
