import os
import cv2
import numpy as np
import open3d as o3d
import torch

# MiDaS modeli yükle (CPU kullanımı için)
midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
midas.eval()
device = torch.device("cpu")  # sadece CPU kullan
midas.to(device)
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms").small_transform

# Kamera parametreleri (örnek: 320x240 görüntü)
fx, fy, cx, cy = 320, 320, 160, 120

# Derinlik haritasını 3D nokta bulutuna çevir
def depth_to_point_cloud(depth_map, fx, fy, cx, cy):
    h, w = depth_map.shape
    i, j = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_map
    x = (i - cx) * z / fx
    y = (j - cy) * z / fy
    return np.stack((x, y, z), axis=-1).reshape(-1, 3)

# LiDAR .bin dosyasını oku
def load_lidar_bin(path):
    lidar = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
    return lidar[:, :3]

# Sadece belirli görüş açısını (örn: 60°) filtrele
def filter_lidar_fov(points, angle_deg=60):
    angles = np.arctan2(points[:, 1], points[:, 0]) * 180 / np.pi
    fov_half = angle_deg / 2
    mask = (angles > -fov_half) & (angles < fov_half)
    return points[mask]

# Dosya yolları
video_path = "video.mp4"
lidar_dir = "lidar_frames"

cap = cv2.VideoCapture(video_path)
frame_id = 0
combined_cloud = o3d.geometry.PointCloud()

while cap.isOpened() and frame_id < 100:
    ret, frame = cap.read()
    if not ret:
        break

    # Görüntüyü işle
    frame_resized = cv2.resize(frame, (320, 240))
    rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
    input_tensor = midas_transforms(rgb).to(device)

    # Derinlik tahmini
    with torch.no_grad():
        prediction = midas(input_tensor)
        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=rgb.shape[:2],
            mode="bicubic",
            align_corners=False
        ).squeeze()
        depth_map = prediction.cpu().numpy()

    # Kamera point cloud
    cam_points = depth_to_point_cloud(depth_map, fx, fy, cx, cy)

    # LiDAR verisi oku ve %60 ön açıyı filtrele
    lidar_path = os.path.join(lidar_dir, f"frame_{frame_id:04d}.bin")
    if os.path.exists(lidar_path):
        lidar_points = load_lidar_bin(lidar_path)
        lidar_points = filter_lidar_fov(lidar_points, angle_deg=60)
    else:
        lidar_points = np.empty((0, 3))

    # Open3D point cloud oluştur
    pc_cam = o3d.geometry.PointCloud()
    pc_cam.points = o3d.utility.Vector3dVector(cam_points)

    pc_lidar = o3d.geometry.PointCloud()
    pc_lidar.points = o3d.utility.Vector3dVector(lidar_points)

    # ICP ile hizala
    if len(lidar_points) > 0:
        reg = o3d.pipelines.registration.registration_icp(
            pc_cam, pc_lidar, 1.0, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        aligned = pc_cam.transform(reg.transformation)
        combined_cloud += aligned + pc_lidar
    else:
        combined_cloud += pc_cam

    frame_id += 1

cap.release()

# Sonuç haritayı göster
o3d.visualization.draw_geometries([combined_cloud])
