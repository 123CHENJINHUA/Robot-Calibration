import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys
import open3d as o3d
from math import *

from camera import Camera


def select_roi(image):
    roi = cv2.selectROI("Select ROI", image, fromCenter=False, showCrosshair=True)
    cv2.destroyWindow("Select ROI")
    return roi

def draw_roi(image, roi, normal_vector=None):
    x, y, w, h = roi
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    if normal_vector is not None:
        center = (x + w // 2, y + h // 2)
        # 绘制X轴箭头（红色）
        end_point_x = (int(center[0] + 50), center[1])
        cv2.arrowedLine(image, center, end_point_x, (0, 0, 255), 2)
        # 绘制Y轴箭头（绿色）
        end_point_y = (center[0], int(center[1] - 50))
        cv2.arrowedLine(image, center, end_point_y, (0, 255, 0), 2)
        # 绘制Z轴箭头（蓝色，表示法向量）
        end_point_z = (int(center[0] + normal_vector[0] * 50), int(center[1] - normal_vector[1] * 50))
        cv2.arrowedLine(image, center, end_point_z, (255, 0, 0), 2)
    return image
    

def calculate_normal_vector(depth_image, roi, downsample_factor=3):
    x, y, w, h = roi
    points = []
    for i in range(y, y + h, downsample_factor):
        for j in range(x, x + w, downsample_factor):
            z = depth_image[i, j]
            if z > 0:
                points.append([j, i, z])
    points = np.array(points, dtype=np.float64)
    centroid = np.mean(points, axis=0)
    points -= centroid
    u, s, vh = np.linalg.svd(points)
    normal_vector = vh[2, :]
    return normal_vector

def main():
    roi = None
    camera = Camera()
    time.sleep(1)
    while True:
        color_frame, aligned_depth_frame = camera.get_frames()
        intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        color_image, depth_image, vertices, vertices_color = camera.process_frames(color_frame, aligned_depth_frame)
        if roi is None:
            roi = select_roi(color_image)
        normal_vector = calculate_normal_vector(depth_image, roi)
        image = draw_roi(color_image, roi, normal_vector)
        print("Normal Vector:", normal_vector)
        cv2.imshow("ROI", image)
        cv2.imshow("Depth", depth_image)
        if cv2.waitKey(100) & 0xFF == 27:  # Press 'ESC' to exit the loop
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()