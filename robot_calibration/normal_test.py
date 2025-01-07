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

# def draw_roi2(image, roi, normal_vector=None):
#     x, y, w, h = roi
#     cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#     if normal_vector is not None:
#         center = (x + w // 2, y + h // 2)
#         # 绘制X轴箭头（红色）
#         end_point_x = (int(center[0] + 50), center[1])
#         cv2.arrowedLine(image, center, end_point_x, (0, 0, 255), 2)
#         # 绘制Y轴箭头（绿色）
#         end_point_y = (center[0], int(center[1] - 50))
#         cv2.arrowedLine(image, center, end_point_y, (0, 255, 0), 2)
#         # 绘制Z轴箭头（蓝色，表示法向量）
#         end_point_z = (int(center[0] + normal_vector[0] * 50), int(center[1] - normal_vector[1] * 50))
#         cv2.arrowedLine(image, center, end_point_z, (255, 0, 0), 2)
#     return image

# def calculate_normal_vector2(depth_image, roi, downsample_factor=3):
#     x, y, w, h = roi
#     points = []
#     for i in range(y, y + h, downsample_factor):
#         for j in range(x, x + w, downsample_factor):
#             z = depth_image[i, j]
#             if z > 0:
#                 points.append([j, i, z])
#     points = np.array(points, dtype=np.float64)
#     centroid = np.mean(points, axis=0)
#     points -= centroid
#     u, s, vh = np.linalg.svd(points)
#     normal_vector = vh[2, :] #PCA
#     return normal_vector

def draw_roi(image, roi, mtx, dist, surface_normal=None ,mask_3Dcoord=None):
    x, y, w, h = roi
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    tvec = np.array([mask_3Dcoord[0],mask_3Dcoord[1],mask_3Dcoord[2]])
    rvec = np.array([mask_3Dcoord[3],mask_3Dcoord[4],mask_3Dcoord[5]])
    cv2.drawFrameAxes(image,mtx,dist,rvec,tvec,length=0.05,thickness=2)
    image_point,_ = cv2.projectPoints(np.array([[0,0,0]],dtype = np.float64),rvec,tvec,mtx,dist)
    image_x = int(image_point[0][0][0])
    image_y = int(image_point[0][0][1])

    image = cv2.circle(image, (image_x, image_y), radius=10, color=(255, 255, 255), thickness=1)
    text = "["+str(round(surface_normal[0],4))+" "+str(round(surface_normal[1],4))+" "+str(round(surface_normal[2],4))+"]"
    cv2.putText(image, text, (image_x+25, image_y+25), cv2.FONT_HERSHEY_PLAIN,
            1.0, (0, 0, 255), thickness=1)
    return image
    



def calculate_normal_vector(camera,roi,color_image, aligned_depth_frame,intrinsics,rtz):
    #compute surface normal at mask
    x, y, w, h = roi
    area = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]])
    
    # Compute surface normal at mask
    mask = np.zeros(color_image.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [area], 255)
    vert_mask, color_mask = mask_vertices_colors(camera, mask, color_image, aligned_depth_frame,intrinsics)
    surface_normal = get_surface_normal(vert_mask)
    surface_normal, normal2RotVec = rodrigues_rotation(surface_normal,rtz)
    # normal2RotM,_ = cv2.Rodrigues(normal2RotVec)
    # print('next:\n')
    # print(normal2RotM)

    mask_xcoord, mask_ycoord = get_mask_center(mask)
    mask_3Dcoord = camera.deproject_pixel([mask_xcoord,mask_ycoord],aligned_depth_frame,intrinsics)
    # if np.sum(mask_3Dcoord) != 0:
    mask_3Dcoord.extend(normal2RotVec[0])
    mask_3Dcoord.extend(normal2RotVec[1])
    mask_3Dcoord.extend(normal2RotVec[2])
    return surface_normal, mask_3Dcoord


def get_mask_center(mask):
    "return pixel coordinates of mask center"
    mask_center = np.mean(np.argwhere(mask),axis=0)

    return int(mask_center[1]), int(mask_center[0])

def mask_vertices_colors(camera, mask, color_image, aligned_depth_frame,intrinsics):
    idxes = np.argwhere(mask)
    points = []
    colors = []
    for row in idxes:
        coords = camera.deproject_pixel([row[1],row[0]],aligned_depth_frame,intrinsics)
        points.append(coords)
        colors.append(color_image[row[0],row[1], :])

    return np.asarray(points).astype(np.float32), np.asarray(colors).astype(np.uint8)

def get_surface_normal(mask):

    pcd = o3d.geometry.PointCloud()
    mask = mask.astype(np.float64)
    pcd.points = o3d.utility.Vector3dVector(mask[~np.any(mask == 0, axis =1)])
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    for i in range(np.asarray(pcd.normals).shape[0]):
        if pcd.normals[i][2] > 0:
            pcd.normals[i][0] = -pcd.normals[i][0]
            pcd.normals[i][1] = -pcd.normals[i][1]
            pcd.normals[i][2] = -pcd.normals[i][2]
    
    normals = np.asarray(pcd.normals)
    return np.sum(normals, axis=0) / normals.shape[0]

def rodrigues_rotation(surface_normal,rtz):

    surface_normal = surface_normal/np.linalg.norm(surface_normal)

    # surface_normal = np.array([-0.3,-0.3,-1])
    print("surface_normal",surface_normal)
    print("norm",np.linalg.norm(surface_normal))
    v1 = np.array([0,0,1])
    v2 = -surface_normal
    # R_cam_to_base = np.dot(R_all_end_to_base_1,RT_cam_to_end[:3,:3])
    # v2 = np.dot(np.linalg.inv(R_cam_to_base),np.array([0,0,-1]))


    rotationAxis = np.cross(v1,v2)
    rotationAxis = rotationAxis/np.sqrt(np.sum(rotationAxis*rotationAxis))
    rotationAngle = np.arccos(np.dot(v1,v2)/(np.sqrt(np.sum(v1*v1))*np.sqrt(np.sum(v2*v2))))

    rotationVector = rotationAxis*rotationAngle


    theta = rotationAngle
    r = np.array(rotationAxis).reshape(3,1)

    rx,ry,rz = r[:,0]
    M = np.array([
        [0,-rz,ry],
        [rz,0,-rx],
        [-ry,rx,0]
    ])
    R = np.eye(3)
    R[:3,:3] = np.cos(theta)*np.eye(3)+(1-np.cos(theta)) * r @ r.T + np.sin(theta) * M
    z=-rtz*np.pi/180
    print("z",rtz)
    Rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    # R = np.dot(Rz,R)
    R = np.dot(R,Rz)
    

    rotationVector,_ = cv2.Rodrigues(R)
    # print('First')
    # print(R)
    return surface_normal, rotationVector



if __name__ == "__main__":
    roi = None
    camera = Camera()
    time.sleep(1)

    cv_file3 = cv2.FileStorage("./camera_calibration/charuco_camera_calibration_realsense.yaml", cv2.FILE_STORAGE_READ)
    mtx= cv_file3.getNode("camera_matrix").mat()
    dist = cv_file3.getNode("dist_coeff").mat()

    while True:
        color_frame, aligned_depth_frame = camera.get_frames()
        intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        color_image, depth_image, vertices, vertices_color = camera.process_frames(color_frame, aligned_depth_frame)
        if roi is None:
            roi = select_roi(color_image)
        surface_normal, mask_3Dcoord = calculate_normal_vector(camera,roi,color_image, aligned_depth_frame,intrinsics,rtz=135)
        color_image = draw_roi(color_image, roi, mtx, dist, surface_normal, mask_3Dcoord)
        print("Normal Vector:", surface_normal)
        cv2.imshow("ROI", color_image)
        cv2.imshow("Depth", depth_image)
        if cv2.waitKey(1) & 0xFF == 27:  # Press 'ESC' to exit the loop
            break
    cv2.destroyAllWindows()