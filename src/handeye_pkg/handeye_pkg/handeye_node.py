#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import cv2
import cv2.aruco as aruco
import numpy as np
import time

class ArucoHandEyeCalibration(Node):
    def __init__(self):
        super().__init__('aruco_hand_eye_calibration')
        
        # 初始化参数
        self.bridge = CvBridge()
        self.R_list = []  # 相机到标记的旋转矩阵列表
        self.T_list = []  # 相机到标记的平移向量列表
        self.robot_pose_list = []  # 机器人位姿列表（x,y,z,rx,ry,rz）
        
        # 存储最新的机器人位姿
        self.latest_robot_pose = None
        self.pose_data = None
        
        # Aruco参数
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.board = cv2.aruco.CharucoBoard((3, 3), 0.095, 0.071, self.aruco_dict)
        self.parameters = aruco.DetectorParameters()
        self.parameters.adaptiveThreshConstant = 10
        
        # 相机内参和畸变系数（将从camera_info中获取）
        self.mtx = None
        self.dist = None
        self.camera_info_received = False

        
        # 订阅彩色图像
        self.color_sub = self.create_subscription(
            Image,
            '/camera/realsense2_camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # 订阅相机信息
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/realsense2_camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.robot_sub = self.create_subscription(
            TransformStamped,
            '/robot1_transform_rpy',
            self.robot_callback,
            10
        )

        # 数据收集标志
        self.collecting_data = False
        self.data_count = 0
        self.max_data_points = 3000  # 最大收集数据点数
        
        self.get_logger().info('Aruco手眼标定节点已启动')
        self.get_logger().info('等待相机内参和TF数据...')
        
        # 数据保存相关
        self.save_counter = 0

    def robot_callback(self, msg: TransformStamped):
        trans = msg.transform.translation
        rot = msg.transform.rotation

        trans.x = trans.x * 1000.0
        trans.y = trans.y * 1000.0
        trans.z = trans.z * 1000.0

        # 直接提取，假设rot.x, rot.y, rot.z已经是欧拉角（度）
        self.pose_data = [
            trans.x, trans.y, trans.z,
            rot.x, rot.y, rot.z  # 欧拉角（度）
        ]
        
    
    def camera_info_callback(self, msg: CameraInfo):
        self.mtx = np.array(msg.k).reshape(3, 3)
        self.dist = np.array(msg.d)
        # self.get_logger().info('已获取相机内参')
        # self.get_logger().info(f'相机内参矩阵:\n{self.mtx}')
        # self.get_logger().info(f'畸变系数: {self.dist}')
        

    # def image_callback(self, color_msg):
    #     color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
    #     cv2.imshow('Aruco Detection', color_image)
    #     cv2.waitKey(1) 
    
    def image_callback(self, color_msg):
        """处理同步的图像和相机信息"""
        try:
            self.latest_robot_pose = self.pose_data
            # 将ROS图像转换为OpenCV格式
            color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            frame_copy = color_image.copy()
            
            # 转换为灰度图
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            
            # 检测ArUco标记
            corners, ids, rejectedImgPoints = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.parameters
            )
            
            
            # 如果检测到标记
            if ids is not None and len(ids) > 0:
                # 绘制检测到的标记
                aruco.drawDetectedMarkers(frame_copy, corners)
                
                # 插值Charuco角点
                retval, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(
                    corners, ids, color_image, self.board
                )
                
                if retval and len(charucoIds) >= 4:  # 至少需要4个角点
                    # 估计姿态
                    retval, rvec_, tvec_ = cv2.aruco.estimatePoseCharucoBoard(
                        charucoCorners, charucoIds, self.board, 
                        self.mtx, self.dist, None, None
                    )
                    
                    if retval:
                        # 绘制坐标轴
                        cv2.drawFrameAxes(frame_copy, self.mtx, self.dist, 
                                         rvec_, tvec_, length=0.05, thickness=2)
                        
                        # 在图像上显示检测状态
                        cv2.putText(frame_copy, "ArUco Detected", (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        # 如果有有效的机器人位姿，保存数据
                       
                        if self.latest_robot_pose is not None:
                            
                            # 将旋转向量转换为旋转矩阵
                            R_mask2cam, _ = cv2.Rodrigues(rvec_)
                            
                            # 保存数据
                            self.R_list.append(R_mask2cam)
                            self.T_list.append(tvec_)
                            self.robot_pose_list.append(self.latest_robot_pose)
                            
                            self.data_count += 1
                            
                            # 显示当前数据收集状态
                            status_text = f'Data Points: {self.data_count}/{self.max_data_points}'
                            cv2.putText(frame_copy, status_text, (10, 60), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            
                            # 显示当前机器人位姿
                            pose_text = f'Robot: [{self.latest_robot_pose[0]:.3f}, {self.latest_robot_pose[1]:.3f}, {self.latest_robot_pose[2]:.3f}]'
                            cv2.putText(frame_copy, pose_text, (10, 90), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                            
                            if self.data_count % 10 == 0:
                                self.get_logger().info(
                                    f'数据点 #{self.data_count}: '
                                    f'机器人: [{self.latest_robot_pose[0]:.3f}, {self.latest_robot_pose[1]:.3f}, {self.latest_robot_pose[2]:.3f}]'
                                )
                            
                            # 达到最大数据点数时自动保存
                            if self.data_count >= self.max_data_points and not self.collecting_data:
                                self.collecting_data = True
                                self.save_data()
                                self.get_logger().info(f'已收集{self.data_count}个数据点，自动保存并退出')
                                # 可以选择退出或继续收集
                                rclpy.shutdown()
                    
                    else:
                        cv2.putText(frame_copy, "Pose Estimation Failed", (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(frame_copy, "No ArUco Markers", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 显示数据收集状态
            if self.latest_robot_pose is None:
                cv2.putText(frame_copy, "Waiting for Robot TF...", (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            
            # 显示图像
            cv2.imshow('Aruco Detection', frame_copy)
            
            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):  # 按's'键保存当前数据
                self.save_data()
            elif key == ord('q') or key == 27:  # 按'q'或ESC键退出
                self.get_logger().info('用户请求退出')
                self.save_data()
                cv2.destroyAllWindows()
                rclpy.shutdown()
            elif key == ord('c'):  # 按'c'键清除数据
                self.clear_data()
                
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}', exc_info=True)
    
    def clear_data(self):
        """清除已收集的数据"""
        self.R_list.clear()
        self.T_list.clear()
        self.robot_pose_list.clear()
        self.data_count = 0
        self.get_logger().info('已清除所有数据')
    
    def save_data(self):
        """保存数据到文件"""
        if self.data_count == 0:
            self.get_logger().warn('没有数据可保存')
            return
        
        try:
 
            
            self.get_logger().info(f'开始保存数据，共{self.data_count}个点...')
            
            np.save(f'./R_list.npy', self.R_list)
            np.save(f'./T_list.npy', self.T_list)
            np.save(f'./Robot_data.npy', self.robot_pose_list)

            with open(f'./R_list.txt', 'w') as filename_R:
                for value in self.R_list:
                    filename_R.write(str(value))
                    filename_R.write('\n')
            
            with open(f'./T_list.txt', 'w') as filename_T:
                for value in self.T_list:
                    filename_T.write(f"{value[0][0]}, {value[1][0]}, {value[2][0]}\n")

            with open(f'./Robot_data.txt', 'w') as filename:
                for value in self.robot_pose_list:
                    filename.write(f"{value[0]}, {value[1]}, {value[2]}, {value[3]}, {value[4]}, {value[5]}\n")

            self.get_logger().info(f'所有数据已保存')
            
        except Exception as e:
            self.get_logger().error(f'保存数据失败: {e}', exc_info=True)
    
    def destroy_node(self):
        """节点销毁时保存数据"""
        if self.data_count > 0:
            self.get_logger().info('节点关闭，自动保存数据...')
            # self.save_data()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建节点
    node = ArucoHandEyeCalibration()

    try:
        # 运行节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()