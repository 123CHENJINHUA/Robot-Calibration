"""
Framework   : OpenCV Aruco
Description : Calibration of camera and using that for finding pose of multiple markers
Status      : Working
References  :
    1) https://docs.opencv.org/3.4.0/d5/dae/tutorial_aruco_detection.html
    2) https://docs.opencv.org/3.4.3/dc/dbb/tutorial_py_calibration.html
    3) https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import time
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
from pathlib import Path

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        
        # TCP pose data from robots
        self.tcp_pose1 = None
        self.tcp_pose2 = None
        
        # Create subscribers for robot TCP poses
        self.subscription1 = self.create_subscription(
            Float64MultiArray,
            'robot1/tcp_pose',
            self.tcp_pose1_callback,
            10)
        
        self.subscription2 = self.create_subscription(
            Float64MultiArray,
            'robot2/tcp_pose',
            self.tcp_pose2_callback,
            10)
        
        self.get_logger().info('Data Collection Node initialized')
    
    def tcp_pose1_callback(self, msg):
        """Callback for robot 1 TCP pose: [x, y, z, rx, ry, rz]"""
        self.tcp_pose1 = msg.data
    
    def tcp_pose2_callback(self, msg):
        """Callback for robot 2 TCP pose: [x, y, z, rx, ry, rz]"""
        self.tcp_pose2 = msg.data


def process_camera(node, camera_id1, camera_id2, mtx1, dist1, mtx2, dist2):

    robot_pos1 = []
    robot_pos2 = []
    R1_list = []
    T1_list = []
    R2_list = []
    T2_list = []
    R3_list = []
    T3_list = []

    cap1 = cv2.VideoCapture(camera_id1)
    cap2 = cv2.VideoCapture(camera_id2)
    width = 640
    height = 480
    fps = 30
    cap1.set(3, width)  #设置宽度
    cap1.set(4, height)  #设置长度
    cap1.set(5, fps)  
    cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    cap2.set(3, width)  #设置宽度
    cap2.set(4, height)  #设置长度
    cap2.set(5, fps)  
    cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    intrinsics = color_profile.get_intrinsics()
    mtx3 = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                    [0, intrinsics.fy, intrinsics.ppy],
                    [0, 0, 1]])
    dist3 = np.array(intrinsics.coeffs, dtype=np.float32)

    current_time = time.time()
    last_time = current_time

    count_data = 0
    ###------------------ ARUCO TRACKER ---------------------------
    while (True):

        current_time = time.time()

        # Spin once to process callbacks and get latest TCP poses
        rclpy.spin_once(node, timeout_sec=0.001)
        
        # Get TCP poses from ROS2 topics
        if node.tcp_pose1 is not None and node.tcp_pose2 is not None:
            ret_org1 = node.tcp_pose1
            ret_org2 = node.tcp_pose2
        else:
            node.get_logger().warn('Waiting for TCP pose data...')
            continue

        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        frames = pipeline.wait_for_frames()
        frame3 = frames.get_color_frame()

        # check read success explicitly (avoid truth-testing arrays)
        if not ret1 or not ret2 or frame3 is None:
            print("no frame")
            continue

        # Only copy/convert frames after verifying they're valid
        frame_copy1 = frame1.copy()
        frame_copy2 = frame2.copy()
        frame3 = np.asanyarray(frame3.get_data())
        frame_copy3 = frame3.copy()

        # operations on the frame
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        gray3 = cv2.cvtColor(frame3, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        board = cv2.aruco.CharucoBoard((3, 3), 0.066, 0.049, aruco_dict)
        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners1, ids1, rejectedImgPoints = aruco.detectMarkers(gray1, aruco_dict, parameters=parameters)
        corners2, ids2, rejectedImgPoints = aruco.detectMarkers(gray2, aruco_dict, parameters=parameters)
        corners3, ids3, rejectedImgPoints = aruco.detectMarkers(gray3, aruco_dict, parameters=parameters)

        # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

        # check if the ids lists are not None (and contain elements)
        # `detectMarkers` returns None for ids when no markers are found
        if ids1 is not None and ids2 is not None and ids3 is not None:

            # draw a square around the markers
            aruco.drawDetectedMarkers(frame_copy1, corners1)
            aruco.drawDetectedMarkers(frame_copy2, corners2)
            aruco.drawDetectedMarkers(frame_copy3, corners3)

            retval1, charucoCorners1, charucoIds1 = cv2.aruco.interpolateCornersCharuco(corners1, ids1, frame_copy1, board)
            retval2, charucoCorners2, charucoIds2 = cv2.aruco.interpolateCornersCharuco(corners2, ids2, frame_copy2, board)
            retval3, charucoCorners3, charucoIds3 = cv2.aruco.interpolateCornersCharuco(corners3, ids3, frame_copy3, board)
            if retval1 and retval2 and retval3:
                retval1, rvec_1, tvec_1 = cv2.aruco.estimatePoseCharucoBoard(charucoCorners1, charucoIds1, board, mtx1, dist1, None, None)
                retval2, rvec_2, tvec_2 = cv2.aruco.estimatePoseCharucoBoard(charucoCorners2, charucoIds2, board, mtx2, dist2, None, None)
                retval3, rvec_3, tvec_3 = cv2.aruco.estimatePoseCharucoBoard(charucoCorners3, charucoIds3, board, mtx3, dist3, None, None)

                # If pose estimation is successful, draw the axis
                if retval1 and retval2 and retval3:
                    cv2.drawFrameAxes(frame_copy1, mtx1, dist1, rvec_1, tvec_1, length=0.05, thickness=2)
                    cv2.drawFrameAxes(frame_copy2, mtx2, dist2, rvec_2, tvec_2, length=0.05, thickness=2)
                    cv2.drawFrameAxes(frame_copy3, mtx3, dist3, rvec_3, tvec_3, length=0.05, thickness=2)

                    if current_time - last_time > 0.2:
                        
                        last_time = current_time
                        count_data += 1
                        print(f"Data count: {count_data}")

                        R_mask2cam = np.zeros((3, 3), dtype=np.float64)
                        cv2.Rodrigues(rvec_1, R_mask2cam)
                        R1_list.append(R_mask2cam)
                        T1_list.append(tvec_1)
                        R_mask2cam = np.zeros((3, 3), dtype=np.float64)
                        cv2.Rodrigues(rvec_2, R_mask2cam)
                        R2_list.append(R_mask2cam)
                        T2_list.append(tvec_2)
                        R_mask2cam = np.zeros((3, 3), dtype=np.float64)
                        cv2.Rodrigues(rvec_3, R_mask2cam)
                        R3_list.append(R_mask2cam)
                        T3_list.append(tvec_3)

                        # If Aruco mark detected well, record the TCP data.
                        robot_pos1.append(ret_org1)
                        robot_pos2.append(ret_org2)

                else:
                    # code to show 'No Ids' when no markers are found
                    cv2.putText(frame_copy1, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(frame_copy2, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(frame_copy3, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # resize each frame to the same display size and merge horizontally
        disp_size = (320, 240)  # (width, height) for display
        frame_disp1 = cv2.resize(frame_copy1, disp_size)
        frame_disp2 = cv2.resize(frame_copy2, disp_size)
        frame_disp3 = cv2.resize(frame_copy3, disp_size)

        # Concatenate horizontally and show in one window
        merged = np.hstack((frame_disp1, frame_disp2, frame_disp3))
        cv2.imshow('merged', merged)

        k = cv2.waitKey(1)
        if k == 27:
            break

    # When everything done, release the capture
    
    file_path = Path(__file__).resolve()
    pkg_name = 'calibration_pkg'

    # Prefer saving under the SOURCE workspace: <ws>/src/calibration_pkg/calibration_pkg/hand_eye_calibration/data
    data_dir = None
    for p in file_path.parents:
        if p.name == 'install':
            ws_root = p.parent  # workspace root
            data_dir = ws_root / 'src' / pkg_name / pkg_name / 'hand_eye_calibration' / 'data'
            break

    # If not running from install, fall back to package root path
    if data_dir is None:
        data_dir = file_path.parent / 'data'

    if not data_dir.exists():
        data_dir.mkdir(parents=True, exist_ok=True)
    
    data_dir = str(data_dir)
    
    np.save(os.path.join(data_dir, 'R_list_1.npy'), R1_list)
    np.save(os.path.join(data_dir, 'T_list_1.npy'), T1_list)
    np.save(os.path.join(data_dir, 'R_list_2.npy'), R2_list)
    np.save(os.path.join(data_dir, 'T_list_2.npy'), T2_list)
    np.save(os.path.join(data_dir, 'R_list_3.npy'), R3_list)
    np.save(os.path.join(data_dir, 'T_list_3.npy'), T3_list)
    filename_R1 = open(os.path.join(data_dir, 'R_list_1.txt'), 'w')
    filename_T1 = open(os.path.join(data_dir, 'T_list_1.txt'), 'w')
    filename_R2 = open(os.path.join(data_dir, 'R_list_2.txt'), 'w')
    filename_T2 = open(os.path.join(data_dir, 'T_list_2.txt'), 'w')
    filename_R3 = open(os.path.join(data_dir, 'R_list_3.txt'), 'w')
    filename_T3 = open(os.path.join(data_dir, 'T_list_3.txt'), 'w')

    for value in R1_list:
        filename_R1.write(str(value))
        filename_R1.write('\n')
    for value in T1_list:
        filename_T1.write(str(value[0][0]))
        filename_T1.write(', ')
        filename_T1.write(str(value[1][0]))
        filename_T1.write(', ')
        filename_T1.write(str(value[2][0]))
        filename_T1.write('\n')

    for value in R2_list:
        filename_R2.write(str(value))
        filename_R2.write('\n')
    for value in T2_list:
        filename_T2.write(str(value[0][0]))
        filename_T2.write(', ')
        filename_T2.write(str(value[1][0]))
        filename_T2.write(', ')
        filename_T2.write(str(value[2][0]))
        filename_T2.write('\n')

    for value in R3_list:
        filename_R3.write(str(value))
        filename_R3.write('\n')
    for value in T3_list:
        filename_T3.write(str(value[0][0]))
        filename_T3.write(', ')
        filename_T3.write(str(value[1][0]))
        filename_T3.write(', ')
        filename_T3.write(str(value[2][0]))
        filename_T3.write('\n')



    np.save(os.path.join(data_dir, 'Robot_data_1.npy'), robot_pos1)
    np.save(os.path.join(data_dir, 'Robot_data_2.npy'), robot_pos2)
    filename1 = open(os.path.join(data_dir, 'Robot_data_1.txt'), 'w')
    filename2 = open(os.path.join(data_dir, 'Robot_data_2.txt'), 'w')

    for value in robot_pos1:
        filename1.write(str(value[0]))
        filename1.write(', ')
        filename1.write(str(value[1]))
        filename1.write(', ')
        filename1.write(str(value[2]))
        filename1.write(', ')
        filename1.write(str(value[3]))
        filename1.write(', ')
        filename1.write(str(value[4]))
        filename1.write(', ')
        filename1.write(str(value[5]))
        filename1.write('\n')

    for value in robot_pos2:
        filename2.write(str(value[0]))
        filename2.write(', ')
        filename2.write(str(value[1]))
        filename2.write(', ')
        filename2.write(str(value[2]))
        filename2.write(', ')
        filename2.write(str(value[3]))
        filename2.write(', ')
        filename2.write(str(value[4]))
        filename2.write(', ')
        filename2.write(str(value[5]))
        filename2.write('\n')

    cv2.destroyAllWindows()
    pipeline.stop()


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the ROS2 node
    node = DataCollectionNode()
    
    # Get the script directory for proper file paths
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    camera_calib_dir = os.path.join(os.path.dirname(script_dir), 'camera_calibration')
    
    # Load mtx and dist
    cv_file1 = cv2.FileStorage(os.path.join(camera_calib_dir, "charuco_camera_calibration1.yaml"), cv2.FILE_STORAGE_READ)
    mtx1 = cv_file1.getNode("camera_matrix").mat()
    dist1 = cv_file1.getNode("dist_coeff").mat()

    cv_file2 = cv2.FileStorage(os.path.join(camera_calib_dir, "charuco_camera_calibration2.yaml"), cv2.FILE_STORAGE_READ)
    mtx2 = cv_file2.getNode("camera_matrix").mat()
    dist2 = cv_file2.getNode("dist_coeff").mat()

    cv_file3 = cv2.FileStorage(os.path.join(camera_calib_dir, "charuco_camera_calibration_realsense.yaml"), cv2.FILE_STORAGE_READ)
    mtx3 = cv_file3.getNode("camera_matrix").mat()
    dist3 = cv_file3.getNode("dist_coeff").mat()

    camera1 = 8
    camera2 = 6

    try:
        process_camera(node, camera1, camera2, mtx1, dist1, mtx2, dist2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()