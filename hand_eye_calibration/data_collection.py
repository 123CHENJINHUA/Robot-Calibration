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
from fairino import Robot
from Joy import joy
import time
import multiprocessing
import pyrealsense2 as rs

def process_camera(camera_id, robot, mtx, dist):
   

    robot_pos = []
    R_list = []
    T_list = []

    cap = cv2.VideoCapture(camera_id)
    width = 1920
    height = 1080
    fps = 60
    cap.set(3, width)  #设置宽度
    cap.set(4, height)  #设置长度
    cap.set(5, fps)  
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    

    ###------------------ ARUCO TRACKER ---------------------------
    while (True):

        ret2, frame = cap.read()
        ret_org = robot.GetActualTCPPose()
        print(ret_org[1])

        frame_copy = frame.copy()
        
        #if ret returns false, there is likely a problem with the webcam/camera.
        #In that case uncomment the below line, which will replace the empty frame 
        #with a test image used in the opencv docs for aruco at https://www.docs.opencv.org/4.5.3/singlemarkersoriginal.jpg
        # frame = cv2.imread('./images/test image.jpg') 

        # operations on the frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        board = cv2.aruco.CharucoBoard((3, 3), 0.041, 0.030, aruco_dict)
        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

        # check if the ids list is not empty
        # if no check is added the code will crash
        if np.all(ids != None):

            # draw a square around the markers
            aruco.drawDetectedMarkers(frame_copy, corners)
            retval, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(corners, ids, frame, board)
            if retval:
                retval, rvec_, tvec_ = cv2.aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, mtx, dist, None, None)

                # If pose estimation is successful, draw the axis
                if retval:
                    cv2.drawFrameAxes(frame_copy, mtx, dist, rvec_, tvec_, length=0.05, thickness=2)

                    R_mask2cam = np.zeros((3, 3), dtype=np.float64)
                    cv2.Rodrigues(rvec_, R_mask2cam)
                    R_list.append(R_mask2cam)
                    T_list.append(tvec_)

                    # If Aruco mark detected well, record the TCP data.
                    ret = ret_org
                    robot_pos.append(ret[1])

                else:
                    # code to show 'No Ids' when no markers are found
                    cv2.putText(frame_copy, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # display the resulting frame
        frame_copy = cv2.resize(frame_copy,(frame_copy.shape[1]//2, frame_copy.shape[0]//2))
        cv2.imshow('frame_'+str(camera_id), frame_copy)
        k = cv2.waitKey(1)
        if k == 27:
            break

    # When everything done, release the capture
    np.save(f'./hand_eye_calibration/data/R_list_{camera_id}.npy', R_list)
    np.save(f'./hand_eye_calibration/data/T_list_{camera_id}.npy', T_list)
    filename_R = open(f'./hand_eye_calibration/data/R_list_{camera_id}.txt', 'w')
    filename_T = open(f'./hand_eye_calibration/data/T_list_{camera_id}.txt', 'w')

    for value in R_list:
        filename_R.write(str(value))
        filename_R.write('\n')
    for value in T_list:
        filename_T.write(str(value[0][0]))
        filename_T.write(', ')
        filename_T.write(str(value[1][0]))
        filename_T.write(', ')
        filename_T.write(str(value[2][0]))
        filename_T.write('\n')

    np.save(f'./hand_eye_calibration/data/Robot_data_{camera_id}.npy', robot_pos)
    filename = open(f'./hand_eye_calibration/data/Robot_data_{camera_id}.txt', 'w')
    for value in robot_pos:
        filename.write(str(value[0]))
        filename.write(', ')
        filename.write(str(value[1]))
        filename.write(', ')
        filename.write(str(value[2]))
        filename.write(', ')
        filename.write(str(value[3]))
        filename.write(', ')
        filename.write(str(value[4]))
        filename.write(', ')
        filename.write(str(value[5]))
        filename.write('\n')

    cv2.destroyAllWindows()

def process_robot_control(robot,control):
    n_pos = [0.0,0.0,0.0,0.0,0.0,0.0]
    
    error = robot.ServoMoveStart()  #伺服运动开始
    print("伺服运动开始错误码",error)
    while(n_pos):
        n_pos = control.getmotion()
        #print(n_pos)
        if n_pos == None:
            break
        error = robot.ServoCart(1, n_pos, vel=40)   #笛卡尔空间伺服模式运动
        if error!=0:
            error_cart =error
        time.sleep(0.008)
    print("笛卡尔空间伺服模式运动错误码", error_cart)
    error = robot.ServoMoveEnd()  #伺服运动开始
    print("伺服运动结束错误码",error)


def realsense_camera(camera_id,robot, mtx, dist):

    robot_pos = []
    R_list = []
    T_list = []

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while True:

            ret_org = robot.GetActualTCPPose()
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            frame_copy = color_image.copy()
            # operations on the frame
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # set dictionary size depending on the aruco marker selected
            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
            board = cv2.aruco.CharucoBoard((3, 3), 0.041, 0.030, aruco_dict)
            # detector parameters can be set here (List of detection parameters[3])
            parameters = aruco.DetectorParameters()
            parameters.adaptiveThreshConstant = 10

            # lists of ids and the corners belonging to each id
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # font for displaying text (below)
            font = cv2.FONT_HERSHEY_SIMPLEX

            # check if the ids list is not empty
            # if no check is added the code will crash
            if np.all(ids != None):

                # draw a square around the markers
                aruco.drawDetectedMarkers(frame_copy, corners)
                retval, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(corners, ids, color_image, board)
                if retval:
                    retval, rvec_, tvec_ = cv2.aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, mtx, dist, None, None)

                    # If pose estimation is successful, draw the axis
                    if retval:

                        cv2.drawFrameAxes(frame_copy, mtx, dist, rvec_, tvec_, length=0.05, thickness=2)

                        R_mask2cam = np.zeros((3, 3), dtype=np.float64)
                        cv2.Rodrigues(rvec_, R_mask2cam)
                        R_list.append(R_mask2cam)
                        T_list.append(tvec_)

                        # If Aruco mark detected well, record the TCP data.
                        ret = ret_org
                        robot_pos.append(ret[1])

                    else:
                        # code to show 'No Ids' when no markers are found
                        cv2.putText(frame_copy, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # display the resulting frame
            frame_copy = cv2.resize(frame_copy,(frame_copy.shape[1]//2, frame_copy.shape[0]//2))
            depth_colormap = cv2.resize(depth_colormap,(depth_colormap.shape[1]//2, depth_colormap.shape[0]//2))
            # Stack both images horizontally
            images = np.hstack((frame_copy, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            k = cv2.waitKey(1)
            if k == 27:
                break

        # When everything done, release the capture
        np.save(f'./hand_eye_calibration/data/R_list_{camera_id}.npy', R_list)
        np.save(f'./hand_eye_calibration/data/T_list_{camera_id}.npy', T_list)
        filename_R = open(f'./hand_eye_calibration/data/R_list_{camera_id}.txt', 'w')
        filename_T = open(f'./hand_eye_calibration/data/T_list_{camera_id}.txt', 'w')

        for value in R_list:
            filename_R.write(str(value))
            filename_R.write('\n')
        for value in T_list:
            filename_T.write(str(value[0][0]))
            filename_T.write(', ')
            filename_T.write(str(value[1][0]))
            filename_T.write(', ')
            filename_T.write(str(value[2][0]))
            filename_T.write('\n')

        np.save(f'./hand_eye_calibration/data/Robot_data_{camera_id}.npy', robot_pos)
        filename = open(f'./hand_eye_calibration/data/Robot_data_{camera_id}.txt', 'w')
        for value in robot_pos:
            filename.write(str(value[0]))
            filename.write(', ')
            filename.write(str(value[1]))
            filename.write(', ')
            filename.write(str(value[2]))
            filename.write(', ')
            filename.write(str(value[3]))
            filename.write(', ')
            filename.write(str(value[4]))
            filename.write(', ')
            filename.write(str(value[5]))
            filename.write('\n')

        cv2.destroyAllWindows()
        # Stop streaming
        pipeline.stop()

    finally:

        # Stop streaming
        print("finish!")

if __name__ == "__main__":
    # Load mtx and dist
    cv_file1 = cv2.FileStorage("./camera_calibration/charuco_camera_calibration1.yaml", cv2.FILE_STORAGE_READ)
    mtx1 = cv_file1.getNode("camera_matrix").mat()
    dist1 = cv_file1.getNode("dist_coeff").mat()

    cv_file2 = cv2.FileStorage("./camera_calibration/charuco_camera_calibration2.yaml", cv2.FILE_STORAGE_READ)
    mtx2 = cv_file2.getNode("camera_matrix").mat()
    dist2 = cv_file2.getNode("dist_coeff").mat()

    cv_file3 = cv2.FileStorage("./camera_calibration/charuco_camera_calibration_realsense.yaml", cv2.FILE_STORAGE_READ)
    mtx3 = cv_file3.getNode("camera_matrix").mat()
    dist3 = cv_file3.getNode("dist_coeff").mat()


     # Connect to robot
    robot = Robot.RPC('192.168.58.2')
    control = joy.XboxController()

    ret = robot.DragTeachSwitch(0)

    # ret = robot.RobotEnable(1)
    # time.sleep(2)

    # Change robot's mode to dragteach 
    # ret = robot.Mode(1) #机器人切入手动模式
    # print("机器人切入手动模式", ret)
    # ret = robot.DragTeachSwitch(0)
    # time.sleep(1)
    # ret = robot.DragTeachSwitch(1)  #机器人切入拖动示教模式，必须在手动模式下才能切入拖动示教模式
    # print("机器人切入拖动示教模式", ret)
    # time.sleep(1)
    # ret,state = robot.IsInDragTeach()    #查询是否处于拖动示教模式，1-拖动示教模式，0-非拖动示教模式
    # if ret == 0:
    #     print("当前拖动示教模式状态：", state)
    # else:
    #     print("查询失败，错误码为：",ret)

    # Create processes for each camera

    processes = []

    p1 = multiprocessing.Process(target=process_robot_control, args=(robot,control))
    # p2 = multiprocessing.Process(target=process_camera, args=(0, robot, mtx1, dist1))
    # p3 = multiprocessing.Process(target=process_camera, args=(2, robot, mtx2, dist2))
    p4 = multiprocessing.Process(target=realsense_camera, args=(4, robot, mtx3, dist3))
    processes.append(p1)
    # processes.append(p2)
    # processes.append(p3)
    processes.append(p4)

    p1.start()
    #time.sleep(2)
    # p2.start()
    # p3.start()
    p4.start()


    # Wait for all processes to finish
    for p in processes:
        p.join()