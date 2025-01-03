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
from math import *

def myRPY2R_robot(x, y, z):
    x=x*np.pi/180
    y=y*np.pi/180
    z=z*np.pi/180

    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz@Ry@Rx
    return R

#用于根据位姿计算变换矩阵
def pose_robot(x, y, z, Tx, Ty, Tz):
    thetaX = x / 180 * pi
    thetaY = y / 180 * pi
    thetaZ = z / 180 * pi
    R = myRPY2R_robot(thetaX, thetaY, thetaZ)
    t = np.array([[Tx], [Ty], [Tz]])
    RT1 = np.column_stack([R, t])  # 列合并
    RT1 = np.vstack((RT1, np.array([0,0,0,1])))
    # RT1=np.linalg.inv(RT1)
    return RT1

def process_camera(camera_id, robot, mtx, dist):
   

    CAM_RT = np.load('./hand_eye_calibration/result/RT_.'+str(camera_id)+'.npy')

    cap = cv2.VideoCapture(camera_id)
    width = 1920
    height = 1080
    fps = 60
    cap.set(3, width)  #设置宽度
    cap.set(4, height)  #设置长度
    cap.set(5, fps)  
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    result = []

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
                    org_point = np.array([[0,0,0]], dtype=np.float64)
                    p1, _ = cv2.projectPoints(org_point, rvec_, tvec_, mtx, dist)
                    p1_x = int(p1[0][0][0])
                    p1_y = int(p1[0][0][1])
                
                    if (p1_x > 1280 or p1_y > 1280):
                        continue
                    cv2.circle(frame_copy, (p1_x, p1_y), 10, (0, 0, 255))

                    cv2.drawFrameAxes(frame_copy, mtx, dist, rvec_, tvec_, length=0.05, thickness=2)

                    ret = np.array(ret_org[1])
                    R_all_end_to_base_1 = myRPY2R_robot(ret[3],ret[4],ret[5])
                    T_all_end_to_base_1 = ret[:3].reshape(3, 1)
                
                    R_mask2cam = np.zeros((3, 3), dtype=np.float64)
                    cv2.Rodrigues(rvec_, R_mask2cam)
                    T_all_chess_to_cam_1 = 1000*tvec_[:3,0].reshape(3, 1)
                    R_all_chess_to_cam_1 = R_mask2cam

                    RT_end_to_base=np.column_stack((R_all_end_to_base_1,T_all_end_to_base_1))
                    RT_end_to_base=np.vstack((RT_end_to_base,np.array([0,0,0,1])))

                    RT_chess_to_cam=np.column_stack((R_all_chess_to_cam_1,T_all_chess_to_cam_1))
                    RT_chess_to_cam=np.vstack((RT_chess_to_cam,np.array([0,0,0,1])))

                    RT_cam_to_end=CAM_RT

                    RT_chess_to_base=RT_end_to_base@RT_cam_to_end@RT_chess_to_cam

                    # If Aruco mark detected well, record the TCP data.
                    
                    result.append(RT_chess_to_base[:3,:])

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
    filename = open('./robot_calibration/result1_'+str(camera_id)+'.txt','w')
    for value in result:
        filename.write(str(value))
        filename.write('\n\n')

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


if __name__ == "__main__":
    # Load mtx and dist
    cv_file1 = cv2.FileStorage("./camera_calibration/charuco_camera_calibration1.yaml", cv2.FILE_STORAGE_READ)
    mtx1 = cv_file1.getNode("camera_matrix").mat()
    dist1 = cv_file1.getNode("dist_coeff").mat()

    cv_file2 = cv2.FileStorage("./camera_calibration/charuco_camera_calibration2.yaml", cv2.FILE_STORAGE_READ)
    mtx2 = cv_file2.getNode("camera_matrix").mat()
    dist2 = cv_file2.getNode("dist_coeff").mat()

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
    p2 = multiprocessing.Process(target=process_camera, args=(0, robot, mtx1, dist1))
    #p3 = multiprocessing.Process(target=process_camera, args=(2, robot, mtx2, dist2))
    processes.append(p1)
    processes.append(p2)
    #processes.append(p3)

    p1.start()
    time.sleep(1)
    p2.start()
    #p3.start()


    # Wait for all processes to finish
    for p in processes:
        p.join()