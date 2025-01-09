from realsense_server import Normal_test
from fairino import Robot
import cv2
import time

tool = 1 #工具坐标系编号
user = 0 #工件坐标系编号
offset_flag = 2
offset_pos = [0,-7,-10,0,0,0]


robot = Robot.RPC('192.168.58.2')
camera_ymal_path = "./camera_calibration/charuco_camera_calibration_realsense.yaml"
RT_cam_to_end_path = './hand_eye_calibration/result/RT_4.npy'

init_pose = [46.0,-656.0, 487.0, -95.4, 3.9, 175.0]

ret = robot.MoveL(init_pose, tool, user, vel=20,offset_flag=offset_flag, offset_pos=offset_pos)

time.sleep(1)
normal_test = Normal_test(robot,camera_ymal_path,RT_cam_to_end_path)
result, color_image, depth_image = normal_test.main()
print(result)
cv2.imshow("ROI", color_image)
# cv2.imshow("Depth", depth_image)
cv2.waitKey(1)

ret = robot.MoveL(result, tool, user, vel=20,offset_flag=offset_flag, offset_pos=offset_pos)
print(ret)