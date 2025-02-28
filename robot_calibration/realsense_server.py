from fairino import Robot
from math import *
import time
import numpy as np

from camera import Camera
from normal_test import *

class Normal_test:
    def __init__(self,robot,camera_ymal_path,RT_cam_to_end_path):
        self.camera = Camera()
        self.roi = None
        self.robot = robot
        time.sleep(2)

        cv_file3 = cv2.FileStorage(camera_ymal_path, cv2.FILE_STORAGE_READ)
        self.mtx= cv_file3.getNode("camera_matrix").mat()
        self.dist = cv_file3.getNode("dist_coeff").mat()

        self.RT_cam_to_end = np.load(RT_cam_to_end_path)

        R_end_to_base = np.array([[-1,0,0],[0,0,-1],[0,-1,0]])
        self.Inverse_R_end_to_base = np.linalg.inv(R_end_to_base)



    def isRotationMatrix(self,R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self,R) :
    
        assert(self.isRotationMatrix(R))
        
        sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        
        singular = sy < 1e-6
    
        if  not singular :
            x = atan2(R[2,1] , R[2,2])
            y = atan2(-R[2,0], sy)
            z = atan2(R[1,0], R[0,0])
        else :
            x = atan2(-R[1,2], R[1,1])
            y = atan2(-R[2,0], sy)
            z = 0
    
        return np.array([x, y, z])
    
    def myRPY2R_robot(self,x, y, z):
        x=x*np.pi/180
        y=y*np.pi/180
        z=z*np.pi/180

        Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
        Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
        Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
        R = Rz@Ry@Rx
        return R
    
    def pose_robot(self,x, y, z, Tx, Ty, Tz):
        thetaX = x / 180 * pi
        thetaY = y / 180 * pi
        thetaZ = z / 180 * pi
        R = self.myRPY2R_robot(thetaX, thetaY, thetaZ)
        t = np.array([[Tx], [Ty], [Tz]])
        RT1 = np.column_stack([R, t])  # 列合并
        RT1 = np.vstack((RT1, np.array([0,0,0,1])))
        # RT1=np.linalg.inv(RT1)
        return RT1
    
    def get_normal(self,rtz):
        if self.roi is None:
            color_image, depth_image, aligned_depth_frame, intrinsics,self.roi = select_roi(self.camera)
        surface_normal, mask_3Dcoord = calculate_normal_vector(self.camera,self.roi,color_image, aligned_depth_frame,intrinsics,rtz)
        color_image = draw_roi(color_image, self.roi, self.mtx, self.dist, surface_normal, mask_3Dcoord)
        
        return surface_normal, mask_3Dcoord, color_image, depth_image

    def main(self):

        ret = self.robot.GetActualToolFlangePose()
        value = np.array(ret[1])

        R_all_end_to_base_1 = (self.myRPY2R_robot(value[3],value[4],value[5]))
        T_all_end_to_base_1 = (value[:3].reshape(3, 1))
        RT_end_to_base=np.column_stack((R_all_end_to_base_1,T_all_end_to_base_1))
        RT_end_to_base=np.vstack((RT_end_to_base,np.array([0,0,0,1])))

        R_camera = self.Inverse_R_end_to_base @ R_all_end_to_base_1 @ self.RT_cam_to_end[:3, :3] 
        R_camera_trans = self.rotationMatrixToEulerAngles(R_camera)*180/pi
        # print("R_camera_trans:",R_camera_trans)

        surface_normal, mask_3Dcoord, color_image, depth_image = self.get_normal(rtz=R_camera_trans[2])
        # surface_normal, mask_3Dcoord, color_image, depth_image = self.get_normal(rtz=0)

        # cv2.imshow("ROI", color_image)
        # cv2.imshow("Depth", depth_image)
        # print("surface_normal",surface_normal)
        # print("mask_3Dcoord,%f,%f,%f ", mask_3Dcoord[3] * 180/pi , mask_3Dcoord[4] * 180/pi , mask_3Dcoord[5] * 180/pi)


        tvec = np.array([mask_3Dcoord[0],mask_3Dcoord[1],mask_3Dcoord[2]])*1000 #mm
        rvec = np.array([mask_3Dcoord[3],mask_3Dcoord[4],mask_3Dcoord[5]])
        R_mask2cam = np.zeros((3, 3), dtype=np.float64)
        cv2.Rodrigues(rvec, R_mask2cam)

        R_all_target_to_cam = R_mask2cam
        T_all_target_to_cam = tvec.reshape(3, 1)
        RT_target_to_cam = np.column_stack((R_all_target_to_cam, T_all_target_to_cam))
        RT_target_to_cam = np.vstack((RT_target_to_cam, np.array([0, 0, 0, 1])))

        RT_target_to_base = RT_end_to_base @ self.RT_cam_to_end @ RT_target_to_cam
        trans_target_to_base = self.rotationMatrixToEulerAngles(RT_target_to_base[:3, :3])*180/pi


        result = [round(RT_target_to_base[0,3],3),round(RT_target_to_base[1,3],3),round(RT_target_to_base[2,3],3),round(trans_target_to_base[0],3),round(trans_target_to_base[1],3),round(trans_target_to_base[2],3)]
        result = [float(i) for i in result]

        return result, color_image, depth_image
 
            



if __name__ == "__main__":
    camera_ymal_path = "./camera_calibration/charuco_camera_calibration_realsense.yaml"
    RT_cam_to_end_path = './hand_eye_calibration/result/RT_4.npy'
    robot = Robot.RPC('192.168.58.2')
    normal_test = Normal_test(robot,camera_ymal_path,RT_cam_to_end_path)
    result, color_image, depth_image = normal_test.main()
    print(result)
    cv2.imshow("ROI", color_image)
    cv2.imshow("Depth", depth_image)
    cv2.waitKey(0)
