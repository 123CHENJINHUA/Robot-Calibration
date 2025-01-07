import numpy as np
from math import *
from fairino import Robot

def isRotationMatrix(R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))
    
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

def myRPY2R_robot(x, y, z):
    x=x*np.pi/180
    y=y*np.pi/180
    z=z*np.pi/180

    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz@Ry@Rx
    return R

R_end_to_base = np.array([[-1,0,0],[0,0,-1],[0,-1,0]])

x = 0
y = 0
z = 20
R = myRPY2R_robot(x, y, z)
print("R",R)

R_end_to_base =  R_end_to_base @ R
print("R_end_to_base",R_end_to_base)

RT_cam_to_end = np.load('./hand_eye_calibration/result/RT_4.npy')
Inverse_R_cam_to_end = np.linalg.inv(RT_cam_to_end[:3, :3])

R_end_to_base = np.dot(R_end_to_base,Inverse_R_cam_to_end)

R_end_to_base_trans = rotationMatrixToEulerAngles(R_end_to_base)*180/pi

print("R_end_to_base_trans",R_end_to_base_trans)

tool = 0 #工具坐标系编号

user = 0 #工件坐标系编号
robot = Robot.RPC('192.168.58.2')
offset_flag = 2
offset_pos = [0,0,0,0,0,0]
desc_pos1 = [85.072,-381.534,585.85,R_end_to_base_trans[0],  R_end_to_base_trans[1], R_end_to_base_trans[2]]
ret = robot.MoveL(desc_pos1, tool, user, vel=10,offset_flag=offset_flag, offset_pos=offset_pos)
print(ret)