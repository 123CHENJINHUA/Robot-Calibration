from fairino import Robot
from math import *
import time
import numpy as np

# 与机器人控制器建立连接，连接成功返回一个机器人对象
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

RT = np.array([[-9.99804782e-01,  1.91562448e-02, -4.84116288e-03,  2.59789468e+02],
 [ 4.25256395e-03, -3.06485457e-02, -9.99521177e-01, -1.06093404e+03],
 [-1.92954470e-02, -9.99346639e-01,  3.05610994e-02,  5.57996713e+02]])

R = RT[:3, :3]
T = RT[:3, 3]
R_trans = rotationMatrixToEulerAngles(R)*180/pi
R_trans = R_trans.tolist()
#print(R)
result = [R_trans[0],  R_trans[1], R_trans[2]]
print(result)

transformation_matrix = np.identity(4)
transformation_matrix[:3, :3] = R
transformation_matrix[:3, 3] = T


M1 = np.identity(4)
M1[:3, 3] = [0, 0, 0]
resultM = np.dot(transformation_matrix, M1)
print(resultM)
resultT = resultM[:3, 3]
robot = Robot.RPC('192.168.58.2')

offset_flag = 2
offset_pos = [0,-10,12,0,0,0]
desc_pos1 = [resultT[0],resultT[1],resultT[2],R_trans[0],  R_trans[1], R_trans[2]]


tool = 1 #工具坐标系编号

user = 0 #工件坐标系编号

ret = robot.MoveL(desc_pos1, tool, user, vel=10,offset_flag=offset_flag, offset_pos=offset_pos)   #笛卡尔空间直线运动
