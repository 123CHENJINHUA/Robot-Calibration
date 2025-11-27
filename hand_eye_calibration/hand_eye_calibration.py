import cv2
import numpy as np
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

def process_eye_in_hand(R_list,T_list,Robot_data,num):

    R_all_end_to_base_1 = []
    T_all_end_to_base_1 = []
    R_all_chess_to_cam_1 = []
    T_all_chess_to_cam_1 = []

    for value in Robot_data:
        R_all_end_to_base_1.append(myRPY2R_robot(value[3],value[4],value[5]))
        T_all_end_to_base_1.append(value[:3].reshape(3, 1))

    for value in T_list:
        T_all_chess_to_cam_1.append(1000*value[:3,0].reshape(3, 1))

    for value in R_list:
        R_all_chess_to_cam_1.append(value)

    R,T=cv2.calibrateHandEye(R_all_end_to_base_1,T_all_end_to_base_1,R_all_chess_to_cam_1,T_all_chess_to_cam_1,cv2.CALIB_HAND_EYE_HORAUD)#手眼标定
    RT=np.column_stack((R,T))
    RT = np.vstack((RT, np.array([0, 0, 0, 1])))#即为cam to end变换矩阵
    print('相机相对于末端的变换矩阵为：')
    print(RT)
    filename = open('./hand_eye_calibration/result/RT_'+num+'.txt','w')
    for value in RT:
        filename.write(str(value))
        filename.write('\n\n')
    np.save('./hand_eye_calibration/result/RT_'+num+'.npy',RT)

    result = []
    for i in range(len(R_all_end_to_base_1)):

        RT_end_to_base=np.column_stack((R_all_end_to_base_1[i],T_all_end_to_base_1[i]))
        RT_end_to_base=np.vstack((RT_end_to_base,np.array([0,0,0,1])))
        # print(RT_end_to_base)

        RT_chess_to_cam=np.column_stack((R_all_chess_to_cam_1[i],T_all_chess_to_cam_1[i]))
        RT_chess_to_cam=np.vstack((RT_chess_to_cam,np.array([0,0,0,1])))
        # print(RT_chess_to_cam)

        RT_cam_to_end=np.column_stack((R,T))
        RT_cam_to_end=np.vstack((RT_cam_to_end,np.array([0,0,0,1])))
        # print(RT_cam_to_end)

        RT_chess_to_base=RT_end_to_base@RT_cam_to_end@RT_chess_to_cam#即为固定的棋盘格相对于机器人基坐标系位姿
        # RT_chess_to_base=np.linalg.inv(RT_chess_to_base)

        result.append(RT_chess_to_base[:3,:])

    filename = open('./hand_eye_calibration/result/result1_'+num+'.txt','w')
    for value in result:
        filename.write(str(value))
        filename.write('\n\n')

    return result


def se3_average(RT_list):
    # 使用李代数平均：旋转用Rodrigues向量平均，平移取均值
    rotvecs = []
    translations = []
    for RT in RT_list:
        R = RT[:3, :3]
        t = RT[:3, 3]
        rotvec, _ = cv2.Rodrigues(R)
        rotvecs.append(rotvec.reshape(3))
        translations.append(t.reshape(3))
    rotvec_mean = np.mean(np.stack(rotvecs, axis=0), axis=0)
    t_mean = np.mean(np.stack(translations, axis=0), axis=0)
    R_mean, _ = cv2.Rodrigues(rotvec_mean.reshape(3, 1))
    RT_mean = np.eye(4)
    RT_mean[:3, :3] = R_mean
    RT_mean[:3, 3] = t_mean
    return RT_mean

def main():

    RT_mask_to_base_list = []
    for num in ['1','2']:
        T_list = np.load('./hand_eye_calibration/data/T_list_'+num+'.npy')
        R_list = np.load('./hand_eye_calibration/data/R_list_'+num+'.npy')
        Robot_data = np.load('./hand_eye_calibration/data/Robot_data_'+num+'.npy')
        # 计算并收集三个RT（每次返回的是多个位姿下的mask到base的3x4）
        RT = process_eye_in_hand(R_list, T_list, Robot_data, num)
        RT_mask_to_base_list.append(RT)


    num = '3'
    T_cam3_list = np.load('./hand_eye_calibration/data/T_list_'+num+'.npy')
    R_cam3_list = np.load('./hand_eye_calibration/data/R_list_'+num+'.npy')
    # 将 R_cam3_list 与 T_cam3_list 组合为 4x4 变换矩阵列表


    RT_mask_to_base1 = RT_mask_to_base_list[0]
    RT_mask_to_base2 = RT_mask_to_base_list[1]
    RT_mask_to_depth_cam = []

    for R_mat, T_mat in zip(R_cam3_list, T_cam3_list):
        t = 1000*T_mat[:3,0].reshape(3,1)  # 与前面处理保持一致，毫米单位
        RT = np.column_stack((R_mat, t))
        RT = np.vstack((RT, np.array([0,0,0,1])))
        RT_mask_to_depth_cam.append(RT)
    
    # 计算每次的深度相机到基坐标系，以及基坐标系2到基坐标系1，并保存
    RT_depth_cam_to_base_list = []
    RT_base2_to_base1_list = []
    for i in range(len(RT_mask_to_depth_cam)):
        # 将 process_eye_in_hand 的3x4补成4x4再参与运算
        RT_mtb1 = np.vstack((RT_mask_to_base1[i], np.array([0,0,0,1])))
        RT_mtb2 = np.vstack((RT_mask_to_base2[i], np.array([0,0,0,1])))
        RT_dcam = RT_mask_to_depth_cam[i]

        RT_depth_cam_to_base1 = RT_mtb1 @ np.linalg.inv(RT_dcam)  # 深度相机到基坐标系
        RT_base2_to_base1 = RT_mtb1 @ np.linalg.inv(RT_mtb2)      # 基坐标系2到基坐标系1

        RT_depth_cam_to_base_list.append(RT_depth_cam_to_base1)
        RT_base2_to_base1_list.append(RT_base2_to_base1)

    filename = open('./hand_eye_calibration/result/RT_depth_cam_to_base'+'.txt','w')
    for value in RT_depth_cam_to_base_list:
        filename.write(str(value))
        filename.write('\n\n')
    filename.close()

    filename = open('./hand_eye_calibration/result/RT_base2_to_base1'+'.txt','w')
    for value in RT_base2_to_base1_list:
        filename.write(str(value))
        filename.write('\n\n')
    filename.close()

    # 对多次结果进行李代数平均，得到更精确的RT
    RT_depth_cam_to_base_mean = se3_average(RT_depth_cam_to_base_list)
    RT_base2_to_base1_mean = se3_average(RT_base2_to_base1_list)

    np.save('./hand_eye_calibration/result/RT_depth_cam_to_base_mean.npy', RT_depth_cam_to_base_mean)
    np.save('./hand_eye_calibration/result/RT_base2_to_base1_mean.npy', RT_base2_to_base1_mean)

    with open('./hand_eye_calibration/result/RT_depth_cam_to_base_mean.txt','w') as f:
        for row in RT_depth_cam_to_base_mean:
            f.write(str(row))
            f.write('\n\n')
    with open('./hand_eye_calibration/result/RT_base2_to_base1_mean.txt','w') as f:
        for row in RT_base2_to_base1_mean:
            f.write(str(row))
            f.write('\n\n')


if __name__ == '__main__':
    main()