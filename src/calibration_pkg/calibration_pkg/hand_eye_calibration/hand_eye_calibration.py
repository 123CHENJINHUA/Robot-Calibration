import cv2
import numpy as np
from math import *
import os
from pathlib import Path


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

def process_eye_in_hand(R_list,T_list,Robot_data,num,result_dir):

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
    
    filename = open(os.path.join(result_dir, 'RT_'+num+'.txt'),'w')
    for value in RT:
        filename.write(str(value))
        filename.write('\n\n')
    np.save(os.path.join(result_dir, 'RT_'+num+'.npy'),RT)

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

    filename = open(os.path.join(result_dir, 'result1_'+num+'.txt'),'w')
    for value in result:
        filename.write(str(value))
        filename.write('\n\n')

    return result


def se3_average(RT_list, remove_outliers=True, outlier_method='iqr', threshold=1.5):
    """
    SE3 average using Lie algebra, with optional outlier removal
    
    Parameters:
    -----------
    RT_list : list of 4x4 np.array
        List of transformation matrices
    remove_outliers : bool
        Whether to remove outliers before averaging
    outlier_method : str
        'iqr' (Interquartile Range) or 'sigma' (3-sigma rule)
    threshold : float
        For 'iqr': IQR multiplier (default 1.5)
        For 'sigma': sigma multiplier (default 3.0)
    
    Returns:
    --------
    RT_mean : 4x4 np.array
        Average transformation matrix
    """
    if len(RT_list) == 0:
        raise ValueError("RT_list is empty")
    
    if len(RT_list) == 1:
        return RT_list[0]
    
    # Extract rotation vectors and translations
    rotvecs = []
    translations = []
    for RT in RT_list:
        R = RT[:3, :3]
        t = RT[:3, 3]
        rotvec, _ = cv2.Rodrigues(R)
        rotvecs.append(rotvec.reshape(3))
        translations.append(t.reshape(3))
    
    rotvecs = np.array(rotvecs)
    translations = np.array(translations)
    
    # Remove outliers if requested
    if remove_outliers and len(RT_list) > 3:
        valid_indices = detect_outliers(rotvecs, translations, method=outlier_method, threshold=threshold)
        
        if len(valid_indices) == 0:
            print("Warning: All samples detected as outliers. Using all data.")
            valid_indices = list(range(len(RT_list)))
        elif len(valid_indices) < len(RT_list):
            print(f"Outlier removal: {len(RT_list) - len(valid_indices)} outliers removed, {len(valid_indices)} samples remaining")
        
        rotvecs = rotvecs[valid_indices]
        translations = translations[valid_indices]
    
    # Calculate mean
    rotvec_mean = np.mean(rotvecs, axis=0)
    t_mean = np.mean(translations, axis=0)
    
    R_mean, _ = cv2.Rodrigues(rotvec_mean.reshape(3, 1))
    RT_mean = np.eye(4)
    RT_mean[:3, :3] = R_mean
    RT_mean[:3, 3] = t_mean
    
    return RT_mean


def detect_outliers(rotvecs, translations, method='iqr', threshold=1.5):
    """
    Detect outliers in rotation vectors and translations
    
    Parameters:
    -----------
    rotvecs : np.array (N, 3)
        Rotation vectors
    translations : np.array (N, 3)
        Translation vectors
    method : str
        'iqr' (Interquartile Range) or 'sigma' (3-sigma rule)
    threshold : float
        For 'iqr': IQR multiplier (default 1.5)
        For 'sigma': sigma multiplier (default 3.0)
    
    Returns:
    --------
    valid_indices : list
        Indices of valid (non-outlier) samples
    """
    n_samples = len(rotvecs)
    
    if method == 'iqr':
        # Use IQR method
        valid_mask = np.ones(n_samples, dtype=bool)
        
        # Check rotation vectors
        rotvec_norms = np.linalg.norm(rotvecs, axis=1)
        q1, q3 = np.percentile(rotvec_norms, [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - threshold * iqr
        upper_bound = q3 + threshold * iqr
        valid_mask &= (rotvec_norms >= lower_bound) & (rotvec_norms <= upper_bound)
        
        # Check translations
        translation_norms = np.linalg.norm(translations, axis=1)
        q1, q3 = np.percentile(translation_norms, [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - threshold * iqr
        upper_bound = q3 + threshold * iqr
        valid_mask &= (translation_norms >= lower_bound) & (translation_norms <= upper_bound)
        
    elif method == 'sigma':
        # Use 3-sigma rule (or custom sigma multiplier)
        valid_mask = np.ones(n_samples, dtype=bool)
        
        # Check rotation vectors
        rotvec_norms = np.linalg.norm(rotvecs, axis=1)
        mean_rot = np.mean(rotvec_norms)
        std_rot = np.std(rotvec_norms)
        if std_rot > 1e-6:
            valid_mask &= np.abs(rotvec_norms - mean_rot) <= threshold * std_rot
        
        # Check translations
        translation_norms = np.linalg.norm(translations, axis=1)
        mean_trans = np.mean(translation_norms)
        std_trans = np.std(translation_norms)
        if std_trans > 1e-6:
            valid_mask &= np.abs(translation_norms - mean_trans) <= threshold * std_trans
    
    else:
        raise ValueError(f"Unknown outlier detection method: {method}")
    
    valid_indices = np.where(valid_mask)[0]
    return valid_indices

def main():
    
    file_path = Path(__file__).resolve()
    pkg_name = 'calibration_pkg'

    # Prefer saving under the SOURCE workspace: <ws>/src/calibration_pkg/calibration_pkg/hand_eye_calibration/data
    data_dir = None
    result_dir = None
    for p in file_path.parents:
        if p.name == 'install':
            ws_root = p.parent  # workspace root
            data_dir = ws_root / 'src' / pkg_name / pkg_name / 'hand_eye_calibration' / 'data'
            result_dir = ws_root / 'src' / pkg_name / pkg_name / 'hand_eye_calibration' / 'result'
            break

    # If not running from install, fall back to package root path
    if data_dir is None:
        data_dir = file_path.parent / 'data'
        result_dir = file_path.parent / 'result'
    
    data_dir = str(data_dir)
    result_dir = str(result_dir)
    
    # Create result directory if it doesn't exist
    os.makedirs(result_dir, exist_ok=True)
    
    print(f'Reading data from: {data_dir}')
    print(f'Writing results to: {result_dir}')

    RT_mask_to_base_list = []
    for num in ['1','2']:
        T_list = np.load(os.path.join(data_dir, 'T_list_'+num+'.npy'))
        R_list = np.load(os.path.join(data_dir, 'R_list_'+num+'.npy'))
        Robot_data = np.load(os.path.join(data_dir, 'Robot_data_'+num+'.npy'))
        # 计算并收集三个RT（每次返回的是多个位姿下的mask到base的3x4）
        RT = process_eye_in_hand(R_list, T_list, Robot_data, num, result_dir)
        RT_mask_to_base_list.append(RT)


    num = '3'
    T_cam3_list = np.load(os.path.join(data_dir, 'T_list_'+num+'.npy'))
    R_cam3_list = np.load(os.path.join(data_dir, 'R_list_'+num+'.npy'))
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
    RT_depth_cam_to_base1_list = []
    RT_depth_cam_to_base2_list = []
    RT_base2_to_base1_list = []
    for i in range(len(RT_mask_to_depth_cam)):
        # 将 process_eye_in_hand 的3x4补成4x4再参与运算
        RT_mtb1 = np.vstack((RT_mask_to_base1[i], np.array([0,0,0,1])))
        RT_mtb2 = np.vstack((RT_mask_to_base2[i], np.array([0,0,0,1])))
        RT_dcam = RT_mask_to_depth_cam[i]

        RT_depth_cam_to_base1 = RT_mtb1 @ np.linalg.inv(RT_dcam)  # 深度相机到基坐标系
        RT_depth_cam_to_base2 = RT_mtb2 @ np.linalg.inv(RT_dcam)  # 深度相机到基坐标系2

        RT_base2_to_base1 = RT_mtb1 @ np.linalg.inv(RT_mtb2)      # 基坐标系2到基坐标系1

        RT_depth_cam_to_base1_list.append(RT_depth_cam_to_base1)
        RT_depth_cam_to_base2_list.append(RT_depth_cam_to_base2)
        RT_base2_to_base1_list.append(RT_base2_to_base1)

    filename = open(os.path.join(result_dir, 'RT_depth_cam_to_base1.txt'),'w')
    for value in RT_depth_cam_to_base1_list:
        filename.write(str(value))
        filename.write('\n\n')
    filename.close()

    filename = open(os.path.join(result_dir, 'RT_depth_cam_to_base2.txt'),'w')
    for value in RT_depth_cam_to_base2_list:
        filename.write(str(value))
        filename.write('\n\n')
    filename.close()

    filename = open(os.path.join(result_dir, 'RT_base2_to_base1.txt'),'w')
    for value in RT_base2_to_base1_list:
        filename.write(str(value))
        filename.write('\n\n')
    filename.close()

    # Perform Lie algebra averaging with outlier removal for more accurate RT (iqr/sigma)
    print("\n=== Calculating depth camera to base transformation (with outlier removal) ===")
    RT_depth_cam_to_base1_mean = se3_average(RT_depth_cam_to_base1_list, remove_outliers=True, outlier_method='iqr', threshold=1.5)
    RT_depth_cam_to_base2_mean = se3_average(RT_depth_cam_to_base2_list, remove_outliers=True, outlier_method='iqr', threshold=1.5)

    print("\n=== Calculating base2 to base1 transformation (with outlier removal) ===")
    RT_base2_to_base1_mean = se3_average(RT_base2_to_base1_list, remove_outliers=True, outlier_method='iqr', threshold=1.5)

    np.save(os.path.join(result_dir, 'RT_depth_cam_to_base1.npy'), RT_depth_cam_to_base1_list)
    np.save(os.path.join(result_dir, 'RT_depth_cam_to_base2.npy'), RT_depth_cam_to_base2_list)
    np.save(os.path.join(result_dir, 'RT_base2_to_base1.npy'), RT_base2_to_base1_list)

    np.save(os.path.join(result_dir, 'RT_depth_cam_to_base1_mean.npy'), RT_depth_cam_to_base1_mean)
    np.save(os.path.join(result_dir, 'RT_depth_cam_to_base2_mean.npy'), RT_depth_cam_to_base2_mean)
    np.save(os.path.join(result_dir, 'RT_base2_to_base1_mean.npy'), RT_base2_to_base1_mean)

    with open(os.path.join(result_dir, 'RT_depth_cam_to_base1_mean.txt'),'w') as f:
        for row in RT_depth_cam_to_base1_mean:
            f.write(str(row))
            f.write('\n\n')
    with open(os.path.join(result_dir, 'RT_depth_cam_to_base2_mean.txt'),'w') as f:
        for row in RT_depth_cam_to_base2_mean:
            f.write(str(row))
            f.write('\n\n')
    with open(os.path.join(result_dir, 'RT_base2_to_base1_mean.txt'),'w') as f:
        for row in RT_base2_to_base1_mean:
            f.write(str(row))
            f.write('\n\n')


if __name__ == '__main__':
    main()