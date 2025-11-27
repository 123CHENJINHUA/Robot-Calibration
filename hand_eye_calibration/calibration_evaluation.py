import cv2
import numpy as np
import matplotlib.pyplot as plt
from math import *


def rotation_matrix_to_euler(R):
    """
    Convert rotation matrix to Euler angles (roll, pitch, yaw) in degrees
    """
    sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    
    return np.array([x, y, z]) * 180.0 / np.pi


def rotation_matrix_to_axis_angle(R):
    """
    Convert rotation matrix to axis-angle representation
    """
    rotvec, _ = cv2.Rodrigues(R)
    angle = np.linalg.norm(rotvec)
    return angle * 180.0 / np.pi


def calculate_pose_error(RT1, RT2):
    """
    Calculate position and rotation errors between two transformation matrices
    RT1, RT2: 4x4 transformation matrices
    Returns: position_error (mm), rotation_error (degrees)
    """
    # Position error
    t1 = RT1[:3, 3]
    t2 = RT2[:3, 3]
    position_error = np.linalg.norm(t1 - t2)
    
    # Rotation error
    R1 = RT1[:3, :3]
    R2 = RT2[:3, :3]
    R_error = R1 @ R2.T
    rotation_error = rotation_matrix_to_axis_angle(R_error)
    
    return position_error, rotation_error


def calculate_statistics(data):
    """
    Calculate statistical information
    """
    mean_val = np.mean(data)
    std_val = np.std(data)
    max_val = np.max(data)
    min_val = np.min(data)
    
    return {
        'mean': mean_val,
        'std': std_val,
        'max': max_val,
        'min': min_val,
        'rms': np.sqrt(np.mean(np.array(data)**2))
    }


def plot_error_distribution(errors, title, xlabel, filename):
    """
    Plot error distribution graphs
    """
    plt.figure(figsize=(10, 6))
    
    # Error vs index plot
    plt.subplot(1, 2, 1)
    plt.plot(errors, 'b.-')
    plt.title(f'{title}')
    plt.xlabel('Index')
    plt.ylabel(xlabel)
    plt.grid(True)
    
    # Error histogram
    plt.subplot(1, 2, 2)
    plt.hist(errors, bins=20, alpha=0.7, edgecolor='black')
    plt.title(f'{title} - Distribution')
    plt.xlabel(xlabel)
    plt.ylabel('Frequency')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    # plt.show()


def evaluate_against_mean(RT_list, RT_mean, name):
    """
    Evaluate errors of each transformation matrix relative to the mean
    """
    print(f"\n=== {name} Error Evaluation Against Mean ===")
    
    position_errors = []
    rotation_errors = []
    
    for i, RT in enumerate(RT_list):
        pos_err, rot_err = calculate_pose_error(RT, RT_mean)
        position_errors.append(pos_err)
        rotation_errors.append(rot_err)
        print(f"Measurement {i+1}: Position error = {pos_err:.3f} mm, Rotation error = {rot_err:.3f} deg")
    
    # Statistical information
    pos_stats = calculate_statistics(position_errors)
    rot_stats = calculate_statistics(rotation_errors)
    
    print(f"\nPosition error statistics relative to mean (mm):")
    print(f"  Mean: {pos_stats['mean']:.3f}")
    print(f"  Std: {pos_stats['std']:.3f}")
    print(f"  Max: {pos_stats['max']:.3f}")
    print(f"  Min: {pos_stats['min']:.3f}")
    print(f"  RMS: {pos_stats['rms']:.3f}")
    
    print(f"\nRotation error statistics relative to mean (deg):")
    print(f"  Mean: {rot_stats['mean']:.3f}")
    print(f"  Std: {rot_stats['std']:.3f}")
    print(f"  Max: {rot_stats['max']:.3f}")
    print(f"  Min: {rot_stats['min']:.3f}")
    print(f"  RMS: {rot_stats['rms']:.3f}")
    
    # Plot error distribution
    plot_error_distribution(position_errors, 
                          f'{name} Position Error', 
                          'Position Error (mm)',
                          f'./hand_eye_calibration/result/{name.replace(" ", "_")}_position_error.png')
    
    plot_error_distribution(rotation_errors, 
                          f'{name} Rotation Error', 
                          'Rotation Error (deg)',
                          f'./hand_eye_calibration/result/{name.replace(" ", "_")}_rotation_error.png')
    
    return pos_stats, rot_stats


def analyze_transformation_matrix(RT, name):
    """
    Analyze detailed information of transformation matrix
    """
    print(f"\n=== {name} Transformation Matrix Analysis ===")
    
    # Extract rotation and translation
    R = RT[:3, :3]
    t = RT[:3, 3]
    
    print(f"Translation vector (mm): [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}]")
    print(f"Translation distance: {np.linalg.norm(t):.3f} mm")
    
    # Euler angles
    euler_angles = rotation_matrix_to_euler(R)
    print(f"Euler angles (deg): Roll={euler_angles[0]:.3f}, Pitch={euler_angles[1]:.3f}, Yaw={euler_angles[2]:.3f}")
    
    # Rotation angle
    rotation_angle = rotation_matrix_to_axis_angle(R)
    print(f"Total rotation angle: {rotation_angle:.3f} deg")
    
    # Check orthogonality of rotation matrix
    orthogonality_error = np.linalg.norm(R @ R.T - np.eye(3))
    print(f"Rotation matrix orthogonality error: {orthogonality_error:.6f}")
    
    # Check if determinant is 1
    det_R = np.linalg.det(R)
    print(f"Rotation matrix determinant: {det_R:.6f} (should be close to 1)")


def main():
    """
    Main evaluation function
    """
    print("Starting calibration result evaluation...")
    
    try:
        # Load calibration results
        RT_depth_cam_to_base_list = np.load('./hand_eye_calibration/result/RT_depth_cam_to_base.npy')
        RT_base2_to_base1_list = np.load('./hand_eye_calibration/result/RT_base2_to_base1.npy')
        
        RT_depth_cam_to_base_mean = np.load('./hand_eye_calibration/result/RT_depth_cam_to_base_mean.npy')
        RT_base2_to_base1_mean = np.load('./hand_eye_calibration/result/RT_base2_to_base1_mean.npy')
        
        print(f"Successfully loaded calibration results:")
        print(f"  Depth camera to base transformations: {len(RT_depth_cam_to_base_list)} measurements")
        print(f"  Base2 to base1 transformations: {len(RT_base2_to_base1_list)} measurements")
        
    except FileNotFoundError as e:
        print(f"Error: Cannot find calibration result files - {e}")
        return
    except Exception as e:
        print(f"Error: Problem loading calibration results - {e}")
        return
    
    # Analyze mean transformation matrices
    analyze_transformation_matrix(RT_depth_cam_to_base_mean, "Depth Camera to Base (Mean)")
    analyze_transformation_matrix(RT_base2_to_base1_mean, "Base2 to Base1 (Mean)")
    
    # Evaluate errors against mean values only
    depth_cam_vs_mean = evaluate_against_mean(RT_depth_cam_to_base_list, RT_depth_cam_to_base_mean, "Depth Camera to Base")
    base_transform_vs_mean = evaluate_against_mean(RT_base2_to_base1_list, RT_base2_to_base1_mean, "Base2 to Base1")
    
    # Save evaluation report
    with open('./hand_eye_calibration/result/calibration_evaluation_report.txt', 'w', encoding='utf-8') as f:
        f.write("Calibration Result Evaluation Report\n")
        f.write("=" * 50 + "\n\n")
        
        f.write("1. Data Overview\n")
        f.write(f"   Depth camera to base transformations: {len(RT_depth_cam_to_base_list)} measurements\n")
        f.write(f"   Base2 to base1 transformations: {len(RT_base2_to_base1_list)} measurements\n\n")
        
        if depth_cam_vs_mean:
            f.write("2. Depth Camera to Base Transformation Error Against Mean\n")
            f.write(f"   Position error mean: {depth_cam_vs_mean[0]['mean']:.3f} mm\n")
            f.write(f"   Position error std: {depth_cam_vs_mean[0]['std']:.3f} mm\n")
            f.write(f"   Position error RMS: {depth_cam_vs_mean[0]['rms']:.3f} mm\n")
            f.write(f"   Position error max: {depth_cam_vs_mean[0]['max']:.3f} mm\n")
            f.write(f"   Rotation error mean: {depth_cam_vs_mean[1]['mean']:.3f} deg\n")
            f.write(f"   Rotation error std: {depth_cam_vs_mean[1]['std']:.3f} deg\n")
            f.write(f"   Rotation error RMS: {depth_cam_vs_mean[1]['rms']:.3f} deg\n")
            f.write(f"   Rotation error max: {depth_cam_vs_mean[1]['max']:.3f} deg\n\n")
        
        if base_transform_vs_mean:
            f.write("3. Base2 to Base1 Transformation Error Against Mean\n")
            f.write(f"   Position error mean: {base_transform_vs_mean[0]['mean']:.3f} mm\n")
            f.write(f"   Position error std: {base_transform_vs_mean[0]['std']:.3f} mm\n")
            f.write(f"   Position error RMS: {base_transform_vs_mean[0]['rms']:.3f} mm\n")
            f.write(f"   Position error max: {base_transform_vs_mean[0]['max']:.3f} mm\n")
            f.write(f"   Rotation error mean: {base_transform_vs_mean[1]['mean']:.3f} deg\n")
            f.write(f"   Rotation error std: {base_transform_vs_mean[1]['std']:.3f} deg\n")
            f.write(f"   Rotation error RMS: {base_transform_vs_mean[1]['rms']:.3f} deg\n")
            f.write(f"   Rotation error max: {base_transform_vs_mean[1]['max']:.3f} deg\n\n")
        
        f.write("Evaluation completed. Detailed charts saved to result directory.\n")
    
    print("\nEvaluation completed! Report saved to './hand_eye_calibration/result/calibration_evaluation_report.txt'")
    print("Error distribution plots saved to result directory")


if __name__ == '__main__':
    main()
