import os
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from sklearn.cluster import DBSCAN
import spatial_utils


# region Geo & Coordinates


def check_proximity_global(global_position_pred, global_true_pos, threshold=5):
    difference = np.linalg.norm(global_true_pos - global_position_pred)
    if difference <= threshold:  # check if this is in meters
        return True, difference
    else:
        return False, np.inf


def simple_convert_point_airsim_to_global(airsim_point: np.array):
    x1, y1 = airsim_point[0], airsim_point[1]
    x2 = y1
    y2 = -x1
    z2 = airsim_point[2]
    return np.array([x2, y2, z2])


def simple_convert_point_global_to_arisim(global_point: np.array):
    x2, y2 = global_point[0], global_point[1]
    x1 = -y2
    y1 = x2
    z1 = global_point[2]
    return np.array([x1, y1, z1])


def get_yaw_by_orientation(orientation):
    orientation = [orientation.x_val, orientation.y_val, orientation.z_val, orientation.w_val]
    rotator = Rotation.from_quat(orientation)
    euler_angles = rotator.as_euler(seq='ZYX', degrees=True)
    yaw = euler_angles[0]
    return np.ceil(yaw)


def convert_airsim_point_to_global_point_using_lidar2map_tf(airsim_point, lidar_to_map, execution_time, curr_vel):
    eng_point, dump = spatial_utils.convert_eng_airsim(airsim_point, [0, 0, 0])
    eng_point[0] -= execution_time * curr_vel * 2.0  # Compensate for sensor sync
    lidar_point = np.append(eng_point, 1)
    point_global = np.matmul(lidar_to_map, lidar_point)[:3]
    return point_global


def get_other_position_ref_to_self(self_pos, other_pos):
    estimated_x = other_pos.x_val - self_pos.x_val
    estimated_y = other_pos.y_val - self_pos.y_val
    estimated_z = other_pos.z_val - self_pos.z_val
    other_estimated_position = [estimated_x, estimated_y, estimated_z]
    return other_estimated_position

# endregion


# region DBSCAN


def evaluate_and_save_dbscan_results(dbscan_model, prediction_location, err, yaw, car1_velocity):
    path = 'car_mapping_experiments/dbscan_car_detection_experiments/'
    file_name = 'dbscan_car_detection_experiments.csv'
    eps = dbscan_model.eps
    min_samples = dbscan_model.min_samples
    unique_labels = np.unique(dbscan_model.labels_)
    num_clusters = len(unique_labels) - 1  # because label == -1 is for noise
    if prediction_location is not None:
        stats = {'Eps': eps, 'Min Samples': min_samples, 'Number Of Clusters Found': num_clusters,
                 'Error': round(err, 3), 'Yaw': yaw, 'Velocity': car1_velocity,
                 'Predicted Location': prediction_location}

        save_results_to_csv(path + file_name, stats)


# endregion


def save_results_to_csv(path, results: dict):
    if not os.path.isfile(path):
        df = pd.DataFrame([results])
        df.to_csv(path, index=False)
    else:
        existing_data = pd.read_csv(path)
        result_df = pd.DataFrame([results])
        updated_data = pd.concat([existing_data, result_df], ignore_index=True)
        updated_data.to_csv(path, index=False)


