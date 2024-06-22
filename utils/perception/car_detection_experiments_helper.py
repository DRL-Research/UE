import os

import airsim.utils
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from sklearn.cluster import DBSCAN
import spatial_utils


# region Geo & Coordinates


def check_proximity(global_position_pred, global_true_pos, threshold=1):
    difference = np.linalg.norm(global_true_pos - global_position_pred)
    if difference <= threshold:
        return True, difference
    else:
        return False, np.inf


def get_yaw_by_orientation(orientation):
    # Yaw Convention will be in AirSim because of Visualization:
    # yaw == 0 -> car2 is heading x+
    # yaw == 90 -> car2 is heading y+
    # yaw == 180 -> car2 is heading x-
    # yaw == 270 -> car2 is heading y-

    orientation = [orientation.x_val, orientation.y_val, orientation.z_val, orientation.w_val]
    rotator = Rotation.from_quat(orientation)
    euler_angles = rotator.as_euler(seq='ZYX', degrees=True)
    yaw = euler_angles[0]

    if -1 <= yaw <= 1:
        return 0
    elif (-91 <= yaw <= -89) or (-181 <= yaw <= -179):
        return int(yaw + 360)
    else:
        return np.ceil(yaw)


def convert_airsim_point_to_global_point_using_lidar2map_tf(airsim_point, lidar_to_map, execution_time, curr_vel):
    # convert airsim -> ENG
    eng_point, dump = spatial_utils.convert_eng_airsim(airsim_point, [0, 0, 0])
    eng_point[0] -= execution_time * curr_vel * 2.0  # Compensate for sensor sync
    lidar_point = np.append(eng_point, 1) # add one for matmul
    point_global = np.matmul(lidar_to_map, lidar_point)[:3] # from [k,m,n,1] get the k,m,n
    return point_global

# endregion


# region DBSCAN


def evaluate_and_save_dbscan_results(dbscan_model, prediction_location, err, yaw, car1_velocity):
    path = 'car_mapping_experiments/dbscan_results/'
    file_name = 'dbscan_results.csv'
    eps = dbscan_model.eps
    min_samples = dbscan_model.min_samples
    unique_labels = np.unique(dbscan_model.labels_)
    num_clusters = len(unique_labels) - 1  # because label == -1 is for noise
    if prediction_location is not None:
        prediction_location = [round(prediction_location[0], 3), round(prediction_location[1], 3)]
        stats = {'Eps': eps, 'Min Samples': min_samples, 'Number Of Clusters Found': num_clusters,
                 'Error': round(err, 3), 'Yaw': yaw, 'Car 1 Velocity':round(car1_velocity, 3),
                 'Predicted Location': prediction_location, 'Car 2 Velocity': 0.0}

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
