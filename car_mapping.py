import os.path
import matplotlib.pyplot as plt
import numpy as np

import dbscan_utils
from experiments_helper import *
from dbscan_utils import *


def show_cloud(title, points_cloud: np.ndarray, save_path='car_mapping_experiments'):
    x = points_cloud[:, 0]
    y = points_cloud[:, 1]
    # convert from airsim to global (points_cloud assume to be airsim)
    x = points_cloud[:, 1] * -1
    y = points_cloud[:, 0]
    plt.scatter(x, y, marker='o', s=3)
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')
    plt.xlim([0, 50])
    plt.ylim([-50, 50])
    plt.title(title)
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    fig_name = f'{title}.png'
    plt.savefig(os.path.join(save_path, fig_name))
    plt.show()


def car_detection(points_cloud, lidar_to_map, execution_time, velocity, yaw, other_true_pos):
    # step 1: filter the cloud
    # assume that 50m is not relevant for now, it's far enough and 10m is too close
    min_distance_from_car1, max_distance_from_car1 = 10.0, 50.0
    # assume height of a car is between 0.5m to 4m
    min_height_of_point, max_height_of_point = 0.5, 4.0
    #  check if the point given from lidar has height
    filtered_points_cloud = dbscan_utils.filter_cloud(points_cloud, min_distance=min_distance_from_car1,
                              max_distance=max_distance_from_car1, min_height=-min_height_of_point,
                                                      max_height=max_height_of_point)
    # step 2: cluster cloud & save results
    run_dbscan_params_experiments(filtered_points_cloud, lidar_to_map, execution_time, velocity, yaw, other_true_pos)


def run_dbscan_params_experiments(filtered_points_cloud, lidar_to_map, execution_time, velocity, yaw, other_true_pos):
    eps_candidates = list(np.arange(0.1, 4.5, 0.1))  # min distance between two points to consider them neighbours
    min_samples_candidates = list(np.arange(3, 10, 1))  # min number of neighbours to count as a core point

    for eps in eps_candidates:
        for min_samples in min_samples_candidates:
            # build a dbscan model -> make prediction -> evaluate results -> save results
            dbscan_candidate_model = DBSCAN(eps=eps, min_samples=min_samples).fit(filtered_points_cloud)
            prediction_location, err = run_ONE_dbscan_params_experiment(dbscan_candidate_model, lidar_to_map, execution_time,
                                                               velocity, other_true_pos)
            evaluate_and_save_dbscan_results(dbscan_candidate_model, prediction_location, err, yaw, velocity)
    print('-' * 100)
    print('Finished to run DBSCAN Experiments')
    print('-' * 100)


def run_ONE_dbscan_params_experiment(dbscan_model, lidar_to_map, execution_time, curr_vel, other_true_pos):
    curr_segments, airsim_curr_centroids, curr_labels = dbscan_utils.collate_segmentation(dbscan_model, 1.0)
    airsim_curr_centroids.sort(key=lambda x: np.linalg.norm(x))
    close_dict = {}
    global_centroids = []
    # target
    car2_true_position = other_true_pos  # [0, -30, 0]
    global_car2_true_position = convert_airsim_point_to_global_point_using_lidar2map_tf(
        car2_true_position, lidar_to_map, execution_time, curr_vel)
    # candidates
    for airsim_centroid in airsim_curr_centroids:
        global_centroid = convert_airsim_point_to_global_point_using_lidar2map_tf(
            airsim_centroid, lidar_to_map, execution_time, curr_vel)
        global_centroids.append(global_centroid)
    # compare candidates to target

    global_car2_position_prediction, smallest_difference,  = None, -np.inf
    for i in range(len(global_centroids)):
        global_centroid = global_centroids[i]
        is_close_enough, difference = check_proximity_global(global_centroid, global_car2_true_position)
        if is_close_enough and difference < smallest_difference:
            global_car2_position_prediction, smallest_difference = global_centroid, difference

    return global_car2_position_prediction, smallest_difference


#
# 1. add documentation in code & notion
# 2. add speed of the car's
# 3. check about the metrics, interesting the best prediction so change
#    just to get the best prediction and not all the possible predictions
# 4. add to notion the Project Goal's

# 5. general:
#   after that we will try more complex experiments like movement
#   change car2 distance from car1 and then experiment
#   move car2 to different static locations and run experiment

print('check correction')



