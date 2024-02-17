import copy
import os.path
import matplotlib.pyplot as plt
import numpy as np

import dbscan_utils
import spatial_utils
from experiments_helper import *
from dbscan_utils import *

yaw_mapping = {0: '^', 90: '>', 180: 'v', 270: '<'}

def show_cloud(points_cloud_eng, yaw, save_path='car_mapping_experiments'):
    yaw_description = yaw_mapping[yaw]
    title = f'Points Cloud After Clustering yaw:{yaw} Face-Direction: {yaw_description}'
    x = points_cloud_eng[:, 0]
    y = points_cloud_eng[:, 1]
    # x,y flip because of airsim coordinates
    plt.scatter(y, x, marker='o', s=3)
    plt.xlabel('Y-axis')
    plt.ylabel('X-axis')
    plt.xlim([-40, 40])
    plt.ylim([-40, 40])
    plt.title(title)
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    fig_name = f'{title}.png'
    plt.savefig(os.path.join(save_path, fig_name))
    plt.show()


def car_detection(points_cloud, lidar_to_map, execution_time, velocity, yaw, other_true_pos_eng):
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
    run_dbscan_params_experiments(filtered_points_cloud, lidar_to_map, execution_time, velocity, yaw, other_true_pos_eng)


def run_dbscan_params_experiments(filtered_points_cloud, lidar_to_map, execution_time, velocity, yaw, other_true_pos_eng):
    eps_candidates = list(np.arange(0.1, 4.5, 0.1))  # min distance between two points to consider them neighbours
    min_samples_candidates = list(np.arange(1, 10, 1))  # min number of neighbours to count as a core point

    for eps in eps_candidates:
        for min_samples in min_samples_candidates:
            # build a dbscan model -> make prediction -> evaluate results -> save results
            dbscan_candidate_model = DBSCAN(eps=eps, min_samples=min_samples).fit(filtered_points_cloud)
            predicted_other_car_location, err = run_ONE_dbscan_params_experiment(dbscan_candidate_model, lidar_to_map, execution_time,
                                                                                 velocity, other_true_pos_eng)
            #evaluate_and_save_dbscan_results(dbscan_candidate_model, predicted_other_car_location, err, yaw, velocity)
    print('-' * 100)
    print('Finished to run DBSCAN Experiments')
    print('-' * 100)


def run_ONE_dbscan_params_experiment(dbscan_model, lidar_to_map, execution_time, curr_vel, other_car_eng_true_position):
    curr_segments, airsim_curr_centroids, curr_labels = dbscan_utils.collate_segmentation(dbscan_model, 1.0)
    airsim_curr_centroids.sort(key=lambda x: np.linalg.norm(x))
    eng_global_centroids = []

    # convert candidates predictions to ENG
    for airsim_centroid in airsim_curr_centroids:
        eng_global_centroid = convert_airsim_point_to_global_point_using_lidar2map_tf(
            airsim_centroid, lidar_to_map, execution_time, curr_vel)
        eng_global_centroids.append(eng_global_centroid)

    # compare candidates to target
    car2_eng_position_prediction, smallest_difference,  = None, np.inf
    for eng_global_centroid in eng_global_centroids:
        is_close_enough, difference = check_proximity(eng_global_centroid, other_car_eng_true_position)
        if is_close_enough and difference < smallest_difference:
            car2_eng_position_prediction, smallest_difference = eng_global_centroid, difference

    return car2_eng_position_prediction, smallest_difference

