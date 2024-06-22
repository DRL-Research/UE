import os.path
import matplotlib.pyplot as plt
from utils.perception import dbscan_utils
from utils.perception.car_detection_experiments_helper import *
from utils.perception.dbscan_utils import *

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
    x = points_cloud_eng[:, 0]
    y = points_cloud_eng[:, 1]


def show_clusters(model, car2_position, eps, min_samples, points):
    title = f"Clusters Found with Eps:{eps}, Min-Samples:{min_samples}"
    centroids = []
    labels = model.labels_
    for label in np.unique(labels):
        if label == -1:  # Noise points
            continue
        cluster_points = points[labels == label]
        centroid = cluster_points.mean(axis=0)  # Centroid as mean of cluster points
        centroids.append(centroid)

    x = [centroid[0] for centroid in centroids]
    y = [centroid[1] * -1 for centroid in centroids]
    plt.scatter(y, x, marker='o', s=3)
    plt.xlabel('Y-axis')
    plt.ylabel('X-axis')
    plt.xlim([-20, -10])
    plt.ylim([23, 27])
    plt.title(title)
    plt.show()


def car_detection(airsim_client, points_cloud, lidar_to_map, execution_time, velocity, other_car_name):
    """
    :param airsim_client:
    :param points_cloud: received from the lidar data
    :param lidar_to_map: lidar to map transition matrix
    :param execution_time:
    :param velocity:
    :param other_car_name: that we want to detect, this is for comparison
    :return:
    """
    other_car_pos_and_orientation = airsim_client.simGetObjectPose(other_car_name)  # contain position + orientation
    other_car_position = other_car_pos_and_orientation.position
    other_car_position_as_np_array = np.array(
        [other_car_position.x_val, other_car_position.y_val, other_car_position.z_val])
    other_car_position_eng, _ = spatial_utils.convert_eng_airsim(other_car_position_as_np_array, [0, 0, 0])
    other_car_true_pos_eng = other_car_position_eng
    # region Keep In Mind
    # we will keep this to remember this is another way to get the correct position of car2:
    # car2_also_location = spatial_utils.tf_matrix_from_airsim_object(car2_pos_and_oriantation)
    # car2_also_location = car2_also_location[3, :3] == car2_position_eng
    # endregion

    # we want to test our experiments with different kinds of yaw & we don't want to do it HARD-CODDED
    # we just need to set the yaw in the settings & then we calculate it
    other_car_orientation = other_car_pos_and_orientation.orientation
    yaw = get_yaw_by_orientation(other_car_orientation)
    # ---------------------------------------------------------------------------------------------------
    # step 1: filter the cloud
    # assume that 50m is not relevant for now, it's far enough and 10m is too close
    min_distance_from_car1, max_distance_from_car1 = 10.0, 50.0
    # assume height of a car is between 0.5m to 4m
    min_height_of_point, max_height_of_point = 0.5, 4.0
    #  check if the point given from lidar has height
    filtered_points_cloud = dbscan_utils.filter_cloud(points_cloud,
                                                      min_distance=min_distance_from_car1,
                                                      max_distance=max_distance_from_car1,
                                                      min_height=-min_height_of_point,
                                                      max_height=max_height_of_point)

    # step 2: cluster cloud & save results
    run_dbscan_params_experiments(filtered_points_cloud, lidar_to_map, execution_time,
                                  velocity, yaw, other_car_true_pos_eng)


def run_dbscan_params_experiments(filtered_points_cloud, lidar_to_map, execution_time, velocity, yaw,
                                  other_true_pos_eng):
    eps_candidates = [0.3,
                      0.4]  # list(np.arange(0.1, 4.5, 0.1))  # min distance between two points to consider them neighbours
    min_samples_candidates = [2, 3]  # list(np.arange(1, 10, 1))  # min number of neighbours to count as a core point

    for eps in eps_candidates:
        for min_samples in min_samples_candidates:
            # build a dbscan model -> make prediction -> evaluate results -> save results
            filtered_points_cloud[:, 2] = 0
            dbscan_candidate_model = DBSCAN(eps=eps, min_samples=min_samples).fit(filtered_points_cloud)
            predicted_other_car_location, err = run_ONE_dbscan_params_experiment(dbscan_candidate_model, lidar_to_map,
                                                                                 execution_time,
                                                                                 velocity, other_true_pos_eng)
            # show_clusters(dbscan_candidate_model, other_true_pos_eng[:2], eps, min_samples, filtered_points_cloud)
            evaluate_and_save_dbscan_results(dbscan_candidate_model, predicted_other_car_location, err, yaw, velocity)
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
    other_car_eng_true_position = other_car_eng_true_position[:2]  # we dont care about Z-axis then we can remove it
    car2_eng_position_prediction, smallest_difference, = None, np.inf
    for eng_global_centroid in eng_global_centroids:
        # we dont care about Z-axis then we can remove it:
        eng_global_centroid = eng_global_centroid[:2]
        is_close_enough, difference = check_proximity(eng_global_centroid, other_car_eng_true_position)
        if is_close_enough and difference < smallest_difference:
            car2_eng_position_prediction, smallest_difference = eng_global_centroid, difference

    return car2_eng_position_prediction, smallest_difference
