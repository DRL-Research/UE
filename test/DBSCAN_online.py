#!/usr/bin/env python
import numpy as np
import time
import pickle
from sklearn.cluster import DBSCAN
import airsim
from perception import dbscan_utils
from spatial_utils_v1 import set_initial_pose


def aggregate_detections(airsim_client, iterations=2):
    pointcloud = np.array([])
    for curr_iter in range(iterations):
        lidarData = airsim_client.getLidarData()
        pointcloud = np.append(pointcloud, np.array(lidarData.point_cloud, dtype=np.dtype('f4')))
    return np.reshape(pointcloud, (int(pointcloud.shape[0] / 3), 3))


if __name__ == '__main__':

    # Airsim is stupid, always spawns at zero. Must compensate using "playerstart" in unreal:
    starting_x = 10.0
    starting_y = 20.0

    # connect to the AirSim simulator
    client = airsim.CarClient()
    client.confirmConnection()
    # client.enableApiControl(True)
    set_initial_pose(client, [0.0, 0.0], -90.0)
    time.sleep(2.0)

    all_segments = []
    all_centroids = []
    start_time = time.time()
    idx = 0
    while time.time() - start_time < 30:
        # Constant throttle 0.1 (speed in the future)
        # trackers over centroids
        # separate into left/right
        # control law for steering
        # add color detection for "shufuni"
        point_cloud = aggregate_detections(client)  # Airsim's stupid lidar implementation
        point_cloud[:, 2] *= -1  # Z is reversed in Airsim because of flying convention
        filtered_pc = dbscan_utils.filter_cloud(point_cloud, 5.0, 20.0, -0.5, 1.0)
        # after it works, dont forget to throw z away

        if filtered_pc.size > 0:
            db = DBSCAN(eps=0.3, min_samples=3).fit(filtered_pc)
            curr_segments, curr_centroids = dbscan_utils.collate_segmentation(db, 0.5)

            if curr_segments:
                all_segments.append(curr_segments)
                all_centroids.append(curr_centroids)

            print(idx)
            idx += 1
            time.sleep(0.1)

    pickle_data = {}
    pickle_data['all_segments'] = all_segments
    pickle_data['all_centroids'] = all_centroids
    with open('dbscan_session.pickle', 'wb') as pickle_file:
        pickle.dump(pickle_data, pickle_file)
    print('saved pickle data')
