import numpy as np
import time
import pickle
from sklearn.cluster import DBSCAN
import airsim
import dbscan_utils
import spatial_utils
import tracker_utils
import camera_utils
import path_control
import os
import cv2
import struct
import bezier
import spline_utils
import path_following
import turn_helper
from turn_consts import *
from car_mapping import *
from setup_simulation import *

decimation = 30e9  # Used to save an output image every X iterations.


def mapping_loop(client, moving_car_name='Car1', setup_manager: SetupManager = None):
    global decimation
    image_dest = os.path.join(os.getcwd(), 'images')
    data_dest = os.path.join(os.getcwd(), 'recordings')
    os.makedirs(image_dest, exist_ok=True)
    os.makedirs(data_dest, exist_ok=True)
    save_data = False

    # Constant transform matrices:
    # Notating A_to_B means that taking a vector in frame A and left-multiplying by the matrix
    # will result in the same point represented in frame B, even though the definition of the deltas
    # within the parentheses describe the transformation from B to A.
    lidar_pos = [2, 0, -0.1]
    lidar_rot = [0, 0, 0]
    left_cam = camera_utils.AirsimCamera(640, 360, 70, [2, -0.5, -0.5], [-40.0, -10.0, 0])
    right_cam = camera_utils.AirsimCamera(640, 360, 70, [2, 0.5, -0.5], [40.0, -10.0, 0])
    lidar_to_vehicle = spatial_utils.tf_matrix_from_airsim_pose(lidar_pos, lidar_rot)
    left_cam_to_vehicle = left_cam.tf_matrix
    right_cam_to_vehicle = right_cam.tf_matrix
    lidar_to_left_cam = np.matmul(np.linalg.inv(left_cam_to_vehicle), lidar_to_vehicle)
    lidar_to_right_cam = np.matmul(np.linalg.inv(right_cam_to_vehicle), lidar_to_vehicle)

    # Define pure pursuit parameters
    pursuit_follower = path_control.PursuitFollower(2.0, 6.0)
    pursuit_follower.k_steer = 0.5

    # Open access to shared memory blocks:
    shmem_active, shmem_setpoint, shmem_output = path_control.SteeringProcManager.retrieve_shared_memories()

    # Initialize vehicle starting point
    # spatial_utils.set_airsim_pose(client, [0.0, 0.0], [180.0, 0, 0])
    time.sleep(1.0)
    car_controls = airsim.CarControls()
    car_controls.throttle = 0.2
    client.setCarControls(car_controls, vehicle_name=moving_car_name)
    start_time = time.perf_counter()
    last_iteration = start_time
    sample_time = 0.1
    execution_time = 0.0
    initial_car_position = spatial_utils.get_car_settings_position(client, moving_car_name)
    initial_yaw, vehicle_pose, curr_vel, vehicle_to_map = None, None, None, None
    distance_from_initial_position = np.inf
    reached_start_turning_point = DISTANCE_BEFORE_START_TURNING > distance_from_initial_position
    while not reached_start_turning_point:
        vehicle_pose = client.simGetVehiclePose(moving_car_name)
        vehicle_to_map = spatial_utils.tf_matrix_from_airsim_object(vehicle_pose)
        car_state = client.getCarState()
        curr_vel = car_state.speed
        current_car_position = spatial_utils.get_car_settings_position(client, moving_car_name)
        distance_from_initial_position = spatial_utils.calculate_distance_in_2d_from_3dvector(current_car_position,
                                                                                              initial_car_position)
        vehicle_rotation = spatial_utils.extract_rotation_from_airsim(vehicle_pose.orientation)  # return yaw,pitch,roll
        initial_yaw = vehicle_rotation[0]  # return the yaw
        # for start, the vehicle is moving straight a few meters
        reached_start_turning_point = DISTANCE_BEFORE_START_TURNING <= distance_from_initial_position

    try:
        tracked_points_bezier = turn_helper.create_bezier_curve(client, initial_yaw, vehicle_pose,
                                                                direction="left",
                                                                moving_car_name=moving_car_name)
        return tracked_points_bezier, execution_time, curr_vel, vehicle_to_map
    except:
        raise Exception("Turn Mapping Problem")
