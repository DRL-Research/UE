import numpy as np
import bezier
import path_following
import spline_utils
import spatial_utils
import time
import math


def airsim_point_to_global(airsim_point, execution_time, curr_vel, transition_matrix):
    centroid_eng, dump = spatial_utils.convert_eng_airsim(airsim_point, [0, 0, 0])
    centroid_eng[0] -= execution_time * curr_vel * 2.0  # Compensate for sensor sync
    centroid_lidar = np.append(centroid_eng, 1)
    global_point = np.matmul(transition_matrix, centroid_lidar)[:3]
    return global_point


def global_point_to_airsim(global_point, lidar_to_map):
    # Append a homogeneous coordinate for the inverse transformation
    global_point_homogeneous = np.append(global_point, 1)

    # Apply the inverse transformation
    lidar_to_map_inv = np.linalg.inv(lidar_to_map)
    # lidar_point_homogeneous = np.matmul(lidar_to_map_inv, global_point_homogeneous)
    lidar_point_homogeneous = np.linalg.solve(lidar_to_map, global_point_homogeneous)

    # Convert back to Lidar frame
    lidar_point = lidar_point_homogeneous[:3]

    # Convert Lidar point to AirSim frame
    airsim_point, dump = spatial_utils.convert_eng_airsim(lidar_point, [0, 0, 0])
    airsim_point[1] = airsim_point[1]*(-1)

    return airsim_point


def get_other_position_ref_to_self(self_pos, other_pos):
    ###############################################################################
    x = other_pos[0] - self_pos.x_val
    y = other_pos[1] - self_pos.y_val
    z = other_pos[2] - self_pos.z_val
    return [x, y, z]
    ################################################################################
    # x = other_pos.x_val - self_pos.x_val
    # y = other_pos.y_val - self_pos.y_val
    # z = other_pos.z_val - self_pos.z_val
    # return [x, y, z]


def get_points_for_bezier_curve(current_car1_settings_position, execution_time, curr_vel, transition_matrix, direction):

    if direction == 'left':
        destination_point_settings_position = [current_car1_settings_position.x_val + 10, current_car1_settings_position.y_val - 6,
                                       current_car1_settings_position.z_val]
        control_point_settings_position = [current_car1_settings_position.x_val + 10, current_car1_settings_position.y_val,
                                           current_car1_settings_position.z_val]

    else:  # direction == right
        destination_point_settings_position = [current_car1_settings_position.x_val + 10, current_car1_settings_position.y_val + 6,
                                       current_car1_settings_position.z_val]
        control_point_settings_position = [current_car1_settings_position.x_val + 10, current_car1_settings_position.y_val,
                                           current_car1_settings_position.z_val]


    destination_point_global = airsim_point_to_global(destination_point_settings_position, execution_time, curr_vel, transition_matrix)

    control_point_global = airsim_point_to_global(control_point_settings_position, execution_time, curr_vel,transition_matrix)

    destination_x = destination_point_global[0]
    destination_y = destination_point_global[1]
    control_x = control_point_global[0]
    control_y = control_point_global[1]

    destination_point_global = np.array([destination_x, destination_y])
    control_point_global = np.array([control_x, control_y])

    return destination_point_global, control_point_global


def create_bezier_curve(client, current_car1_settings_position, execution_time, curr_vel, car1_initial_settings_position, transition_matrix, direction):

    # Calculate the Euclidean distance
    distance_from_car1_initial_settings_position = math.sqrt((current_car1_settings_position.x_val - car1_initial_settings_position.x_val) ** 2 +
                                                             (current_car1_settings_position.y_val - car1_initial_settings_position.y_val) ** 2 +
                                                             (current_car1_settings_position.z_val - car1_initial_settings_position.z_val) ** 2)


    if 2.5 <= distance_from_car1_initial_settings_position <= 2.7:
        destination_point_global, control_point_global = get_points_for_bezier_curve(current_car1_settings_position,
                                                                                     execution_time, curr_vel,
                                                                                    transition_matrix,direction)

        # destination_point_global = np.array([33.5,17.5])
        # control_point_global = np.array([13.5,17.5])

        x_start = client.simGetVehiclePose().position.x_val
        y_start = client.simGetVehiclePose().position.y_val
        start_point = np.array([x_start,y_start])  # beacause this is the start point ref to car 1

        # start_point = np.array([0.0, 0.0])  # beacause this is the start point ref to car 1

        global_curve_points = bezier.generate_curve_points(start_point, control_point_global,
                                                           destination_point_global)  # todo this is not the same coorainate maybe
        global_curve_points = [[p[0], p[1], 0] for p in global_curve_points]  # global
        return global_curve_points
    else:
        return None


