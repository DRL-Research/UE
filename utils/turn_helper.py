import numpy as np
import bezier
import path_following
import spline_utils
import spatial_utils
import time
import math


def airsim_to_global(airsim_points, execution_time, curr_vel, lidar_to_map):
    global_points = []
    for airsim_point in airsim_points:
        centroid_eng, dump = spatial_utils.convert_eng_airsim(airsim_point, [0, 0, 0])
        centroid_eng[0] -= execution_time * curr_vel * 2.0  # Compensate for sensor sync
        centroid_lidar = np.append(centroid_eng, 1)
        global_point = np.matmul(lidar_to_map, centroid_lidar)[:3]
        global_points.append(global_point)
    return global_points


def airsim_point_to_global(airsim_point, execution_time, curr_vel, lidar_to_map):
    centroid_eng, dump = spatial_utils.convert_eng_airsim(airsim_point, [0, 0, 0])
    centroid_eng[0] -= execution_time * curr_vel * 2.0  # Compensate for sensor sync
    centroid_lidar = np.append(centroid_eng, 1)
    global_point = np.matmul(lidar_to_map, centroid_lidar)[:3]
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


def retrive_check_points_during_turn(client, execution_time, curr_vel, lidar_to_map):
    global_check_points = []
    total_duration = 30
    print_interval = 2
    start_time = time.time()

    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time

        # Check if the total duration has been reached
        if elapsed_time >= total_duration:
            break

        if elapsed_time % print_interval == 0:
            airsim_vehicle_pose = client.simGetVehiclePose()
            x_airsim = airsim_vehicle_pose.position.x_val
            y_airsim = airsim_vehicle_pose.position.y_val
            z_airsim = airsim_vehicle_pose.position.z_val
            point_airsim = [x_airsim, y_airsim, z_airsim]
            vehicle_global_pose = airsim_point_to_global(point_airsim, execution_time, curr_vel, lidar_to_map)
            global_check_points.append(vehicle_global_pose)

        # Optionally, you can add a small delay to avoid high CPU usage in the loop
        time.sleep(0.1)

    return global_check_points


def get_points_for_bezier_curve(current_car1_settings_position, execution_time, curr_vel, lidar_to_map, direction):

    if direction == 'left':
        # destination_point_settings_position = [current_car1_settings_position.x_val - 25, current_car1_settings_position.y_val - 17.5,
        #                                current_car1_settings_position.z_val]
        # control_point_settings_position = [current_car1_settings_position.x_val - 5, current_car1_settings_position.y_val - 17.5,
        #                                    current_car1_settings_position.z_val]
        destination_point_settings_position = [current_car1_settings_position.x_val + 25, current_car1_settings_position.y_val - 17.5,
                                       current_car1_settings_position.z_val]
        control_point_settings_position = [current_car1_settings_position.x_val + 5, current_car1_settings_position.y_val - 17.5,
                                           current_car1_settings_position.z_val]
    else:  # direction == right
        destination_point_settings_position = [current_car1_settings_position.x_val + 15, current_car1_settings_position.y_val - 17.5,
                                       current_car1_settings_position.z_val]
        control_point_settings_position = [current_car1_settings_position.x_val - 5, current_car1_settings_position.y_val - 17.5,
                                           current_car1_settings_position.z_val]

    # destination_point_airsim = get_other_position_ref_to_self(current_car1_settings_position,
    #                                                           destination_point_settings_position)  # the position of car2 ref to car 1 #todo airsim position ???
    # destination_point_global = airsim_point_to_global(destination_point_airsim, execution_time, curr_vel, lidar_to_map)
    # control_point_airsim = get_other_position_ref_to_self(current_car1_settings_position,
    #                                                       control_point_settings_position)  # the position of car3 ref to car 1
    #
    # control_point_global = airsim_point_to_global(control_point_airsim, execution_time, curr_vel, lidar_to_map)


    destination_point_global = airsim_point_to_global(destination_point_settings_position, execution_time, curr_vel, lidar_to_map)

    control_point_global = airsim_point_to_global(control_point_settings_position, execution_time, curr_vel, lidar_to_map)

    destination_x = destination_point_global[0]
    destination_y = destination_point_global[1]
    control_x = control_point_global[0]
    control_y = control_point_global[1]

    destination_point_global = np.array([destination_x, destination_y])
    control_point_global = np.array([control_x, control_y])

    # destination_point_global = np.array([15, -20])
    # control_point_global = np.array([35, -20])

    return destination_point_global, control_point_global


def create_bezier_curve(current_car1_settings_position, execution_time, curr_vel, lidar_to_map, car1_initial_settings_position,global_position, direction,
                        car1_airsim_position, epsilon):

    # Calculate the Euclidean distance
    distance_from_car1_initial_settings_position = math.sqrt((current_car1_settings_position.x_val - car1_initial_settings_position.x_val) ** 2 +
                                                             (current_car1_settings_position.y_val - car1_initial_settings_position.y_val) ** 2 +
                                                             (current_car1_settings_position.z_val - car1_initial_settings_position.z_val) ** 2)


    if 2.5 <= distance_from_car1_initial_settings_position <= 2.7:
        destination_point_global, control_point_global = get_points_for_bezier_curve(current_car1_settings_position,
                                                                                     execution_time, curr_vel,
                                                                                     lidar_to_map, direction)
        # destination_point_global = np.array([float(global_position[0]-25),float(global_position[1]-17.5)])
        # control_point_global = np.array([float(global_position[0] - 5), float(global_position[1] - 17.5)])
        # start_point = np.array([float(global_position[0]), float(global_position[1])])  # beacause this is the start point ref to car 1

        # destination_point_global = np.array([20,10.0])
        # control_point_global = np.array([20.0,0.0])
        # start_point = np.array([2.5, 0.0])  # beacause this is the start point ref to car 1

        start_point = np.array([0.0, 0.0])  # beacause this is the start point ref to car 1
        global_curve_points = bezier.generate_curve_points(start_point, control_point_global,
                                                           destination_point_global)  # todo this is not the same coorainate maybe
        global_curve_points = [[p[0], p[1], 0] for p in global_curve_points]  # global
        return global_curve_points
    else:
        return None


