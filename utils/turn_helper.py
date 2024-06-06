import numpy as np
import bezier
import path_following
import spline_utils
import spatial_utils
import math


def airsim_point_to_global(airsim_point, execution_time, curr_vel, transition_matrix):
    airsim_point_copy = airsim_point.copy()
    centroid_eng, dump = spatial_utils.convert_eng_airsim(airsim_point_copy, [0, 0, 0])
    # centroid_eng[0] -= execution_time * curr_vel * 2.0  # Compensate for sensor sync
    # centroid_lidar = np.append(centroid_eng, 1)
    # global_point = np.matmul(transition_matrix, centroid_lidar)[:3]
    return centroid_eng

def get_points_for_bezier_curve(client, moving_car_name, initial_yaw, direction, execution_time, curr_vel, transition_matrix):
    # the destination and control points in this function are from the vehicle point of view ( assumes start at 0,0)
    vehicle_pose = client.simGetVehiclePose(moving_car_name)
    if -1 <= abs(initial_yaw) <= 1:  # yaw = 0
        if direction == 'right':
            destination_point_airsim = [vehicle_pose.position.x_val + 20,
                                                   vehicle_pose.position.y_val + 10,
                                                   vehicle_pose.position.z_val]
            control_point_airsim = [vehicle_pose.position.x_val + 20,
                                               vehicle_pose.position.y_val,
                                               vehicle_pose.position.z_val]

        else:  # direction == left
            destination_point_airsim = [vehicle_pose.position.x_val + 20,
                                                   vehicle_pose.position.y_val - 10,
                                                    vehicle_pose.position.z_val]
            control_point_airsim = [vehicle_pose.position.x_val + 20,
                                               vehicle_pose.position.y_val,
                                               vehicle_pose.position.z_val]
    elif 179 <= abs(initial_yaw) <= 181:  # yaw = 180
        if direction == 'right':
            destination_point_airsim = [vehicle_pose.position.x_val - 20,
                                                   vehicle_pose.position.y_val - 10,
                                                   vehicle_pose.position.z_val]
            control_point_airsim = [vehicle_pose.position.x_val - 20,
                                               vehicle_pose.position.y_val,
                                               vehicle_pose.position.z_val]

        else:  # direction == left
            destination_point_airsim = [vehicle_pose.position.x_val - 20,
                                                   vehicle_pose.position.y_val + 10,
                                                   vehicle_pose.position.z_val]
            control_point_airsim = [vehicle_pose.position.x_val - 20,
                                               vehicle_pose.position.y_val,
                                               vehicle_pose.position.z_val]
    elif 89 <= initial_yaw <= 91:  # yaw = 90
        if direction == 'right':
            destination_point_airsim = [
                vehicle_pose.position.x_val - 10,
                vehicle_pose.position.y_val + 20,
                vehicle_pose.position.z_val]

            control_point_airsim = [vehicle_pose.position.x_val,
                                               vehicle_pose.position.y_val+20,
                                               vehicle_pose.position.z_val]

        else:  # direction == left
            destination_point_airsim = [
                vehicle_pose.position.x_val + 10,
                vehicle_pose.position.y_val + 20,
                vehicle_pose.position.z_val]

            control_point_airsim = [vehicle_pose.position.x_val,
                                               vehicle_pose.position.y_val+20,
                                               vehicle_pose.position.z_val]
    else:       # yaw -90 or 270
        if direction == 'right':
            destination_point_airsim = [
                vehicle_pose.position.x_val + 10,
                vehicle_pose.position.y_val - 15,
                vehicle_pose.position.z_val]

            control_point_airsim = [vehicle_pose.position.x_val,
                                               vehicle_pose.position.y_val - 15,
                                               vehicle_pose.position.z_val]

        else:  # direction == left
            destination_point_airsim = [
                vehicle_pose.position.x_val - 10,
                vehicle_pose.position.y_val - 20,
                vehicle_pose.position.z_val]

            control_point_airsim = [vehicle_pose.position.x_val,
                                               vehicle_pose.position.y_val - 20,
                                               vehicle_pose.position.z_val]


    destination_point_global = airsim_point_to_global(destination_point_airsim, execution_time, curr_vel, transition_matrix)

    control_point_global = airsim_point_to_global(control_point_airsim, execution_time, curr_vel,transition_matrix)


    destination_point_global = np.array([destination_point_global[0], destination_point_global[1]])
    control_point_global = np.array([control_point_global[0], control_point_global[1]])

    destination_point_airsim = np.array([destination_point_airsim[0], destination_point_airsim[1]])

    print(f"destination from airsim : {destination_point_airsim}")
    print(f"destination from global : {destination_point_global}")
    return destination_point_global, control_point_global

def create_bezier_curve(client, car1_initial_settings_position, current_car1_settings_position, execution_time, curr_vel, transition_matrix, direction, moving_car_name):

    # Calculate the Euclidean distance
    distance_from_car1_initial_settings_position = math.sqrt((current_car1_settings_position.x_val - car1_initial_settings_position.x_val) ** 2 +
                                                             (current_car1_settings_position.y_val - car1_initial_settings_position.y_val) ** 2 +
                                                             (current_car1_settings_position.z_val - car1_initial_settings_position.z_val) ** 2)
    vehicle_pose = client.simGetVehiclePose(moving_car_name)
    vehicle_rotation = spatial_utils.extract_rotation_from_airsim(vehicle_pose.orientation) # return  : yaw, pitch, roll
    initial_yaw = vehicle_rotation[0] # retrun the yaw
    # for start, the vehicle is moving straight a few meters
    if 2.5 <= distance_from_car1_initial_settings_position <= 2.7:
        destination_point_global, control_point_global = get_points_for_bezier_curve(client, moving_car_name,
                                                                                     initial_yaw, direction,
                                                                                     execution_time, curr_vel,
                                                                                     transition_matrix)

        start_point_airsim = [vehicle_pose.position.x_val,
                                vehicle_pose.position.y_val,
                                vehicle_pose.position.z_val]
        start_point_global = airsim_point_to_global(start_point_airsim, execution_time, curr_vel, transition_matrix)
        start_point_global_np = np.array([start_point_global[0],start_point_global[1]]) # needed for creating the bezier

        global_curve_points = bezier.generate_curve_points(start_point_global_np, control_point_global,
                                                           destination_point_global)
        global_curve_points = [[p[0], p[1], 0] for p in global_curve_points]  # global
        return global_curve_points
    else:
        return None


