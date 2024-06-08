import airsim
import numpy as np
import bezier
import path_following
import plots_utils
import spline_utils
import spatial_utils
import math
from turn_consts import *
from plots_utils import *

def airsim_point_to_global_full_version(airsim_point, execution_time, curr_vel, transition_matrix):
    # todo: explain why we didnt use it and save for later use
    airsim_point_copy = airsim_point.copy()
    centroid_eng, dump = spatial_utils.convert_eng_airsim(airsim_point_copy, [0, 0, 0])
    centroid_eng[0] -= execution_time * curr_vel * 2.0  # Compensate for sensor sync
    centroid_lidar = np.append(centroid_eng, 1)
    global_point = np.matmul(transition_matrix, centroid_lidar)[:3]
    return global_point

def airsim_point_to_global(airsim_point):
    airsim_point_copy = airsim_point.copy()
    centroid_eng, dump = spatial_utils.convert_eng_airsim(airsim_point_copy, BASE_ROTATION)
    return centroid_eng

def get_points_for_bezier_curve(client, moving_car_name, initial_yaw, direction):
    # the destination and control points in this function are from the vehicle point of view ( assumes start at 0,0)
    vehicle_pose = client.simGetVehiclePose(moving_car_name)
    yaw_is_0 = ZERO_YAW_LOW_BOUNDREY <= abs(initial_yaw) <= ZERO_YAW_HIGH_BOUNDERY
    yaw_is_90 = NINETY_YAW_LOW_BOUNDREY <= initial_yaw <= NINETY_YAW_HIGH_BOUNDERY
    yaw_is_180 = ONE_HUNDRED_EIGHTY_YAW_LOW_BOUNDREY <= initial_yaw <= ONE_HUNDRED_EIGHTY_YAW_HIGH_BOUNDERY
    # todo: later in this file there are method that need to implement and replace the current implementation
    #  of the current [vehicke_pose.position.x_val + 20 and so on, for example the value 20 shouldnt be called 20
    if yaw_is_0:
        if direction == TURN_DIRECTION_RIGHT:
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
    elif yaw_is_180:
        if direction == TURN_DIRECTION_RIGHT:
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
    elif yaw_is_90:  # yaw = 90
        if direction == TURN_DIRECTION_RIGHT:
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
        if direction == TURN_DIRECTION_RIGHT:
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

    destination_point_global = airsim_point_to_global(destination_point_airsim)
    destination_point_airsim = np.array([destination_point_airsim[0], destination_point_airsim[1]])
    destination_point_global = np.array([destination_point_global[0], destination_point_global[1]])
    print(f"destination from airsim : {destination_point_airsim}")
    print(f"destination from global : {destination_point_global}")

    control_point_global = airsim_point_to_global(control_point_airsim)
    control_point_global = np.array([control_point_global[0], control_point_global[1]])

    return destination_point_global, control_point_global

def set_destination_point(vehicle_pose: airsim.Pose):
    # todo: implement
    pass

def set_control_point(vehicle_pose: airsim.Pose):
    # todo: implement
    pass


def create_bezier_curve(client, car1_initial_settings_position, current_car1_settings_position, execution_time, curr_vel, transition_matrix, direction, moving_car_name):

    # Calculate the Euclidean distance
    # todo: this should be in a function (too long so hard to read)
    distance_from_car1_initial_settings_position = math.sqrt((current_car1_settings_position.x_val - car1_initial_settings_position.x_val) ** 2 +
                                                             (current_car1_settings_position.y_val - car1_initial_settings_position.y_val) ** 2 +
                                                             (current_car1_settings_position.z_val - car1_initial_settings_position.z_val) ** 2)
    vehicle_pose = client.simGetVehiclePose(moving_car_name)
    vehicle_rotation = spatial_utils.extract_rotation_from_airsim(vehicle_pose.orientation) # return  : yaw, pitch, roll
    initial_yaw = vehicle_rotation[0] # retrun the yaw
    # for start, the vehicle is moving straight a few meters
    # todo: this values are fine but they should be in a variable
    reached_start_turning_point = 2.5 <= distance_from_car1_initial_settings_position <= 2.7
    if reached_start_turning_point:
        destination_point_global, control_point_global = \
            get_points_for_bezier_curve(client, moving_car_name, initial_yaw, direction)

        start_point_airsim = [vehicle_pose.position.x_val,
                                vehicle_pose.position.y_val,
                                vehicle_pose.position.z_val]
        start_point_global = airsim_point_to_global(start_point_airsim)
        # todo: [0] and [1] is not clear, change it to x and y
        start_point_global_np = np.array([start_point_global[0],start_point_global[1]]) # needed for creating the bezier

        global_curve_points = bezier.generate_curve_points(start_point_global_np, control_point_global,
                                                           destination_point_global)
        global_curve_points = [[p[0], p[1], 0] for p in global_curve_points]  # global
        global_curve_points_open = []  # todo: find a better name
        # this way is more clear to a user that read this, he doesnt need to debug the code
        for point_as_tuple in global_curve_points:
            x = point_as_tuple[0]
            y = point_as_tuple[1]
            z = 0  # by definition
            point = [x, y, z]
            global_curve_points_open.append(point)
        global_curve_points = global_curve_points_open

        return global_curve_points
    else:
        return None

def filter_tracked_points_and_generate_spline(tracked_points):

    x = [sublist[0] for sublist in tracked_points[::2]]
    y = [sublist[1] for sublist in tracked_points[::2]]

    spline_obj = spline_utils.PathSpline(x, y)
    spline_obj.generate_spline(amount=0.1, meters=True, smoothing=1, summation=len(x))
    print('Done!')

    plots_utils.plot_the_spline(spline_obj.xi, -1 * spline_obj.yi)
    return spline_obj
