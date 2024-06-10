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
    yaw_is_180 = ONE_EIGHTY_YAW_LOW_BOUNDREY <= initial_yaw <= ONE_EIGHTY_YAW_HIGH_BOUNDERY

    if yaw_is_0:
        destination_point_airsim, control_point_airsim = calculate_points_for_yaw_0(vehicle_pose, direction)
    elif yaw_is_180:
        destination_point_airsim, control_point_airsim = calculate_points_for_yaw_180(vehicle_pose, direction)
    elif yaw_is_90:
        destination_point_airsim, control_point_airsim = calculate_points_for_yaw_90(vehicle_pose, direction)
    else:       # yaw 270 (-90)
        destination_point_airsim, control_point_airsim = calculate_points_for_yaw_270(vehicle_pose, direction)

    destination_point_global = airsim_point_to_global(destination_point_airsim)
    destination_point_airsim_x, destination_point_airsim_y = destination_point_airsim[0], destination_point_airsim[1]   # just for monitoring

    destination_point_airsim = np.array([destination_point_airsim_x, destination_point_airsim_y])   # no need - just for monitoring

    destination_point_global_x = destination_point_global[0]
    destination_point_global_y = destination_point_global[1]
    destination_point_global = np.array([destination_point_global_x, destination_point_global_y])
    print(f"destination from airsim : {destination_point_airsim}")
    print(f"destination from global : {destination_point_global}")


    control_point_global = airsim_point_to_global(control_point_airsim)
    control_point_global_x = control_point_global[0]
    control_point_global_y = control_point_global[1]
    control_point_global = np.array([control_point_global_x, control_point_global_y])

    return destination_point_global, control_point_global


def calculate_points_for_yaw_0(vehicle_pose, direction):
    if direction == TURN_DIRECTION_RIGHT:
        destination_point = [vehicle_pose.position.x_val + FORWARD_DISTANCE_RIGHT_TURN,
                             vehicle_pose.position.y_val + SIDE_DISTANCE_RIGHT_TURN,
                             vehicle_pose.position.z_val]
        control_point = [vehicle_pose.position.x_val + FORWARD_DISTANCE_RIGHT_TURN,
                         vehicle_pose.position.y_val,
                         vehicle_pose.position.z_val]
    else:
        destination_point = [vehicle_pose.position.x_val + FORWARD_DISTANCE_LEFT_TURN,
                             vehicle_pose.position.y_val - SIDE_DISTANCE_LEFT_TURN,
                             vehicle_pose.position.z_val]
        control_point = [vehicle_pose.position.x_val + FORWARD_DISTANCE_LEFT_TURN,
                         vehicle_pose.position.y_val,
                         vehicle_pose.position.z_val]
    return destination_point, control_point


def calculate_points_for_yaw_180(vehicle_pose, direction):
    if direction == TURN_DIRECTION_RIGHT:
        destination_point = [vehicle_pose.position.x_val - FORWARD_DISTANCE_RIGHT_TURN,
                             vehicle_pose.position.y_val - SIDE_DISTANCE_RIGHT_TURN,
                             vehicle_pose.position.z_val]
        control_point = [vehicle_pose.position.x_val - FORWARD_DISTANCE_RIGHT_TURN,
                         vehicle_pose.position.y_val,
                         vehicle_pose.position.z_val]
    else:
        destination_point = [vehicle_pose.position.x_val - FORWARD_DISTANCE_LEFT_TURN,
                             vehicle_pose.position.y_val + SIDE_DISTANCE_LEFT_TURN,
                             vehicle_pose.position.z_val]
        control_point = [vehicle_pose.position.x_val - FORWARD_DISTANCE_LEFT_TURN,
                         vehicle_pose.position.y_val,
                         vehicle_pose.position.z_val]
    return destination_point, control_point


def calculate_points_for_yaw_90(vehicle_pose, direction):
    if direction == TURN_DIRECTION_RIGHT:
        destination_point = [vehicle_pose.position.x_val - SIDE_DISTANCE_RIGHT_TURN,
                             vehicle_pose.position.y_val + FORWARD_DISTANCE_RIGHT_TURN,
                             vehicle_pose.position.z_val]
        control_point = [vehicle_pose.position.x_val,
                         vehicle_pose.position.y_val + FORWARD_DISTANCE_RIGHT_TURN,
                         vehicle_pose.position.z_val]
    else:
        destination_point = [vehicle_pose.position.x_val + SIDE_DISTANCE_LEFT_TURN,
                             vehicle_pose.position.y_val + FORWARD_DISTANCE_LEFT_TURN,
                             vehicle_pose.position.z_val]
        control_point = [vehicle_pose.position.x_val,
                         vehicle_pose.position.y_val + FORWARD_DISTANCE_LEFT_TURN,
                         vehicle_pose.position.z_val]
    return destination_point, control_point
def calculate_points_for_yaw_270(vehicle_pose, direction):
    if direction == TURN_DIRECTION_RIGHT:
        destination_point = [vehicle_pose.position.x_val + SIDE_DISTANCE_RIGHT_TURN,
                             vehicle_pose.position.y_val - FORWARD_DISTANCE_RIGHT_TURN,
                             vehicle_pose.position.z_val]
        control_point = [vehicle_pose.position.x_val,
                         vehicle_pose.position.y_val - FORWARD_DISTANCE_RIGHT_TURN,
                         vehicle_pose.position.z_val]
    else:
        destination_point = [vehicle_pose.position.x_val - SIDE_DISTANCE_LEFT_TURN,
                             vehicle_pose.position.y_val - FORWARD_DISTANCE_LEFT_TURN,
                             vehicle_pose.position.z_val]
        control_point = [vehicle_pose.position.x_val,
                         vehicle_pose.position.y_val - FORWARD_DISTANCE_LEFT_TURN,
                         vehicle_pose.position.z_val]
    return destination_point, control_point


def create_bezier_curve(client, initial_car_position, current_car_position, execution_time, curr_vel, transition_matrix, direction, moving_car_name):

    # Calculate the Euclidean distance
    distance_from_initial_position = spatial_utils.get_distance_in_3d(current_car_position,initial_car_position)

    vehicle_pose = client.simGetVehiclePose(moving_car_name)
    vehicle_rotation = spatial_utils.extract_rotation_from_airsim(vehicle_pose.orientation) # return  : yaw, pitch, roll
    initial_yaw = vehicle_rotation[0] # retrun the yaw
    # for start, the vehicle is moving straight a few meters

    reached_start_turning_point = DISTANCE_BEFORE_START_TURNING <= distance_from_initial_position   # dont think we need upper bound <= 2.7
    if reached_start_turning_point:
        destination_point_global, control_point_global = \
            get_points_for_bezier_curve(client, moving_car_name, initial_yaw, direction)

        start_point_airsim = [vehicle_pose.position.x_val,
                                vehicle_pose.position.y_val,
                                vehicle_pose.position.z_val]
        start_point_global = airsim_point_to_global(start_point_airsim)
        start_point_x_coordinate = start_point_global[0]
        start_point_y_coordinate = start_point_global[1]
        start_point_global_np = np.array([start_point_x_coordinate,start_point_y_coordinate]) # needed for creating the bezier

        global_curve_points = bezier.generate_curve_points(start_point_global_np, control_point_global,
                                                           destination_point_global)
        global_curve_points_with_z_set_to_zero = []
        # this way is more clear to a user that read this, he doesnt need to debug the code
        for point_as_tuple in global_curve_points:
            x = point_as_tuple[0]
            y = point_as_tuple[1]
            z = 0  # by definition
            point = [x, y, z]
            global_curve_points_with_z_set_to_zero.append(point)
        global_curve_points = global_curve_points_with_z_set_to_zero

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
