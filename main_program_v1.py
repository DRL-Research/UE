import multiprocessing
import os
import random
import threading
import time
from initialization.config_v1 import *
import airsim

from initialization.config_v1 import *
import plots_utils_v1
from initialization.setup_simulation_v1 import SetupManager
from path_planning import turn_mapping_v1, path_following_v1
from utils.path_planning import path_control_v1, turn_helper_v1
from airsim_manager_v1 import AirsimManager


# Function to initialize setup and return airsim_client
def initialize_setup():
    setup_manager = SetupManager()
    time.sleep(0.5)
    airsim_manager = AirsimManager(setup_manager.airsim_client, setup_manager.cars)
    return airsim_manager


# Function to run the car processing
def run_for_single_car(moving_car_name):
    airsim_manager = initialize_setup()
    # Ensure SteeringProcManager initialization
    path_control_v1.SteeringProcManager.create_steering_procedure()  # Initialize shared memory

    try:
        # Use retrieved shared memories
        shmem_active, shmem_setpoint, shmem_output = path_control_v1.SteeringProcManager.retrieve_shared_memories()
        directions = [TURN_DIRECTION_STRAIGHT, TURN_DIRECTION_RIGHT, TURN_DIRECTION_LEFT]
        direction = random.choices(directions, k=1)[0]
        print(f'car name: {moving_car_name}, direction: {direction}')
        # Detect the cones and spline points, and return their location:
        print(f'Starting on-the-fly cone mapping with constant speed and steering procedure for {moving_car_name}.')
        tracked_points, execution_time, curr_vel, transition_matrix = turn_mapping_v1.mapping_loop(
            airsim_manager.airsim_client,
            moving_car_name,
            direction)

        print(f'Mapping complete for {moving_car_name}!')

        # Stop until spline generation is complete:
        print(f'Stopping vehicle {moving_car_name} and generating a path to follow...')
        airsim_manager.stop_car(moving_car_name, 0.1)

        spline = turn_helper_v1.filter_tracked_points_and_generate_spline(tracked_points, moving_car_name)

        # Follow the spline using Stanley's method:
        print(f'Starting variable speed spline following procedure for {moving_car_name}.')
        positions_lst = path_following_v1.following_loop(airsim_manager.airsim_client, spline, execution_time, curr_vel,
                                                         transition_matrix, moving_car_name=moving_car_name)

        print(f'Full process complete for {moving_car_name}! Stopping vehicle.')
        airsim_manager.stop_car(moving_car_name)

        return positions_lst

    finally:
        # Always clean up after use
        # path_control.SteeringProcManager.detach_shared_memories() #todo: if its not comment it fucks up the program
        # path_control.SteeringProcManager.terminate_steering_procedure()
        return positions_lst


if __name__ == '__main__':
    """ Define the cars that will participate in the simulation: """
    moving_car_names = []
    names = [CAR1_NAME, CAR2_NAME, CAR3_NAME, CAR4_NAME]
    uses = [USE_CAR1, USE_CAR2, USE_CAR3, USE_CAR4]
    for i, use in enumerate(uses):
        if use:
            moving_car_names.append(names[i])

    all_cars_positions_list = []
    # Create a process pool
    number_of_processes = len(moving_car_names) + 1
    with multiprocessing.Pool(processes=number_of_processes) as pool:
        # Submit tasks to the process pool
        results = pool.map(run_for_single_car, moving_car_names)

    # Append results to all_cars_positions_list
    for result in results:
        all_cars_positions_list.append(result)

    # here i want to append the positions_lst that return from run_for_car of each procces to the all_cars_positions_list
    if CREATE_PLOTS:
        plots_utils_v1.plot_vehicle_object_path(all_cars_positions_list)

    print('All cars have completed their tasks.')
