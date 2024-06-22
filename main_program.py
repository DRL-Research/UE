import multiprocessing
import os
import random
import threading
import airsim
import numpy as np
import time
from turn_consts import *
import plots_utils
from setup_simulation import SetupManager
import turn_mapping
import path_following
import path_control  # Assuming this is where your SteeringProcManager is defined
from airsim_manager import AirsimManager
import turn_helper

# Function to initialize setup and return airsim_client
def initialize_setup():
    setup_manager = SetupManager()
    setup_manager.extract_cars()
    setup_manager.enableApiCarsControl()
    airsim_client = setup_manager.airsim_client
    time.sleep(0.5)
    airsim_client.confirmConnection()
    airsim_manager = AirsimManager(airsim_client, setup_manager)
    return airsim_client, airsim_manager

# Function to run the car processing
def run_for_single_car(moving_car_name):
    airsim_client, airsim_manager = initialize_setup()

    # Ensure SteeringProcManager initialization
    path_control.SteeringProcManager.create_steering_procedure()  # Initialize shared memory

    try:
        # Use retrieved shared memories
        shmem_active, shmem_setpoint, shmem_output = path_control.SteeringProcManager.retrieve_shared_memories()
        directions = [TURN_DIRECTION_STRAIGHT]  #TURN_DIRECTION_RIGHT, TURN_DIRECTION_LEFT,
        direction = random.choices(directions, k=1)[0]
        # Detect the cones and spline points, and return their location:
        print(f'Starting on-the-fly cone mapping with constant speed and steering procedure for {moving_car_name}.')
        tracked_points, execution_time, curr_vel, transition_matrix = turn_mapping.mapping_loop(airsim_client,
                                                                                                moving_car_name,
                                                                                                direction)

        print(f'Mapping complete for {moving_car_name}!')

        # Stop until spline generation is complete:
        print(f'Stopping vehicle {moving_car_name} and generating a path to follow...')
        car_controls = airsim_client.getCarControls(vehicle_name=moving_car_name)
        car_controls.throttle = 0.0
        airsim_client.setCarControls(car_controls, vehicle_name=moving_car_name)

        spline_obj = turn_helper.filter_tracked_points_and_generate_spline(tracked_points, moving_car_name)

        # Follow the spline using Stanley's method:
        print(f'Starting variable speed spline following procedure for {moving_car_name}.')
        positions_lst = path_following.following_loop(airsim_client, spline_obj, execution_time, curr_vel,transition_matrix, moving_car_name=moving_car_name)

        print(f'Full process complete for {moving_car_name}! Stopping vehicle.')
        car_controls.throttle = 0.0
        car_controls.brake = 1.0
        airsim_client.setCarControls(car_controls, vehicle_name=moving_car_name)
        return positions_lst

    finally:
        # Always clean up after use
        # path_control.SteeringProcManager.detach_shared_memories() #todo: if its not comment it fucks up the program
        # path_control.SteeringProcManager.terminate_steering_procedure()
        return positions_lst

if __name__ == '__main__':
    print("ID of process running main program: {}".format(os.getpid()))
    print("Main thread name: {}".format(threading.current_thread().name))

    # Define the car names
    # moving_car_names = ["Car2","Car3", "Car4"]  # Add more car names as needed
    moving_car_names = ["Car2", "Car4", "Car3"]#,"Car4"]  # Add more car names as needed
    all_cars_positions_list = []

    # Create a process pool
    with multiprocessing.Pool(processes=len(moving_car_names)+1) as pool:
        # Submit tasks to the process pool
        results = pool.map(run_for_single_car, moving_car_names)

    # Append results to all_cars_positions_list
    for result in results:
        all_cars_positions_list.append(result)

    ## here i want to append the positions_lst that return from run_for_car of each procces to the all_cars_positions_list
    x=all_cars_positions_list
    plots_utils.plot_vehicle_object_path(all_cars_positions_list)

    print('All cars have completed their tasks.')







