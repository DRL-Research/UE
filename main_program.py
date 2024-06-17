import os
import threading
import cone_mapping
import path_following
import airsim
import numpy as np
import plots_utils
import spline_utils
import path_control
from setup_simulation import *
from airsim_manager import AirsimManager
import time
import turn_mapping
import turn_helper


def run_for_car(moving_car_name, airsim_client, setup_manager):
    steering_procedure_manager = path_control.SteeringProcManager()

    # Detect the cones and spline points, and return their location:
    print(f'Starting on-the-fly cone mapping with constant speed and steering procedure for {moving_car_name}.')
    tracked_points, execution_time, curr_vel, transition_matrix = turn_mapping.mapping_loop(airsim_client,
                                                                                            moving_car_name,
                                                                                            setup_manager)

    print(f'Mapping complete for {moving_car_name}!')

    # Stop until spline generation is complete:
    print(f'Stopping vehicle {moving_car_name} and generating a path to follow...')
    car_controls = airsim_client.getCarControls(vehicle_name=moving_car_name)
    car_controls.throttle = 0.0
    airsim_client.setCarControls(car_controls, vehicle_name=moving_car_name)

    spline_obj = turn_helper.filter_tracked_points_and_generate_spline(tracked_points)

    # Follow the spline using Stanley's method:
    print(f'Starting variable speed spline following procedure for {moving_car_name}.')
    positions_lst = path_following.following_loop(airsim_client, spline_obj, execution_time, curr_vel,
                                                  transition_matrix, moving_car_name=moving_car_name)

    print(f'Full process complete for {moving_car_name}! Stopping vehicle.')
    car_controls.throttle = 0.0
    car_controls.brake = 1.0
    airsim_client.setCarControls(car_controls, vehicle_name=moving_car_name)
    steering_procedure_manager.terminate_steering_procedure()

if __name__ == '__main__':
    # Perform the setup once
    setup_manager = SetupManager()
    setup_manager.extract_cars()
    setup_manager.enableApiCarsControl()
    airsim_client = setup_manager.airsim_client
    time.sleep(1.0)

    steering_procedure_manager = path_control.SteeringProcManager()
    airsim_client.confirmConnection()

    airsim_manager = AirsimManager(airsim_client, setup_manager)
    print("ID of process running main program: {}".format(os.getpid()))

    print("Main thread name: {}".format(threading.current_thread().name))

    """
    option 1 - works
    """


    # Create and start a thread for each car

    thread1 = threading.Thread(target=run_for_car, args=("Car2", airsim_client, setup_manager))
    thread2 = threading.Thread(target=run_for_car, args=("Car4", airsim_client, setup_manager))

    thread1.start()
    time.sleep(0.1)    ## needed so the threads will not access the buffer simultaneously - something with tornado package
    thread2.start()

    # Wait for all threads to finish
    thread1.join()
    thread2.join()
    """
      option 2
    """
    # # Create a thread pool executor
    # with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
    #     # Submit tasks to the thread pool
    #     futures = [executor.submit(run_for_car, car_name, airsim_client, setup_manager) for car_name in
    #                moving_car_names]
    #
    #     # Wait for all tasks to complete
    #     concurrent.futures.wait(futures)

    print('All cars have completed their tasks.')


