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


# if __name__ == '__main__':
#     # we should get the desired definitions of the simulator by the user: update setting.json & update airsim
#     setup_manager = SetupManager()
#     setup_manager.extract_cars()
#     setup_manager.enableApiCarsControl()
#     airsim_client = setup_manager.airsim_client
#     time.sleep(1.0)
#
#     steering_procedure_manager = path_control.SteeringProcManager()
#     moving_car_name = "Car2"
#     airsim_client.confirmConnection()
#
#     airsim_manager = AirsimManager(airsim_client, setup_manager)
#
#     # Detect the cones and spline points, and return their location:
#     print('Starting on-the-fly cone mapping with constant speed and steering procedure.')
#     #mapping_data, pursuit_points = cone_mapping.mapping_loop(airsim_client)
#     tracked_points, execution_time, curr_vel, transition_matrix = turn_mapping.mapping_loop(airsim_client,
#                                                                                             moving_car_name,
#                                                                                             setup_manager)
#
#     print('Mapping complete!')
#
#     # Stop until spline generation is complete:
#     print('Stopping vehicle and generating a path to follow...')
#     car_controls = airsim_client.getCarControls(vehicle_name=moving_car_name)
#     car_controls.throttle = 0.0
#     airsim_client.setCarControls(car_controls, vehicle_name=moving_car_name)
#
#     spline_obj = turn_helper.filter_tracked_points_and_generate_spline(tracked_points)
#
#     # Follow the spline using Stanley's method:
#     print('Starting variable speed spline following procedure.')
#     positions_lst = path_following.following_loop(airsim_client, spline_obj, execution_time, curr_vel,
#                                                   transition_matrix, moving_car_name=moving_car_name)
#     # plots_utils.combine_plot(spline_obj.xi,spline_obj.yi,positions_lst)
#
#     print('Full process complete! stopping vehicle.')
#     car_controls.throttle = 0.0
#     car_controls.brake = 1.0
#     airsim_client.setCarControls(car_controls, vehicle_name=moving_car_name)
#     steering_procedure_manager.terminate_steering_procedure()


    # # Record the start time
    # start_time = time.time()
    #
    # # Print the current time
    # current_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
    # print(f'Current time: {current_time}')
    #
    # car_state = airsim_client.getCarState()
    # position = car_state.kinematics_estimated.position
    # print(f'start vehicle position: ({position.x_val}, {position.y_val}, {position.z_val})')
    #
    # target_elapsed_time = 20
    # while True:
    #     elapsed_time = time.time() - start_time
    #     if elapsed_time >= target_elapsed_time:
    #         break
    #     # time.sleep(1)  # Sleep for 1 second before checking again
    #
    # # Print the time after the elapsed time
    # current_time_after_elapsed = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
    # print(f'Time after {target_elapsed_time} seconds: {current_time_after_elapsed}')
    # car_state = airsim_client.getCarState()
    # position = car_state.kinematics_estimated.position
    # print(f'after 20 seconds vehicle position: ({position.x_val}, {position.y_val}, {position.z_val})')
    #
    # # Stop the vehicle:
    # car_controls = airsim_client.getCarControls()
    # car_controls.throttle = 0.0
    # car_controls.brake = 1.0
    # airsim_client.setCarControls(car_controls)
    # steering_procedure_manager.terminate_steering_procedure()

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



    # List of car names to run simultaneously
    moving_car_names = ["Car1"]
    threads = []

    # Create and start a thread for each car
    for car_name in moving_car_names:
        run_for_car(car_name, airsim_client, setup_manager)
    #     thread = threading.Thread(target=run_for_car, args=(car_name, airsim_client, setup_manager))
    #     threads.append(thread)
    #     thread.start()
    #
    # # Wait for all threads to finish
    # for thread in threads:
    #     thread.join()

    #print('All cars have completed their tasks.')