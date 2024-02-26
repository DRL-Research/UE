import cone_mapping
import path_following
import airsim

import plots_utils
import spline_utils
import path_control
import time


if __name__ == '__main__':
    # Create an airsim client instance:
    steering_procedure_manager = path_control.SteeringProcManager()
    airsim_client = airsim.CarClient()
    airsim_client.confirmConnection()
    airsim_client.enableApiControl(True)



    # Detect the cones and spline points, and return their location:
    print('Starting on-the-fly cone mapping with constant speed and steering procedure.')
    #mapping_data, pursuit_points = cone_mapping.mapping_loop(airsim_client)
    tracked_points,execution_time,curr_vel,transition_matrix = cone_mapping.mapping_loop(airsim_client)

    print('Mapping complete!')

    # Stop until spline generation is complete:
    print('Stopping vehicle and generating a path to follow...')
    car_controls = airsim_client.getCarControls()
    car_controls.throttle = 0.0
    airsim_client.setCarControls(car_controls)

    # Arrange the points and generate a path spline:
    #track_points = spline_utils.generate_path_points(mapping_data)
    #spline_obj = spline_utils.PathSpline(tracked_points[::2, 0], tracked_points[::2, 1])


    x = [sublist[0] for sublist in tracked_points[::2]]
    y = [sublist[1] for sublist in tracked_points[::2]]

    # check_lst = [[x[i], y[i]] for i in range(len(x))]

    spline_obj = spline_utils.PathSpline(x, y)
    spline_obj.generate_spline(amount=0.1, meters=True, smoothing=1, summation=len(x))
    print('Done!')

    plots_utils.plot_the_spline(spline_obj.xi,spline_obj.yi)

    # Follow the spline using Stanley's method:
    print('Starting variable speed spline following procedure.')
    path_following.following_loop(airsim_client, spline_obj, execution_time, curr_vel, transition_matrix)
    print('Full process complete! stopping vehicle.')

    # Done! stop vehicle:
    car_controls = airsim_client.getCarControls()
    car_controls.throttle = 0.0
    car_controls.brake = 1.0
    airsim_client.setCarControls(car_controls)
    steering_procedure_manager.terminate_steering_procedure()




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