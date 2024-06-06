import math
import plots_utils
import turn_helper
from spline_utils import PathSpline
import numpy as np
import airsim
import time
import pickle
import spatial_utils
import path_control
from pidf_controller import PidfControl
import struct
import csv
import os


def following_loop(client, spline_obj=None, execution_time=None, curr_vel=None, transition_matrix=None,moving_car_name="Car1"):
    data_dest = os.path.join(os.getcwd(), 'recordings')
    os.makedirs(data_dest, exist_ok=True)
    save_data = False

    # In case the file is run as standalone, no mapping procedure was made.
    # Hence, we need to build the spline path out of known cone locations.
    if spline_obj is None:
        # Airsim always spawns at zero. Must compensate using "playerstart" location in unreal:
        starting_x = 12.1
        starting_y = 18.7
        spline_origin_x = 12
        spline_origin_y = 25
        x_offset = spline_origin_x - starting_x
        y_offset = spline_origin_y - starting_y

        # Define spline
        x = np.array(
            [-2.00, 0.00, 0.00, 0.00, -5.00, -13.0, -21.0, -27.0, -32.0, -38.0, -47.0, -55.0, -53.0, -40.0, -25.0,
             -23.0, -37.0, -34.0, -20.0, -8.0])
        y = np.array(
            [6.0, -7.0, -19.0, -34.0, -46.0, -51.0, -54.0, -59.0, -68.0, -74.0, -75.0, -68.0, -54.0, -39.0, -23.0,
             -8.00, 6.00, 21.00, 23.00, 15.00])
        x += x_offset
        y += y_offset
        y *= -1
        spline_obj = PathSpline(x, y)
        spline_obj.generate_spline(0.1, smoothing=1)
        spatial_utils.set_airsim_pose(client, [0.0, 0.0], [90.0, 0, 0])

    # Define Stanley-method parameters:
    follow_handler = path_control.StanleyFollower(spline_obj)
    follow_handler.k_vel *= 2.0  # Arbitrary
    follow_handler.max_velocity = 8.0  # m/s
    follow_handler.min_velocity = 3.0  # m/s
    follow_handler.lookahead = 5.0  # meters
    follow_handler.k_steer = 10.0  # Stanley steering coefficient  # shahar changes 2 to be 10

    # Open access to shared memory blocks:
    shmem_active, shmem_setpoint, shmem_output = path_control.SteeringProcManager.retrieve_shared_memories()

    # Define speed controller:
    speed_controller = PidfControl(0.01)
    speed_controller.set_pidf(0.01, 0.01, 0.0, 0.043)
    speed_controller.set_extrema(min_setpoint=0.01, max_integral=0.01)
    speed_controller.alpha = 0.01

    # Initialize loop variables:
    loop_trigger = False
    leaving_distance = 10.0
    entering_distance = 4.0

    car_controls = airsim.CarControls(moving_car_name)
    car_data = np.ndarray(shape=(0, 8))

    start_time = time.perf_counter()
    last_iteration = start_time
    sample_time = 0.01

    ###################################################################################

    current_position_lst = []
    start_time_lst = time.perf_counter()
    max_run_time = 60
    turn_completed = False

    ###################################################################################

    target_point = [spline_obj.xi[-1] , spline_obj.yi[-1]]
    first_position = [client.simGetVehiclePose().position.x_val , client.simGetVehiclePose().position.y_val ]
    initial_yaw = spatial_utils.extract_rotation_from_airsim(
        client.simGetVehiclePose(moving_car_name).orientation)[0]
    while not turn_completed:
        now = time.perf_counter()

        vehicle_pose = client.simGetVehiclePose(moving_car_name)
        vehicle_to_map = spatial_utils.tf_matrix_from_airsim_object(vehicle_pose)
        car_state = client.getCarState()
        curr_vel = car_state.speed
        curr_pos, curr_rot = spatial_utils.extract_pose_from_airsim(vehicle_pose)
        ###############################################################################################

        rot_airsim = spatial_utils.extract_rotation_from_airsim(vehicle_pose.orientation)       # yaw, pitch, roll
        current_position_airsim = [vehicle_pose.position.x_val, vehicle_pose.position.y_val, vehicle_pose.position.z_val]
        current_position_global = turn_helper.airsim_point_to_global(current_position_airsim,execution_time=execution_time, curr_vel=curr_vel,
                                                                     transition_matrix=transition_matrix)

        distance_from_target_point = math.sqrt((current_position_global[0] - target_point[0]) ** 2 +
                                                (current_position_global[1] - target_point[1]) ** 2)

        if now - start_time_lst >= max_run_time: ## if its miss the distance from the point the car will stop after max_run_time
            plots_utils.plot_the_car_path(current_position_lst)
            return current_position_lst


        ##############################################################################
        curr_heading = np.deg2rad(curr_rot[0])
        """global positions"""
        print(f"vechicle pose {client.simGetVehiclePose(moving_car_name).position}")
        print(f"position global {current_position_global}")
        print(f"position airsim {current_position_airsim}")
        print(f"object pose: {client.simGetObjectPose('Car1').position}")
        print(f'heading: {curr_rot[0]}')
        print(f'Steer: {client.getCarControls("Car1").steering}')
        print(f"distance_from_target_point = {distance_from_target_point}")

        current_position_lst.append(current_position_airsim)
        if distance_from_target_point < 5.0 and \
                ((-0.25 <= client.getCarControls("Car1").steering <= 0.25) or   # todo: maybe no need for steer
                 (88.0<abs(spatial_utils.extract_rotation_from_airsim(vehicle_pose.orientation)[0])<=92.0) or   # turn left and right from yaw 0/180
                 (-1<spatial_utils.extract_rotation_from_airsim(vehicle_pose.orientation)[0]<=1 ) or        # turn left from yaw 90
                (178.0<abs(spatial_utils.extract_rotation_from_airsim(vehicle_pose.orientation)[0])<=182.0 )): # turn right from yaw 90

            # let the car drive in straight line and low speed for few seconds
            car_controls.throttle = 0.2
            car_controls.steering = 0.0
            client.setCarControls(car_controls)

            t = time.perf_counter()
            while True:  ## keep drive staright for 5 seconds after we finished the turn

                elapsed_time = time.perf_counter() - t
                if elapsed_time > 5:
                    print("5 seconds have passed. Exiting the loop.")
                    break

                vehicle_pose = client.simGetVehiclePose(moving_car_name)
                current_position_airsim = [vehicle_pose.position.x_val,vehicle_pose.position.y_val, vehicle_pose.position.z_val]

                current_position_global = turn_helper.airsim_point_to_global(current_position_airsim, execution_time=execution_time,
                                                                             curr_vel=curr_vel,
                                                                             transition_matrix=transition_matrix)
                current_position_lst.append(current_position_airsim)

            turn_completed = True

        # else: ## distance_from_target_point > 0.5:
        if turn_completed:
            break
        desired_speed, desired_steer = follow_handler.calc_ref_speed_steering(current_position_global, curr_vel, curr_heading)
        # Close a control loop over the throttle/speed of the vehicle:
        # throttle_command = speed_controller.velocity_control(desired_speed, 0, curr_vel)
        # throttle_command = np.clip(throttle_command, 0.0, 0.4)
        # car_state = client.getCarState()
        # curr_vel = car_state.speed
        # print(curr_vel)
        #desired_steer *= -1
        print(f"desired steer :{desired_steer}")
        desired_steer /= follow_handler.max_steering  # Convert range to [-1, 1]
        desired_steer = np.clip(desired_steer, -1, 1)  # maybe we need to play with these values

        shmem_setpoint.buf[:8] = struct.pack('d', desired_steer)
        real_steer = struct.unpack('d', shmem_output.buf[:8])[0]

        #car_controls.throttle = throttle_command

        car_controls.throttle = 0.4

        car_controls.steering = real_steer
        client.setCarControls(car_controls)

    plots_utils.plot_the_car_path(current_position_lst)
    plots_utils.combine_plot(spline_obj.xi, spline_obj.yi, current_position_lst)
    return current_position_lst


    """
    # we dont use this section
    # if save_data:
        #     car_data = np.append(car_data,
        #                          [[curr_pos[0], curr_pos[1], curr_rot[0],
        #                            desired_speed, car_state.speed,
        #                            desired_steer, real_steer, throttle_command]],
        #                          axis=0)




    pickling_objects = {'path': follow_handler.path, 'car_data': car_data}
    if save_data:
        with open(os.path.join(data_dest, 'following_session.pickle'), 'wb') as pickle_file:
            pickle.dump(pickling_objects, pickle_file)
        print('saved pickle data')
        with open('car_data.csv', 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['x', 'y', 'heading', 'v_desired', 'v_delivered',
                             's_desired', 's_delivered', 'throttle'])
            writer.writerows(car_data)
        print('saved csv data')
    return pickling_objects, car_data
    """



if __name__ == '__main__':
    # Create an Airsim client:
    airsim_client = airsim.CarClient()
    airsim_client.confirmConnection()
    airsim_client.enableApiControl(True)

    # Instantiate a shared-memory manager and run the loop:
    steering_procedure_manager = path_control.SteeringProcManager()
    following_loop(airsim_client)

    # Done! stop vehicle:
    steering_procedure_manager.terminate_steering_procedure()
    vehicle_controls = airsim_client.getCarControls()
    vehicle_controls.throttle = 0.0
    vehicle_controls.brake = 1.0
    airsim_client.setCarControls(vehicle_controls)
