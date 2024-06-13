import airsim
import numpy as np
import random
from config import CAR1_INITIAL_POSITION, CAR2_INITIAL_POSITION, CAR1_INITIAL_YAW, CAR2_INITIAL_YAW, \
    CAR1_NAME, CAR2_NAME, CAR1_DESIRED_POSITION
from setup_simulation import *


class AirsimManager:

    def __init__(self, airsim_client, setup_manager: SetupManager):

        self.airsim_client = airsim_client
        self.airsim_client.confirmConnection()  # Confirm the connection to the AirSim simulator
        self.car_name_to_offset = {}
        self.setup_manager = setup_manager

        # Set cars throttle to 1:
        for car_name, car_obj in self.setup_manager.cars.items():
            if car_obj.is_active:
                self.setup_manager.set_car_throttle_by_name(car_name)  # default throttle is 1
            # get initial positions according to settings offset
            # (later be used whenever we need to reset to initial position -> start of each epoch)
            # (arbitrary starting position in settings file as long as they are not spawned on top of each other)
            car_pos = self.airsim_client.simGetObjectPose(car_name).position
            self.car_name_to_offset[car_name] = {'x_offset': car_pos.x_val, 'y_offset': car_pos.y_val}

        self.reset_cars_to_initial_positions()

    def reset_cars_to_initial_positions(self):
        self.airsim_client.reset()
        # pick at random (car 2 goes from left/right)
        left_or_right = [-1, 1]  # random.choice([1, -1])

        car1_start_location_x = CAR1_INITIAL_POSITION[0] - self.car_name_to_offset[CAR1_NAME]['x_offset']
        car1_start_location_y = CAR1_INITIAL_POSITION[1] - self.car_name_to_offset[CAR1_NAME]['y_offset']

        car2_start_location_x = CAR2_INITIAL_POSITION[0] - self.car_name_to_offset[CAR2_NAME]['x_offset']
        car2_side = left_or_right[0] if CAR2_SIDE == 'left' else left_or_right[1]
        car2_start_location_y = car2_side * CAR2_INITIAL_POSITION[1] - self.car_name_to_offset[CAR2_NAME]['y_offset']

        car3_start_location_x = CAR3_INITIAL_POSITION[0] - self.car_name_to_offset[CAR3_NAME]['x_offset']
        car3_start_location_y = CAR3_INITIAL_POSITION[1] - self.car_name_to_offset[CAR3_NAME]['y_offset']

        car4_start_location_x = CAR4_INITIAL_POSITION[0] - self.car_name_to_offset[CAR4_NAME]['x_offset']
        car4_side = left_or_right[0] if CAR4_SIDE == 'left' else left_or_right[1]
        car4_start_location_y = car4_side * CAR4_INITIAL_POSITION[1] - self.car_name_to_offset[CAR4_NAME]['y_offset']

        car1_start_yaw = CAR1_INITIAL_YAW
        car2_start_yaw = CAR2_INITIAL_YAW
        car3_start_yaw = CAR3_INITIAL_YAW
        car4_start_yaw = CAR4_INITIAL_YAW

        # Set the reference_position for Car1 and Car2 (Do not change this code)
        reference_position = airsim.Pose(airsim.Vector3r(0.0, 0, -1), airsim.Quaternionr(0, 0.0, 0.0, 1.0))
        for car_name in self.setup_manager.cars:
            self.airsim_client.simSetVehiclePose(reference_position, True, car_name)

        # Set initial position and yaw of Car1
        self.set_initial_position_and_yaw(car_name=CAR1_NAME, start_location_x=car1_start_location_x,
                                          start_location_y=car1_start_location_y, car_start_yaw=car1_start_yaw)
        # Set initial position and yaw of Car2
        self.set_initial_position_and_yaw(car_name=CAR2_NAME, start_location_x=car2_start_location_x,
                                          start_location_y=car2_start_location_y, car_start_yaw=car2_start_yaw)
        # Set initial position and yaw of Car3
        self.set_initial_position_and_yaw(car_name=CAR3_NAME, start_location_x=car3_start_location_x,
                                          start_location_y=car3_start_location_y, car_start_yaw=car3_start_yaw)
        # Set initial position and yaw of Car4
        self.set_initial_position_and_yaw(car_name=CAR4_NAME, start_location_x=car4_start_location_x,
                                          start_location_y=car4_start_location_y, car_start_yaw=car4_start_yaw)

        for car_name, car_obj in self.setup_manager.cars.items():
            if car_obj.is_active:
                self.setup_manager.set_car_throttle_by_name(car_name)

    def collision_occurred(self):
        collision_info = self.airsim_client.simGetCollisionInfo()
        return collision_info.has_collided

    def has_reached_target(self, car_state):
        #  this function gets car state, should it?
        return car_state['x_c1'] > CAR1_DESIRED_POSITION[0]

    def get_car_controls(self, car_name):
        return self.airsim_client.getCarControls(car_name)

    def set_car_controls(self, updated_car_controls, car_name):
        self.airsim_client.setCarControls(updated_car_controls, car_name)

    def get_car_position_and_speed(self, car_name):
        car_position = self.airsim_client.simGetObjectPose(car_name).position
        car_position_and_speed = {
            "x": car_position.x_val,
            "y": car_position.y_val,
            "Vx": self.airsim_client.getCarState(car_name).kinematics_estimated.linear_velocity.x_val,
            "Vy": self.airsim_client.getCarState(car_name).kinematics_estimated.linear_velocity.y_val,
        }
        return car_position_and_speed

    def get_cars_distance(self):
        car1_position_and_speed = self.get_car_position_and_speed(CAR1_NAME)
        car2_position_and_speed = self.get_car_position_and_speed(CAR2_NAME)
        dist_c1_c2 = np.sum(np.square(
            np.array([[car1_position_and_speed["x"], car1_position_and_speed["y"]]]) -
            np.array([[car2_position_and_speed["x"], car2_position_and_speed["y"]]])))
        return dist_c1_c2

    def get_local_input_car1_perspective(self):
        car1_state = self.get_car_position_and_speed(CAR1_NAME)
        car2_state = self.get_car_position_and_speed(CAR2_NAME)
        dist_c1_c2 = self.get_cars_distance()
        local_input_car1_perspective = {
            "x_c1": car1_state["x"],
            "y_c1": car1_state["y"],
            "Vx_c1": car1_state["Vx"],
            "Vy_c1": car1_state["Vy"],
            "x_c2": car2_state["x"],
            "y_c2": car2_state["y"],
            "Vx_c2": car2_state["Vx"],
            "Vy_c2": car2_state["Vy"],
            "dist_c1_c2": dist_c1_c2
        }
        return local_input_car1_perspective

    def get_local_input_car2_perspective(self):
        car2_state = self.get_car_position_and_speed(CAR2_NAME)
        car1_state = self.get_car_position_and_speed(CAR1_NAME)
        dist_c1_c2 = self.get_cars_distance()
        local_input_car1_perspective = {
            "x_c2": car2_state["x"],
            "y_c2": car2_state["y"],
            "Vx_c2": car2_state["Vx"],
            "Vy_c2": car2_state["Vy"],
            "x_c1": car1_state["x"],
            "y_c1": car1_state["y"],
            "Vx_c1": car1_state["Vx"],
            "Vy_c1": car1_state["Vy"],
            "dist_c1_c2": dist_c1_c2
        }
        return local_input_car1_perspective

    def set_initial_position_and_yaw(self, car_name, start_location_x, start_location_y, car_start_yaw):
        # Convert yaw values from degrees to radians as AirSim uses radians
        car_start_yaw_rad = np.radians(car_start_yaw)
        # Set initial position and yaw of Car1
        initial_position_car = airsim.Vector3r(start_location_x, start_location_y, -1)
        initial_orientation_car = airsim.to_quaternion(0, 0, car_start_yaw_rad)  # Roll, Pitch, Yaw
        initial_pose_car = airsim.Pose(initial_position_car, initial_orientation_car)
        self.airsim_client.simSetVehiclePose(initial_pose_car, True, car_name)
