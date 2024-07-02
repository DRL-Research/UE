from typing import NamedTuple, Dict
import airsim
from initialization.config_v1 import *


class Car(NamedTuple):
    name: str
    speed: float
    yaw: float
    initial_position: list
    destination: dict
    is_active: bool

    def get_start_location_x(self, offset):
        return self.initial_position[0] - offset

    def get_start_location_y(self, offset, side=None):
        if side is None:
            return self.initial_position[1] - offset
        else:
            return side * self.initial_position[1] - offset


class CarDict(Dict[str, Car]):
    pass


class SetupManager:
    def __init__(self):
        self.n_active_cars = NUMBER_OF_CAR_IN_SIMULATION
        self.cars = CarDict()
        self.max_cars_we_can_handle = 4
        self.extract_cars()

    def extract_cars(self):
        car1 = Car(name=CAR1_NAME, speed=0.0, yaw=CAR1_INITIAL_YAW, initial_position=CAR1_INITIAL_POSITION,
                   destination=CAR1_DESIRED_POSITION, is_active=USE_CAR1)
        car2 = Car(name=CAR2_NAME, speed=0.0, yaw=CAR2_INITIAL_YAW, initial_position=CAR2_INITIAL_POSITION,
                   destination=CAR2_DESIRED_POSITION, is_active=USE_CAR2)
        car3 = Car(name=CAR3_NAME, speed=0.0, yaw=CAR3_INITIAL_YAW, initial_position=CAR3_INITIAL_POSITION,
                   destination=CAR3_DESIRED_POSITION, is_active=USE_CAR3)
        car4 = Car(name=CAR4_NAME, speed=0.0, yaw=CAR4_INITIAL_YAW, initial_position=CAR4_INITIAL_POSITION,
                   destination=CAR4_DESIRED_POSITION, is_active=USE_CAR4)
        cars = [car1, car2, car3, car4]
        for car in cars:
            self.cars[car.name] = car
