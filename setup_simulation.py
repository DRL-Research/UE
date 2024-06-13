import json
import time
from enum import Enum
from typing import NamedTuple, Dict
import airsim
from config import *


class JsonKeys(Enum):
    NUMBER_OF_ACTIVE_CARS = "Number-Of-Active-Cars"


class Car(NamedTuple):
    name: str
    speed: float
    yaw: float
    initial_position: list
    destination: dict
    is_active: bool


class CarDict(Dict[str, Car]):
    pass


class SetupManager:
    def __init__(self, setup_simulation_path='config.json', airsim_settings_path=''):
        self.setup_simulation_path = setup_simulation_path
        with open(setup_simulation_path, 'r') as f:
            self._data = json.load(f)
        self.n_active_cars = self._data[JsonKeys.NUMBER_OF_ACTIVE_CARS.value]
        self.cars = CarDict()
        self.airsim_settings_full_path = airsim_settings_path + "/setting.json"
        self.airsim_client = airsim.CarClient()
        self.airsim_client.confirmConnection()
        self.max_cars_we_can_handle = 4

    def extract_cars(self):
        active = [True] * self.n_active_cars
        inactive = [False] * (self.max_cars_we_can_handle-self.n_active_cars)
        is_active_per_car = active + inactive
        car1 = Car(name=CAR1_NAME, speed=0.0, yaw=CAR1_INITIAL_YAW, initial_position=CAR1_INITIAL_POSITION,
                   destination=CAR1_DESIRED_POSITION, is_active=is_active_per_car[0])
        car2 = Car(name=CAR2_NAME, speed=0.0, yaw=CAR2_INITIAL_YAW, initial_position=CAR2_INITIAL_POSITION,
                   destination=CAR2_DESIRED_POSITION, is_active=is_active_per_car[1])
        car3 = Car(name=CAR3_NAME, speed=0.0, yaw=CAR3_INITIAL_YAW, initial_position=CAR3_INITIAL_POSITION,
                   destination=CAR3_DESIRED_POSITION, is_active=is_active_per_car[2])
        car4 = Car(name=CAR4_NAME, speed=0.0, yaw=CAR4_INITIAL_YAW, initial_position=CAR4_INITIAL_POSITION,
                   destination=CAR4_DESIRED_POSITION, is_active=is_active_per_car[3])
        cars = [car1, car2, car3, car4]
        for car in cars:
            self.cars[car.name] = car

    def enableApiCarsControl(self):
        for car_id, car_object in self.cars.items():
            self.airsim_client.enableApiControl(is_enabled=True, vehicle_name=car_object.name)

    def set_car_throttle_by_name(self, car_name, throttle=0.5):
        car_controls = airsim.CarControls()
        car_controls.throttle = throttle
        self.airsim_client.setCarControls(car_controls, car_name)
