import json
import time
from enum import Enum
from typing import NamedTuple, Dict
import airsim

import spatial_utils


class JsonKeys(Enum):
    CAR_NAME_AS_ID = "Name as ID"
    CAR_SPEED = "Speed"
    CAR_INITIAL_LOCATION = "Initial Location"
    CAR_DESTINATION = "Destination"
    VEHICLES = "Vehicles"
    X = "X"
    Y = "Y"


class Car(NamedTuple):
    name_as_id: str
    speed: float
    initial_location: dict
    destination: dict


class CarDict(Dict[str, Car]):
    pass


class SetupManager:
    def __init__(self, setup_simulation_path='config.json', airsim_settings_path=''):
        self.setup_simulation_path = setup_simulation_path
        with open(setup_simulation_path, 'r') as f:
            self._data = json.load(f)
        self.cars = CarDict()
        self.airsim_settings_full_path = airsim_settings_path + "/setting.json"

    def extract_cars(self):
        for car_full_id, car_data in self._data.items():
            current_car = Car(name_as_id=car_data[JsonKeys.CAR_NAME_AS_ID],
                              speed=car_data[JsonKeys.CAR_SPEED],
                              initial_location=car_data[JsonKeys.CAR_INITIAL_LOCATION],
                              destination=car_data[JsonKeys.CAR_DESTINATION])
            # noinspection PyTypedDict
            self.cars[car_full_id] = current_car

    # NOTE: if a user want to use this function, he should run it before starting "play" in the simulator.
    def update_airsim_settings(self):
        # read
        with open(self.airsim_settings_full_path, 'r') as f:
            airsim_settings = json.load(f)  # load as a dict

        # update
        vehicles = airsim_settings[JsonKeys.VEHICLES]
        for car_object in self.cars.values():
            name = car_object.name_as_id
            x = car_object.initial_location.get(JsonKeys.X)
            y = car_object.initial_location.get(JsonKeys.Y)
            vehicles[name][JsonKeys.X] = x
            vehicles[name][JsonKeys.Y] = y

        # write
        with open(self.airsim_settings_full_path, 'w') as f:
            json.dump(airsim_settings, f, indent=4)

        return True

    def setup_simulation(self):
        # step 1: read base location
        self.extract_cars()
        # step 2: airsim connection
        airsim_client = airsim.CarClient()
        airsim_client.confirmConnection()
        for car_id, car_object in self.cars.items():
            airsim_client.enableApiControl(is_enabled=True, vehicle_name=car_object.name_as_id)
        # step 3: set positions
        for car_object in self.cars.values():
            spatial_utils.set_airsim_pose(airsim_client, car_object.name_as_id, car_object.initial_location, [90.0, 0, 0])

        time.sleep(1.0)
        # set speed / maybe remove this part
        for car_object in self.cars.values():
            car_controls = airsim.CarControls()
            car_controls.throttle = car_object.speed  # was just 0.2
            airsim_client.setCarControls(vehicle_name=car_object.name_as_id, controls=car_controls)













