import json
from enum import Enum
from typing import NamedTuple, Dict


class JsonKeys(Enum):
    CAR_NAME_AS_ID = "Name as ID"
    CAR_SPEED = "Speed"
    CAR_INITIAL_LOCATION = "Initial Location"
    CAR_DESTINATION = "Destination"


class Car(NamedTuple):
    name_as_id: str
    speed: float
    initial_location: dict
    destination: dict


class CarDict(Dict[str, Car]):
    pass


class SetupManager:
    def __init__(self, setup_simulation_path='setup_simulation.json', airsim_settings_path=''):
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
