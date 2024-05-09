
import airsim


def setCarSpeed(airsim_client: airsim.CarClient, car_name, speed):
    if airsim_client.isApiControlEnabled(car_name):
        pass
    else:
        airsim_client.enableApiControl(is_enabled=True, vehicle_name=car_name)

    car_controls = airsim.CarControls()
    car_controls.throttle = speed
    airsim_client.setCarControls(controls=car_controls, vehicle_name=car_name)

"""
Intersection Simulation:

def create_car(car_name, init_position, yaw)
    add a new car to the settings file
    init its location using the init_position
    init yaw
    define speed as HardCoded - same for everyone
    return the cars name

def plan_turn(start_turning_position, destination_position)
    define control point HardCoded
    plan and return the car route. 
    
def turn(car_name, start_turning_position, route)    
    when car gets to the start_turning_position, start moving accord to route
    
def simulation():  
    car_name = create_car(car_name, init_position, yaw)
    route = plan_turn(start_turning_position, destination_position)
    turn(car_name, start_turning_position, route)
    
notes:
    - note init_position != start_turning_position    
    - we will start with 1 car only

"""