import cone_mapping
import path_following
import airsim
import spline_utils
import path_control
from setup_simulation import *
from airsim_manager import AirsimManager

if __name__ == '__main__':
    # we should get the desired definitions of the simulator by the user: update setting.json & update airsim
    setup_manager = SetupManager()
    setup_manager.extract_cars()
    setup_manager.enableApiCarsControl()
    airsim_client = setup_manager.airsim_client
    time.sleep(1.0)

    steering_procedure_manager = path_control.SteeringProcManager()

    airsim_manager = AirsimManager(airsim_client, setup_manager)

    # Detect the cones and spline points, and return their location:
    #print('Starting on-the-fly cone mapping with constant speed and steering procedure.')
    # mapping_data, pursuit_points = cone_mapping.mapping_loop(airsim_client, setup_manager)
    # #print('Mapping complete!')
    #
    # # Stop until spline generation is complete:
    # print('Stopping vehicle and generating a path to follow...')
    # car_controls = airsim_client.getCarControls()
    # car_controls.throttle = 0.0
    # airsim_client.setCarControls(car_controls)
    #
    # # Arrange the points and generate a path spline:
    # track_points = spline_utils.generate_path_points(mapping_data)
    # spline_obj = spline_utils.PathSpline(track_points[::2, 0], track_points[::2, 1])
    # spline_obj.generate_spline(amount=0.1, meters=True, smoothing=1)
    # print('Done!')
    #
    # # Follow the spline using Stanley's method:
    # print('Starting variable speed spline following procedure.')
    # path_following.following_loop(airsim_client, spline_obj)
    # print('Full process complete! stopping vehicle.')
    #
    # # Done! stop vehicle:
    # car_controls = airsim_client.getCarControls()
    # car_controls.throttle = 0.0
    # car_controls.brake = 1.0
    # airsim_client.setCarControls(car_controls)
    steering_procedure_manager.terminate_steering_procedure()
