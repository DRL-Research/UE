from perception import cone_mapping
from path_planning import path_following_v0
import airsim
from utils.path_planning import spline_utils_v0
from utils.path_planning import path_control_v0


if __name__ == '__main__':
    # Create an airsim client instance:
    steering_procedure_manager = path_control_v0.SteeringProcManager()
    airsim_client = airsim.CarClient()
    airsim_client.confirmConnection()
    airsim_client.enableApiControl(True)

    # Detect the cones and spline points, and return their location:
    print('Starting on-the-fly cone mapping with constant speed and steering procedure.')
    mapping_data, pursuit_points = cone_mapping.mapping_loop(airsim_client)
    print('Mapping complete!')

    # Stop until spline generation is complete:
    print('Stopping vehicle and generating a path to follow...')
    car_controls = airsim_client.getCarControls()
    car_controls.throttle = 0.0
    airsim_client.setCarControls(car_controls)

    # Arrange the points and generate a path spline:
    track_points = spline_utils_v0.generate_path_points(mapping_data)
    spline_obj = spline_utils_v0.PathSpline(track_points[::2, 0], track_points[::2, 1])
    spline_obj.generate_spline(amount=0.1, meters=True, smoothing=1)
    print('Done!')

    # Follow the spline using Stanley's method:
    print('Starting variable speed spline following procedure.')
    path_following_v0.following_loop(airsim_client, spline_obj)
    print('Full process complete! stopping vehicle.')

    # Done! stop vehicle:
    car_controls = airsim_client.getCarControls()
    car_controls.throttle = 0.0
    car_controls.brake = 1.0
    airsim_client.setCarControls(car_controls)
    steering_procedure_manager.terminate_steering_procedure()
