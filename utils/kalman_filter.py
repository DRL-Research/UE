from filterpy.kalman import KalmanFilter as filter_py_kf
from pykalman import KalmanFilter
import numpy as np


def constant_velocity_kalman_filter(dt, initial_state, other_car_velocity, other_car_direction):
    kf = filter_py_kf(dim_x=4, dim_z=2)

    # Define state transition matrix with known velocity and direction
    kf.F = np.array([[1, 0, other_car_velocity * np.cos(other_car_direction) * dt, 0],
                     [0, 1, other_car_velocity * np.sin(other_car_direction) * dt, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    # Define measurement matrix (identity matrix for position)
    kf.H = np.array([[1, 0, 0, 0],  # x-component
                     [0, 1, 0, 0]])  # y-component

    kf.Q *= 0.01      # Define process noise covariance
    kf.R *= 0.1     # Define measurement noise covariance
    kf.x = np.array(initial_state)     # Set initial state

    return kf


def kalman_filter(init_state, transition_matrix):
    kf = KalmanFilter()


# Example usage
if __name__ == "__main__":
    dt = 0.5
    initial_state = [25, -15, 0, 5]  # Initial state [x, y, vx, vy]
    other_car_velocity = 5  # Velocity of the other car
    other_car_direction = np.pi   # Direction of the other car in radians
    kf = constant_velocity_kalman_filter(dt, initial_state, other_car_velocity, other_car_direction)

    measurements = np.array([[25, -15], [25, -12], [25, -9], [25, -6], [25, -3], [25, 0]])

    filtered_states = []
    for measurement in measurements:
        kf.predict()
        kf.update(measurement)
        filtered_states.append(kf.x)

    print("Filtered states:")
    for state in filtered_states:
        output = [round(i, 2) for i in state]
        print(output)