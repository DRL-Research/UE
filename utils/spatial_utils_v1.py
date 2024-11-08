import math
import numpy as np
from scipy.spatial.transform import Rotation

# Unreal has a left-handed coordinate system, with odd rotations.
# The order of rotation is an intrinsic yaw -> pitch -> roll schema.
# To transfer from Unreal to engineering notations, one must:
# flip the Y, yaw and pitch directions (and vice versa).
# In addition, Airsim flips the Z direction because it uses an aerial NED coordinate system,
# so this must also be taken into account.
# ENG or eng refers to an Engineering coordinate system (right-handed),
# with X pointing forward, Y pointing left and Z pointing up.
# Cascaded multiplications add up from the left-hand side.

bottom_row = np.array([0.0, 0.0, 0.0, 1.0]).reshape(1, 4)


# Convert between 2 coordinate systems and vice-vera are the same function!
# Input should be numpy arrays. Behavior with lists is undefined.
def convert_eng_unreal(pos, rot):
    pos[1] *= -1
    rot[:2] *= -1
    return pos, rot


def convert_eng_airsim(pos, rot):
    pos[1] *= -1
    pos[2] *= -1
    rot[:2] *= -1
    return pos, rot


def convert_unreal_airsim(pos, rot):
    pos[2] *= -1
    return pos, rot


# Convert from engineering to camera coordinate systems without messing with rotation matrix calculations.
def eng_to_camera(pos):
    new_pos = np.array([-pos[1], -pos[2], pos[0]])
    return new_pos


def camera_to_eng(pos):
    new_pos = np.array([pos[2], -pos[0], -pos[1]])
    return new_pos


def set_airsim_pose(airsim_client, desired_position, desired_rot, inherit_z=True, moving_car_name='Car1'):
    # Input is in ENG coordinate system!
    # Both desired position and rotation must have 3 elements: [x,y,z] and [yaw,pitch,roll].
    # Converts to Airsim and sends to client.

    desired_position = np.array(desired_position)  # To accept lists as well.
    desired_rot = np.array(desired_rot)  # To accept lists as well.
    initial_pose = airsim_client.simGetVehiclePose(vehicle_name=moving_car_name)

    if inherit_z:
        desired_position = np.append(desired_position, -initial_pose.position.z_val)

    # Convert to Airsim coordinate system:
    pos, rot = convert_eng_airsim(desired_position, desired_rot)

    # Convert Euler angles to a quaternion:
    rotator = Rotation.from_euler('ZYX', rot, degrees=True)
    quat = rotator.as_quat()
    initial_pose.orientation.x_val = quat[0]
    initial_pose.orientation.y_val = quat[1]
    initial_pose.orientation.z_val = quat[2]
    initial_pose.orientation.w_val = quat[3]
    initial_pose.position.x_val = pos[0]
    initial_pose.position.y_val = pos[1]
    initial_pose.position.z_val = pos[2]

    # Send results to client:
    airsim_client.simSetVehiclePose(initial_pose, ignore_collision=True, vehicle_name=moving_car_name)


def extract_rotation_from_airsim(orientation):
    # Input should be a Quaternionr() object directly from Airsim.
    # Output is in Airsim coordinate system.
    quaternion = np.array([orientation.x_val,
                           orientation.y_val,
                           orientation.z_val,
                           orientation.w_val])
    return Rotation.from_quat(quaternion).as_euler('ZYX', degrees=True)


def extract_pose_from_airsim(actor_pose):
    # Input is an Airsim Pose() object.
    # Output is in ENG coordinate system.
    position = np.array([actor_pose.position.x_val, actor_pose.position.y_val, actor_pose.position.z_val])
    rotation = extract_rotation_from_airsim(actor_pose.orientation)
    pos, rot = convert_eng_airsim(position, rotation)
    return pos, rot


def tf_matrix_from_eng_pose(position, rotation):
    # Input is position and rotation objects in ENG coordinate system.
    # Output is a 4x4 transform matrix in ENG coordinate system.
    position = np.array(position)
    rot = Rotation.from_euler('ZYX', rotation, degrees=True).as_matrix()
    tf_matrix = np.append(rot, position.reshape(3, 1), axis=1)
    tf_matrix = np.append(tf_matrix, bottom_row, axis=0)
    return tf_matrix


def tf_matrix_from_unreal_pose(position, rotation):
    # Input is position and rotation objects in Unreal coordinate system.
    # Output is a 4x4 transform matrix in ENG coordinate system.
    yaw_pitch_roll = np.array(rotation)
    pos, rot = convert_eng_unreal(position, yaw_pitch_roll)
    return tf_matrix_from_eng_pose(pos, rot)


def tf_matrix_from_airsim_pose(position, rotation):
    # Input is position and rotation objects in Airsim coordinate system.
    # Output is a 4x4 transform matrix in ENG coordinate system.
    yaw_pitch_roll = np.array(rotation)
    pos, rot = convert_eng_airsim(position, yaw_pitch_roll)
    return tf_matrix_from_eng_pose(pos, rot)


def tf_matrix_from_airsim_object(actor_pose):
    # Input is an Airsim Pose() object.
    # Output is a 4x4 transform matrix in ENG coordinate system.
    position = np.array([actor_pose.position.x_val, actor_pose.position.y_val, actor_pose.position.z_val])
    yaw_pitch_roll = extract_rotation_from_airsim(actor_pose.orientation)
    pos, rot = convert_eng_airsim(position, yaw_pitch_roll)
    # NOTE - for a car in yaw = 180, a user should uncomment the next line.
    # rot = np.array([0, 0, 0])
    return tf_matrix_from_eng_pose(pos, rot)


##########################################################################################################

def get_car_settings_position(client, car_name):
    return client.simGetObjectPose(car_name).position

def calculate_distance_in_2d_from_3dvector(position1, position2):
    """
    Calculate the Euclidean distance between two points in 2D or 3D space.

    Parameters:
    position1 (list or 3dVector): The first position. If is_array is True, it should be a list or array of (x, y, z) coordinates.
    position2 (list or 3dVector): The second position. If is_array is True, it should be a list or array of (x, y, z) coordinates.
    is_array (bool): If True, position1 and position2 are assumed to be lists or arrays of coordinates. Otherwise, they are objects with x_val, y_val, z_val

    Returns:
    float: The Euclidean distance between position1 and position2.
    """
    distance = math.sqrt((position1.x_val - position2.x_val) ** 2 +
                                           (position1.y_val - position2.y_val) ** 2)
    # position1 = [position1.x_val, position1.y_val]
    # position2 = [position2.x_val, position2.y_val]
    # position1 = np.array(position1)
    # position2 = np.array(position2)
    # distance = np.linalg.norm(position1 - position2)

    return distance
def calculate_distance_in_2d_from_array(position1, position2):
    """
    Calculate the Euclidean distance between two points in 2D or 3D space.

    Parameters:
    position1 (list or 3dVector): The first position. If is_array is True, it should be a list or array of (x, y, z) coordinates.
    position2 (list or 3dVector): The second position. If is_array is True, it should be a list or array of (x, y, z) coordinates.
    is_array (bool): If True, position1 and position2 are assumed to be lists or arrays of coordinates. Otherwise, they are objects with x_val, y_val, z_val

    Returns:
    float: The Euclidean distance between position1 and position2.
    """
    # if len(position1) != 2:
    #     position1 = [position1[0], position1[1]]
    # if len(position2) != 2:
    #     position2 = [position2[0], position2[1]]
    # position1 = np.array(position1)
    # position2 = np.array(position2)
    # distance = np.linalg.norm(position1 - position2)
    distance = math.sqrt((position1[0] - position2[0]) ** 2 +
                                           (position1[1] - position2[1]) ** 2)
    return distance

##########################################################################################################


if __name__ == "__main__":
    import airsim
    import time
    client = airsim.CarClient()
    client.confirmConnection()
    time.sleep(1.0)

    set_airsim_pose(client, [0.0, 20.0], [45.0, 0, 0])
    test_pose = client.simGetVehiclePose()
    tf_mat = tf_matrix_from_airsim_pose([1, 2, 3], [40, 0, 0])
    tf_2 = tf_matrix_from_eng_pose([1, 2, 3], [40, 0, 0])
    pass
