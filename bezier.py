import numpy as np

def quadratic_bezier(t, p0, p1, p2):
    u = 1 - t
    return u**2 * p0 + 2 * u * t * p1 + t**2 * p2


def generate_curve_points(p0, p1, p2, num_points=20000):
    curve_points = []
    for i in range(num_points):
        t = i / (num_points - 1)
        point = quadratic_bezier(t, p0, p1, p2)
        curve_points.append(point)
    curve_points = [[p[0], p[1]] for p in curve_points]
    return curve_points

# todo: do we use this ? if not then remove it
def menger_curvature(prev_p, curr_p, next_p):
    """
    Calculate the Menger curvature at a given point on a curve defined by three consecutive points.

    Parameters:
    - prev_p (numpy.ndarray): Coordinates of the previous point.
    - curr_p (numpy.ndarray): Coordinates of the current point.
    - next_p (numpy.ndarray): Coordinates of the next point.

    Returns:
    - curvature (float): Menger curvature at the current point.
    """
    prev_unit_vec = (curr_p - prev_p) / np.linalg.norm(curr_p - prev_p)
    next_unit_vec = (next_p - curr_p) / np.linalg.norm(next_p - curr_p)
    base_angle = np.arccos(np.dot(prev_unit_vec, next_unit_vec))
    base_length = np.linalg.norm(next_p - prev_p)
    curvature = 2 * np.sin(base_angle) / base_length
    if curvature < 1e-4:  # Meaningless value for our purpose
        curvature = 0
    return curvature
