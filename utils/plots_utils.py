import matplotlib.pyplot as plt


def plot_the_spline(xi, yi):
    # Plot the points as a line
    plt.figure(figsize=(10, 6))
    plt.cla()
    ax = plt.gca()
    ax.invert_xaxis()
    plt.plot(yi, xi, label='Line Plot')

    # Add labels and title
    plt.xlabel('Y-axis')
    plt.ylabel('X-axis')
    plt.title('Bezier Curve For The Car')

    # Show legend if needed
    plt.legend()

    # Display the plot
    plt.show()

def plot_the_car_path(points):

    plt.figure(figsize=(10, 6))
    plt.cla()
    ax = plt.gca()
    ax.invert_xaxis()

    # Extract x and y coordinates
    x_coords = [point[0] for point in points]
    y_coords = [point[1] for point in points]

    x_coords = x_coords[::8]
    y_coords = y_coords[::8]

    # Plotting with reversed x-axis
    plt.plot(y_coords, x_coords, marker='o', linestyle='-')
    plt.title('The Car Actual Path')
    plt.xlabel('Y Coordinate')
    plt.ylabel('X Coordinate')

    # # Reverse the x-axis
    # plt.xlim(max(x_coords), min(x_coords))

    plt.grid(True)
    plt.show()

def combine_plot(xi,yi,points):
    plt.figure(figsize=(10, 6))
    plt.cla()
    ax = plt.gca()
    ax.invert_xaxis()

    x_coords = [point[0] for point in points]
    y_coords = [point[1] for point in points]

    x_coords = x_coords[::8]
    y_coords = y_coords[::8]

    plt.plot(yi, xi,color='red', label='bezier')
    # plt.plot(y_coords, x_coords, marker='o', linestyle='-',color='green',label='actual')
    plt.plot(y_coords, x_coords, color='green', label='actual')

    plt.title('The Car Actual Path')
    plt.xlabel('Y Coordinate')
    plt.ylabel('X Coordinate')

    plt.legend()
    plt.show()
