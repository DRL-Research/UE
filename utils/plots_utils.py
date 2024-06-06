import matplotlib.pyplot as plt


def plot_the_spline(xi, yi):
    # Plot the points as a line
    plt.figure(figsize=(10, 6))
    plt.cla()
    ax = plt.gca()
    #ax.invert_xaxis()
    plt.plot(yi, xi, label='Line Plot')

    # Add labels and title
    plt.xlabel('Y-axis')
    plt.ylabel('X-axis')
    plt.title('Bezier Curve For The Car')

    # Show legend if needed
    plt.legend()

    # Display the plot
    plt.show()

def plot_vehicle_relative_path(points):

    plt.figure(figsize=(10, 6))
    plt.cla()
    ax = plt.gca()
    #ax.invert_xaxis()

    # Extract x and y coordinates
    x_coords = [point[0] for point in points]
    y_coords = [point[1] for point in points]

    # x_coords = x_coords[::8]
    # y_coords = y_coords[::8]

    # Plotting with reversed x-axis
    plt.plot(y_coords, x_coords, marker='o', linestyle='-')
    plt.title('The Car Actual Path')
    plt.xlabel('Y Coordinate')
    plt.ylabel('X Coordinate')

    # # Reverse the x-axis
    # plt.xlim(max(x_coords), min(x_coords))

    plt.grid(True)
    plt.show()

def plot_vehicle_object_path(points):
    # todo: modify function to plot all vehicle paths - will get list of ponits for each vehcile
    #
    plt.figure(figsize=(10, 6))
    ax = plt.gca()
    ax.invert_yaxis()

    # Extract x and y coordinates
    x_coords = [point[0] for point in points]
    y_coords = [point[1] for point in points]

    # Plotting with reversed x-axis
    plt.plot(x_coords,y_coords, marker='o', linestyle='-')
    plt.title('The Car Actual Path')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    plt.grid(True)
    plt.show()

def combine_plot(xi, yi, points):
    plt.figure(figsize=(10, 6))
    plt.cla()
    ax = plt.gca()
    #ax.invert_xaxis()

    x_coords = [point[0] for point in points]
    y_coords = [point[1] for point in points]

    # x_coords = x_coords[::8]
    # y_coords = y_coords[::8]

    print('-' * 75)
    # -1 to convert to airsim for the plot
    print(f"route start at: {xi[0], -1*yi[0]}")
    print(f"car start: {points[0][0], points[0][1]}")
    print('-' * 75)
    # -1 to convert to airsim for the plot
    plt.plot(-1*yi, xi,color='red', label='bezier')
    # plt.plot(y_coords, x_coords, marker='o', linestyle='-',color='green',label='actual')
    plt.plot(y_coords, x_coords, color='green', label='actual')

    plt.title('The Car Actual Path')
    plt.xlabel('Y Coordinate')
    plt.ylabel('X Coordinate')

    plt.legend()
    plt.show()
