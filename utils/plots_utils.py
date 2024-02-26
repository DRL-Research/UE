import matplotlib.pyplot as plt


def plot_the_spline(xi, yi):
    # Plot the points as a line
    plt.plot(yi, xi, label='Line Plot')

    # Add labels and title
    plt.xlabel('Y-axis')
    plt.ylabel('X-axis')
    plt.title('Line Plot of (x, y) Points')

    # Show legend if needed
    plt.legend()

    # Display the plot
    plt.show()

def plot_the_car_path(points):
    # Extract x and y coordinates
    x_coords = [point[0] for point in points]
    y_coords = [point[1] for point in points]

    x_coords = x_coords[::8]
    y_coords = y_coords[::8]

    # Plotting with reversed x-axis
    plt.plot(x_coords, y_coords, marker='o', linestyle='-')
    plt.title('the_car_path')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    # # Reverse the x-axis
    # plt.xlim(max(x_coords), min(x_coords))

    plt.grid(True)
    plt.show()
