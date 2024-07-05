import matplotlib.pyplot as plt


# Create the plot
plt.figure(figsize=(10, 6))

# Add text above the center (0, 0)
plt.plot(0, 0, 'ko')  # 'ro' is a red circle
plt.text(0, 1, 'Sensor Location', ha='center', fontsize=12, color='black')

# Plot the point at (0, 10) and add text "True Location"
true_location, =plt.plot(0, 10, 'ro')  # 'ro' is a red circle
plt.text(0, 11, 'True Location', ha='center', fontsize=12, color='red')

# Plot the point at (9.867, -0.087) and add text "Predicted Location"
predicted_location, = plt.plot(-0.087, 9.867, 'bo')  # 'bo' is a blue circle
plt.text(0, 9, 'Predicted Location', ha='center', fontsize=12, color='blue')

# Set the limits of the plot
plt.xlim(-10, 10)
plt.ylim(-5, 15)

# Add labels and title
plt.xlabel('X Coordinates')
plt.ylabel('Y Coordinates')
plt.title('Car Detection By LiDAR, Error = 16 cm')

# Show grid
plt.grid(True)

# Show the plot
plt.show()
