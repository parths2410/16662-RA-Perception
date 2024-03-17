import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def generate_rectangle_points(l, b, c_x=0, c_y=0):
    # Calculate corner points
    top_right = (l/2 + c_x, b/2 + c_y)
    top_left = (-l/2 + c_x, b/2 + c_y)
    bottom_left = (-l/2 + c_x, -b/2 + c_y)
    bottom_right = (l/2 + c_x, -b/2 + c_y)
    
    # Generate points along each edge, separated by 2 mm
    points = []
    
    # Top edge
    points += [(x, top_right[1]) for x in np.arange(top_right[0], top_left[0], -2e-2)]
    # Left edge
    points += [(top_left[0], y) for y in np.arange(top_left[1], bottom_left[1], -2e-2)]
    # Bottom edge
    points += [(x, bottom_left[1]) for x in np.arange(bottom_left[0], bottom_right[0], 2e-2)]
    # Right edge
    points += [(bottom_right[0], y) for y in np.arange(bottom_right[1], top_right[1], 2e-2)]
    
    return points

def generate_circle_points(r, c_x=0, c_y=0):
    # Determine the number of points needed, approximating circumference as 2*pi*r
    circumference = 2 * np.pi * r
    num_points = int(circumference / 2e-2)
    
    # Generate points along the circumference
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    points = [(r * np.cos(t) + c_x, r * np.sin(t) + c_y) for t in theta]
    
    return points

# Choose one of the shapes to visualize
shape = "rectangle"  # Change to "circle" for the circle
if shape == "rectangle":
    points = generate_rectangle_points(10, 5, 2, 2)  # Rectangle with l=10, b=5
elif shape == "circle":
    points = generate_circle_points(5, 2, 2)  # Circle with radius r=5

print (points)

# Setup the plot
fig, ax = plt.subplots()
ax.set_xlim(-10, 10)  # Adjust these limits based on your shape's size
ax.set_ylim(-10, 10)
plt.gca().set_aspect('equal', adjustable='box')  # Keep aspect ratio square

# Initialize a line artist for plotting. This will be updated in the animation.
line, = ax.plot([], [], 'bo')

# Initialize the frame for the animation
def init():
    line.set_data([], [])
    return line,

# Animation update function
def update(frame):
    x, y = zip(*points[:frame+1])  # Get all points up to the current frame
    line.set_data(x, y)
    return line,

# Create animation
ani = FuncAnimation(fig, update, frames=len(points), init_func=init, blit=True, interval=2, repeat=False)

plt.show()