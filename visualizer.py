# visualizer.py
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class LiveNavVisualizer:
    def __init__(self, follow_drone=False):

        self.follow_drone = follow_drone
        self.pose = (0, 0, 0)          # x, y, yaw
        self.obstacles = []           # [(x, y), ...]
        self.path = []                # [(x, y), ...]

        self.fig, self.ax = plt.subplots()
        self.drone_arrow = None
        self.path_line, = self.ax.plot([], [], 'b-', label="Path")
        self.obs_scatter = self.ax.scatter([], [], c='r', s=10, label="Obstacles")

        self.ax.set_title("FlatBuffer NavData Viewer")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.axis("equal")
        self.ax.grid(True)
        self.ax.legend(loc="upper right")

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=500)

    def update_data(self, pose, obstacles, path):
        self.pose = pose
        self.obstacles = obstacles
        self.path = path

    def update_plot(self, frame):
        # Path
        if self.path:
            px, py = zip(*self.path)
            self.path_line.set_data(px, py)
        else:
            self.path_line.set_data([], [])

        # Obstacles
        if self.obstacles:
            ox, oy = zip(*self.obstacles)
            self.obs_scatter.set_offsets(list(zip(ox, oy)))
        else:
            self.obs_scatter.set_offsets(np.empty((0, 2)))


        # Pose arrow
        if self.drone_arrow:
            self.drone_arrow.remove()
        x, y, yaw = self.pose
        dx = 0.3 * round(np.cos(yaw), 2)
        dy = 0.3 * round(np.sin(yaw), 2)
        self.drone_arrow = self.ax.arrow(x, y, dx, dy, head_width=0.2, fc='g', ec='g')

        # Optional: follow drone
        if self.follow_drone:
            self.ax.set_xlim(x - 5, x + 5)
            self.ax.set_ylim(y - 5, y + 5)

    def show(self):
        plt.show()

import numpy as np  # required for cosine/sine
