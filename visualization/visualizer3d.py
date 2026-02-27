import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from stewart_control.config import SERVO_SHAFTS, SERVO_AXES, PLATFORM_SIZE
from stewart_control.kinematics.ik_solver import solve_pose

class StewartVisualizer:
    def __init__(self, canvas: FigureCanvas):
        """
        canvas: FigureCanvas to draw the 3D plot into
        """
        self.canvas = canvas
        self.prev_arm_points = None

        # Create a figure in the canvas
        self.fig = canvas.figure
        self.fig.patch.set_facecolor("#0f1726")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_facecolor("#0f1726")
        self.ax.set_box_aspect([1,1,0.8])

        # initial dummy platform position
        self.platform_pos = {'x':0,'y':0,'z':130,'roll':0,'pitch':0,'yaw':0}

    def update_platform(self, pos_dict, ik_result=None):
        """
        pos_dict: dictionary with keys x, y, z, roll, pitch, yaw
        """
        self.platform_pos = pos_dict
        x = pos_dict['x']
        y = pos_dict['y']
        z = pos_dict['z']
        roll  = pos_dict['roll']
        pitch = pos_dict['pitch']
        yaw   = pos_dict['yaw']

        # Reuse IK if provided, otherwise solve locally.
        solution = ik_result
        if solution is None:
            solution = solve_pose(x, y, z, roll, pitch, yaw, prev_arm_points=self.prev_arm_points)

        # Clear axes
        self.ax.cla()

        # --- Draw servo shafts ---
        self.ax.scatter(SERVO_SHAFTS[:,0], SERVO_SHAFTS[:,1], SERVO_SHAFTS[:,2], s=50, c='r', label='Shafts')

        # Draw shaft axes
        for i in range(6):
            p = SERVO_SHAFTS[i]
            axis = SERVO_AXES[i]
            line = np.array([p - axis*10, p + axis*10])
            self.ax.plot(line[:,0], line[:,1], line[:,2], c='r')

        if solution['success']:
            platform_points = solution["platform_points"]
            arm_points = solution["arm_points"]

            # Draw square platform
            self.draw_square_platform(solution["platform_center"], solution["platform_R"])

            # Draw platform points
            self.ax.scatter(platform_points[:,0], platform_points[:,1], platform_points[:,2], s=60, c='b', label='Platform Points')

            # Draw arm points
            self.ax.scatter(arm_points[:,0], arm_points[:,1], arm_points[:,2], s=60, c='g', label='Arm Points')

            # Draw rods
            for i in range(6):
                rod = np.array([arm_points[i], platform_points[i]])
                self.ax.plot(rod[:,0], rod[:,1], rod[:,2], linewidth=2, c='k')

            # Draw arms
            for i in range(6):
                arm = np.array([SERVO_SHAFTS[i], arm_points[i]])
                self.ax.plot(arm[:,0], arm[:,1], arm[:,2], linewidth=2, c='orange')

            self.prev_arm_points = arm_points.copy()

            # print angles for debug
            angles = solution["servo_angles_deg"]
            # print("Servo angles:", ["%.2f" % a for a in angles])
        else:
            print("FAILED IK:", solution["debug"]["failures"])

        # --- Axis formatting ---
        self.ax.set_xlabel("X", color="#d6e2ff")
        self.ax.set_ylabel("Y", color="#d6e2ff")
        self.ax.set_zlabel("Z", color="#d6e2ff")
        self.ax.tick_params(colors="#9fb4d9")
        self.ax.set_xlim(-120, 120)
        self.ax.set_ylim(-120, 120)
        self.ax.set_zlim(0, 200)
        self.ax.set_title("Stewart Platform IK Visualizer", color="#d6e2ff")

        self.ax.view_init(elev=20, azim=30)  # initial view
        self.ax.legend()
        self.canvas.draw_idle()

    def draw_square_platform(self, center, R):
        half = PLATFORM_SIZE / 2
        corners_local = np.array([
            [-half, -half, 0],
            [ half, -half, 0],
            [ half,  half, 0],
            [-half,  half, 0],
            [-half, -half, 0],
        ])
        corners_world = (R @ corners_local.T).T + center
        self.ax.plot(corners_world[:,0], corners_world[:,1], corners_world[:,2], linewidth=2, c='b')
