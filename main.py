from PyQt5 import QtWidgets
from stewart_control.gui.gui_layout import StewartGUILayout
from stewart_control.kinematics import ik_solver  # or whatever module contains solve_pose
import sys

class IKWrapper:
    """Wrapper to make a solver object with a solve_pose() method for the GUI."""
    def solve_pose(self, x, y, z, roll, pitch, yaw, prev_arm_points=None):
        return ik_solver.solve_pose(x, y, z, roll, pitch, yaw, prev_arm_points)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    # Create a solver instance
    solver_instance = IKWrapper()

    window = StewartGUILayout(ik_solver=solver_instance)
    window.show()

    exit_code = app.exec_()
    sys.exit(exit_code)
