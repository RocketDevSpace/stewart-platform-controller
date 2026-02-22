import sys
from PyQt5.QtWidgets import QApplication
from stewart_control.gui.gui_layout import StewartGUILayout

def run_gui():
    app = QApplication(sys.argv)
    window = StewartGUILayout()
    window.setWindowTitle("Stewart Platform Controller")
    window.show()
    sys.exit(app.exec_())
