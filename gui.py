import os
import sys
import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from PyQt5 import uic, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel
from PyQt5.QtSvg import QSvgWidget
from PyQt5.QtGui import QPixmap, QIcon
import numpy as np
from time import sleep
import ctypes

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl


Form = uic.loadUiType(os.path.join(os.getcwd(), "assets/ui/Form.ui"))[0]

myappid = 'alient12.RobotDT.DTProject.2025' # arbitrary string
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

line1_color = "b"
line2_color = "r"
ax_num_color = "k"
ax_line_color = "k"

class MainWindow(QMainWindow, Form):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)

        # connect to Swift visualization
        self.swift_widget.setUrl(QUrl('http://localhost:52000/?53000'))

        # 3D end-effector tracker
        self.plot3d_fig = Figure(frameon=False, figsize=(5,10));self.plot3d_fig.patch.set_color("w")
        self.plot3d_ax = self.plot3d_fig.add_subplot(111, projection='3d', frame_on=False)
        self.plot3d_canvas = FigureCanvas(self.plot3d_fig)
        # make 3d plot background trasparent
        self.plot3d_ax.set_facecolor((0.0, 0.0, 0.0, 0.0))
        self.plot3d_canvas.setStyleSheet("background-color:transparent;")
        
        self.plot3d_fig.tight_layout()

        self.plot3d_ax.tick_params(axis='x', colors='k');[t.set_color('gray') for t in self.plot3d_ax.xaxis.get_ticklabels()]
        self.plot3d_ax.tick_params(axis='y', colors='k');[t.set_color('gray') for t in self.plot3d_ax.yaxis.get_ticklabels()]
        self.plot3d_ax.tick_params(axis='z', colors='k');[t.set_color('gray') for t in self.plot3d_ax.zaxis.get_ticklabels()]

        t = np.linspace(0, 1, 10)  # Smaller increments for a smoother trajectory
        data = {
            "FAC1_1": np.sin(2 * np.pi * t),  # Example: sine wave trajectory
            "FAC2_1": np.cos(2 * np.pi * t),  # Example: cosine wave trajectory
            "FAC3_1": t  # Linear increment for simplicity
        }
        error = np.random.normal(0, 0.05, size=t.shape)  # Small random errors
        data_with_error = {
            "FAC1_1": np.sin(2 * np.pi * t) + error,
            "FAC2_1": np.cos(2 * np.pi * t) + error,
            "FAC3_1": t
        }
        # self.plot3d_ax.plot(data["FAC1_1"], data["FAC2_1"], data["FAC3_1"], 'o-', markersize=10, color='r', label='Line')
        # self.plot3d_ax.plot(data_with_error["FAC1_1"], data_with_error["FAC2_1"], data_with_error["FAC3_1"], 'o-', markersize=5, color='b', label='Trajectory with Error')
        
        # Plotting the original trajectory with fading effect
        for i in range(len(t)-1):
            self.plot3d_ax.plot(data["FAC1_1"][i:i+2], data["FAC2_1"][i:i+2], data["FAC3_1"][i:i+2],
                                'o-', markersize=5, color=(1, 0, 0, t[i]), label='Original Trajectory' if i == 0 else "")

        # Plotting the nearby trajectory with a small error and fading effect
        for i in range(len(t)-1):
            self.plot3d_ax.plot(data_with_error["FAC1_1"][i:i+2], data_with_error["FAC2_1"][i:i+2], data_with_error["FAC3_1"][i:i+2],
                                'o-', markersize=5, color=(0, 0, 1, t[i]), label='Trajectory with Error' if i == 0 else "")

        self.plot3d_navi = NavigationToolbar(self.plot3d_canvas, self)
        plot3d_l = QVBoxLayout(self.plot3d_widget);plot3d_l.addWidget(self.plot3d_canvas);plot3d_l.addWidget(self.plot3d_navi)
        

        # Joints plot
        self.fig = Figure(frameon=False);self.fig.patch.set_color("w")
        # Create 6 subplots in a 2x3 grid
        self.ax1 = self.fig.add_subplot(321, frame_on=True)
        self.ax2 = self.fig.add_subplot(322, frame_on=True)
        self.ax3 = self.fig.add_subplot(323, frame_on=True)
        self.ax4 = self.fig.add_subplot(324, frame_on=True)
        self.ax5 = self.fig.add_subplot(325, frame_on=True)
        self.ax6 = self.fig.add_subplot(326, frame_on=True)

        self.fig.tight_layout()
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setStyleSheet("background-color:transparent;")
        self.navi = NavigationToolbar(self.canvas, self)
        l = QVBoxLayout(self.joints_plot_widget);l.addWidget(self.canvas);l.addWidget(self.navi)

        x = np.linspace(0, 2 * np.pi, 100);y = np.zeros_like(x)
        
        # (self.line1,) = self.ax.plot(x, y, line1_color, lw=2, label="setpoint")
        # (self.line2,) = self.ax.plot(x, y, line2_color, lw=2, label="sensor")
        # self.ax.grid()
        # self.ax.set_ylim(-180, 180);self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1), ncol=3, fancybox=True, shadow=True)
        # self.ax.tick_params(axis='x', colors=ax_line_color);[t.set_color(ax_num_color) for t in self.ax.xaxis.get_ticklabels()]
        # self.ax.tick_params(axis='y', colors=ax_line_color);[t.set_color(ax_num_color) for t in self.ax.yaxis.get_ticklabels()]
        

        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]:
            (line1,) = ax.plot(x, y, 'b-', lw=2, label="setpoint")
            (line2,) = ax.plot(x, y, 'r-', lw=2, label="sensor")
            
            ax.grid()
            ax.set_ylim(-180, 180)
            # ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1), ncol=3, fancybox=True, shadow=True)
            ax.set_facecolor((0.0, 0.0, 0.0, 0.0))
            ax.tick_params(axis='x', colors='k');[t.set_color('gray') for t in ax.xaxis.get_ticklabels()]
            ax.tick_params(axis='y', colors='k');[t.set_color('gray') for t in ax.yaxis.get_ticklabels()]
        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
