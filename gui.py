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
import time
import ctypes

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl

import multiprocessing
import random


Form = uic.loadUiType(os.path.join(os.getcwd(), "assets/ui/Form.ui"))[0]

myappid = 'alient12.RobotDT.DTProject.2025' # arbitrary string
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

line1_color = "b"
line2_color = "r"
ax_num_color = "k"
ax_line_color = "k"

t = np.linspace(0, 2 * np.pi, 500); x_arr = np.zeros_like(t); x_setpoint_arr = np.zeros_like(t)
T0 = time.time(); T = time.time()
freq = 2 * np.pi / 100; A = 90

t10 = np.linspace(0, 1, 20)
xyz = np.zeros((3, t10.shape[0]))
joints = np.zeros((6, t.shape[0]))

fade_trajectory = False

class MainWindow(QMainWindow, Form):
    def __init__(self, shared_data, event, queue):
        super(MainWindow, self).__init__()
        self.setupUi(self)

        self.init_ui()

        # connect to Swift visualization
        self.swift_widget.setUrl(QUrl('http://localhost:52000/?53000'))
        
        self.shared_data = shared_data
        self.event = event
        self.queue = queue
        
        self.plot_process = PlotProcess(shared_data, event, queue)
        self.plot_process.start()

        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(50)  # 50 ms interval

        # Timer to update data for 3D plot process
        self.plot3d_timer = QtCore.QTimer()
        self.plot3d_timer.timeout.connect(self.update_3d_plot)
        self.plot3d_timer.start(50)  # Update every 50 ms

        self.isFullscreen = False
    
    
    def init_ui(self):
        self.init_plot()
        self.init_plot3d()

    def init_plot(self):
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

        self.jline1 = []
        self.jline2 = []

        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]:
            self.jline1.append(ax.plot(x, y, 'y-', lw=1, label="setpoint")[0])
            self.jline2.append(ax.plot(x, y, 'r-', lw=1, label="sensor")[0])
            
            ax.grid(b=True, which='major', color='gray', linestyle='-')
            ax.set_ylim(-5, 5)
            # ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1), ncol=3, fancybox=True, shadow=True)
            ax.set_facecolor((0.0, 0.0, 0.0, 0.0))
            ax.tick_params(axis='x', colors='k');[t.set_color('gray') for t in ax.xaxis.get_ticklabels()]
            ax.tick_params(axis='y', colors='k');[t.set_color('gray') for t in ax.yaxis.get_ticklabels()]

    def init_plot3d(self):
        """Sets up the GUI layout with the 3D plot placeholder."""
        # TODO:
        # [ ]: Implement using MayaVi2 instead of Matplotlib for better performance

        self.plot3d_fig = Figure(frameon=False, figsize=(5, 10))
        self.plot3d_fig.patch.set_color("w")
        self.plot3d_ax = self.plot3d_fig.add_subplot(111, projection='3d', frame_on=False)
        self.plot3d_canvas = FigureCanvas(self.plot3d_fig)

        # Transparent Background
        self.plot3d_ax.set_facecolor((0.0, 0.0, 0.0, 0.0))
        self.plot3d_canvas.setStyleSheet("background-color:transparent;")

        self.plot3d_fig.tight_layout()

        # Axis Styling
        self.plot3d_ax.tick_params(axis='x', colors='k');[t.set_color('gray') for t in self.plot3d_ax.xaxis.get_ticklabels()]
        self.plot3d_ax.tick_params(axis='y', colors='k');[t.set_color('gray') for t in self.plot3d_ax.yaxis.get_ticklabels()]
        self.plot3d_ax.tick_params(axis='z', colors='k');[t.set_color('gray') for t in self.plot3d_ax.zaxis.get_ticklabels()]

        self.plot3d_navi = NavigationToolbar(self.plot3d_canvas, self)
        layout = QVBoxLayout(self.plot3d_widget)
        layout.addWidget(self.plot3d_canvas)
        layout.addWidget(self.plot3d_navi)

        if fade_trajectory:
            self.line3d1 = []
            self.line3d2 = []

            for i in range(len(t10) - 1):
                    # Plot the original trajectory
                    self.line3d1.append(self.plot3d_ax.plot((0, 0), (0, 0), (0, 0), 'o-', markersize=5, color=(1, 0, 0, t10[i]/2), label='Original' if i == 0 else "")[0])

                    # Plot the expected trajectory
                    self.line3d2.append(self.plot3d_ax.plot((0, 0), (0, 0), (0, 0), 'o-', markersize=5, color=(0, 0, 1, t10[i]/2), label='Expected' if i == 0 else "")[0])
        else:
            self.line3d1 = self.plot3d_ax.plot(0, 0, 0, 'o-', markersize=5, color=(1, 0, 0, 0.2), label='Original')[0]
            self.line3d2 = self.plot3d_ax.plot(0, 0, 0, 'o-', markersize=5, color=(0, 0, 1, 0.2), label='Expected')[0]
            self.line3d1_main = self.plot3d_ax.plot(0, 0, 0, 'o-', markersize=5, color=(1, 0, 0, 1), label='Original')[0]
            self.line3d2_main = self.plot3d_ax.plot(0, 0, 0, 'o-', markersize=5, color=(0, 0, 1, 1), label='Expected')[0]
        
        self.plot3d_ax.set_xlim([-2, 2])
        self.plot3d_ax.set_ylim([-2, 2])
        self.plot3d_ax.set_zlim([-2, 2])
    
    def update_plot(self, message=False):
        if self.event.is_set():
            with self.shared_data["lock"]:
                joints_real = np.frombuffer(self.shared_data["joints_real"].get_obj(), dtype=np.float64).reshape(joints.shape)
                joints_expect = np.frombuffer(self.shared_data["joints_expect"].get_obj(), dtype=np.float64).reshape(joints.shape)
            
            for i in range(6):
                self.jline1[i].set_data(t, joints_real[i])
                self.jline2[i].set_data(t, joints_expect[i])
        
            self.canvas.draw()

            self.event.clear()  # Reset event
    
    def update_3d_plot(self):
        if self.event.is_set():
            with self.shared_data["lock"]:
                end_eff_real = np.frombuffer(self.shared_data["end_eff_real"].get_obj(), dtype=np.float64).reshape(xyz.shape)
                end_eff_expect = np.frombuffer(self.shared_data["end_eff_expect"].get_obj(), dtype=np.float64).reshape(xyz.shape)
            
            if fade_trajectory:
                for i in range(len(t10) - 1):
                    self.line3d1[i].set_data_3d(end_eff_real[0][i:i+2], end_eff_real[1][i:i+2], end_eff_real[2][i:i+2])
                    self.line3d2[i].set_data_3d(end_eff_expect[0][i:i+2], end_eff_expect[1][i:i+2], end_eff_expect[2][i:i+2])
            else:
                self.line3d1.set_data_3d(end_eff_real[0], end_eff_real[1], end_eff_real[2])
                self.line3d2.set_data_3d(end_eff_expect[0], end_eff_expect[1], end_eff_expect[2])
                self.line3d1_main.set_data_3d(end_eff_real[0][-1], end_eff_real[1][-1], end_eff_real[2][-1])
                self.line3d2_main.set_data_3d(end_eff_expect[0][-1], end_eff_expect[1][-1], end_eff_expect[2][-1])

            self.plot3d_canvas.draw()
            # self.event.clear()  # Reset event

    def mouseDoubleClickEvent(self, e):
        if self.isFullscreen:
            self.showMaximized()
            self.isFullscreen = False
        else:
            self.showFullScreen()
            self.isFullscreen = True
    
    def closeEvent(self, event):
        self.plot_process.terminate()
        self.plot_process.join()
        event.accept()


class PlotProcess(multiprocessing.Process):
    def __init__(self, shared_data, event, queue):
        super().__init__()
        self.shared_data = shared_data
        self.event = event  # Event to signal updates
        self.queue = queue

    def run(self):
        T0 = time.time()
        freq = 4 * np.pi / 100; A = 90

        counter = 0
        while True:
            while not self.queue.empty():
                data = self.queue.get()
                print(f"GUI received data: {data}")

                T = time.time() - T0
                x = A * np.sin(freq * (T - 0.1))
                y = A * np.cos(freq * (T - 0.1))
                z = T
                counter += 1
                with self.shared_data["lock"]:  # Ensure thread safety

                    joints_real = np.frombuffer(self.shared_data["joints_real"].get_obj(), dtype=np.float64).reshape(joints.shape)
                    joints_expect = np.frombuffer(self.shared_data["joints_expect"].get_obj(), dtype=np.float64).reshape(joints.shape)

                    # Shift values
                    joints_real[:] = np.roll(joints_real, 1, axis=1)
                    joints_expect[:] = np.roll(joints_expect, 1, axis=1)

                    # Update latest values
                    joints_real[:6, 0] = data['joints_r'][:6]
                    joints_expect[:6, 0] = data['joints_e'][:6]


                    if not counter%1:
                        # Convert shared memory to a NumPy array
                        end_eff_real = np.frombuffer(self.shared_data["end_eff_real"].get_obj(), dtype=np.float64).reshape(xyz.shape)
                        end_eff_expect = np.frombuffer(self.shared_data["end_eff_expect"].get_obj(), dtype=np.float64).reshape(xyz.shape)

                        # Shift values
                        end_eff_real[:] = np.roll(end_eff_real, -1, axis=1)
                        end_eff_expect[:] = np.roll(end_eff_expect, -1, axis=1)

                        # Update latest values
                        end_eff_real[:3, -1] = data['endeff_r']
                        end_eff_expect[:3, -1] = data['endeff_e']

                        counter = 0
                self.event.set()  # Notify GUI to update
            time.sleep(0.1)


if __name__ == "__main__":
    with multiprocessing.Manager() as manager:
        shared_data = {
            "end_eff_real": multiprocessing.Array('d', xyz.shape[0]*xyz.shape[1]),
            "end_eff_expect": multiprocessing.Array('d', xyz.shape[0]*xyz.shape[1]),
            "joints_real": multiprocessing.Array('d', joints.shape[0]*joints.shape[1]),
            "joints_expect": multiprocessing.Array('d', joints.shape[0]*joints.shape[1]),
            "lock": manager.Lock()
        }
        event = multiprocessing.Event()  # For notifying updates

        # app = QtWidgets.QApplication([])
        app = QApplication(sys.argv)
        app.setStyle("Fusion")
        window = MainWindow(shared_data, event)
        window.show()
        # app.exec_()
        sys.exit(app.exec_())
