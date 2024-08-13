import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import numpy as np

class RealTimePlot:
    def __init__(self, update_interval=1, max_display_time=10, num_subplots=2, line_styles=None):
        # Create the application
        self.app = QtWidgets.QApplication([])

        # Create a plot window
        self.win = pg.GraphicsLayoutWidget(show=True, title="Real-Time Plot")
        self.win.resize(1000, 600)

        # Add plots
        self.plots = []
        self.curves = []
        self.line_styles = line_styles if line_styles is not None else ['line'] * num_subplots
        for i in range(num_subplots):
            plot = self.win.addPlot(title=f"Subplot {i+1}")
            plot.setLabel('left', f'Subplot {i+1} Y-Axis')
            plot.addLegend()  # Add a legend to each plot
            self.plots.append(plot)

            if self.line_styles[i] == 'step':
                curve = plot.plot(pen='y', name=f'Subplot {i+1} Data', stepMode=True)
            else:
                curve = plot.plot(pen='y', name=f'Subplot {i+1} Data')
            
            self.curves.append(curve)
            if i < num_subplots - 1:
                self.win.nextRow()

        # Data buffers
        self.xdata = [np.empty(0) for _ in range(num_subplots)]
        self.ydata = [np.empty(0) for _ in range(num_subplots)]

        # Parameters
        self.update_interval = update_interval
        self.max_display_time = max_display_time
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_live_window)
        self.timer.start(update_interval)

    def update_live_window(self):
        for i in range(len(self.plots)):
            # Generate new data
            t = self.xdata[i][-1] + self.update_interval / 1000.0 if len(self.xdata[i]) > 0 else 0
            y = np.sin(2 * np.pi * t + i)  # Different phase for each subplot

            # Append new data to buffers
            self.xdata[i] = np.append(self.xdata[i], t)
            self.ydata[i] = np.append(self.ydata[i], y)

            # Remove old data to keep the buffer size within max_display_time
            if t > self.max_display_time:
                self.xdata[i] = self.xdata[i][self.xdata[i] > t - self.max_display_time]
                self.ydata[i] = self.ydata[i][-len(self.xdata[i]):]

            # Ensure correct lengths for step mode
            if self.line_styles[i] == 'step':
                xdata_step = np.append(self.xdata[i], self.xdata[i][-1] + self.update_interval / 1000.0)
                self.curves[i].setData(xdata_step, self.ydata[i])
            else:
                self.curves[i].setData(self.xdata[i], self.ydata[i])

    def run(self):
        # Start the Qt event loop
        QtWidgets.QApplication.instance().exec_()

# Parameters
update_interval = 100  # milliseconds
max_display_time = 10  # seconds
num_subplots = 2  # number of subplots
line_styles = ['line', 'step']  # specify 'line' or 'step' for each subplot

# Create and run the real-time plot
rt_plot = RealTimePlot(update_interval, max_display_time, num_subplots, line_styles)
rt_plot.run()
