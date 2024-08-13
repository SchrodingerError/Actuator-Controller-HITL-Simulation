import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from multiprocessing import Process, Manager

class LivePlot:
    def __init__(self, mult=1, max_data_points=100):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6))
        self.fig = fig
        self.ax1 = ax1
        self.ax2 = ax2
        self.ax3 = ax3

        self.mult = mult

        self.ax1.set_ylabel('Plot 1')
        self.ax2.set_ylabel('Plot 2')
        self.ax3.set_ylabel('Plot 3')
        self.ax3.set_xlabel('Time')

        self.max_data_points = max_data_points
        self.t = 0
        self.x_data = []
        self.y_data1 = []
        self.y_data1_2 = []
        self.y_data2 = []
        self.y_data3 = []

        # Initialize lines (empty initially)
        self.line1, = self.ax1.plot([], [], label='Plot 1')
        self.line1_2, = self.ax1.plot([], [], label='Plot 1.2')
        self.line2, = self.ax2.plot([], [], label='Plot 2')
        self.line3, = self.ax3.plot([], [], label='Plot 3')

        self.ax1.legend()
        self.ax2.legend()
        self.ax3.legend()

    def generate_random_data(self):
        return random.randint(1, 100)

    def update_data_external(self):
        # Simulate external updates (replace with your actual data update mechanism)
        new_x = self.t
        self.t += 1
        new_y1 = self.generate_random_data() * self.mult
        new_y2 = self.generate_random_data() * self.mult
        new_y3 = self.generate_random_data() * self.mult

        self.x_data.append(new_x)
        self.y_data1.append(new_y1)
        self.y_data1_2.append(new_y1 * -1)  # Example modification of data
        self.y_data2.append(new_y2)
        self.y_data3.append(new_y3)

        # Keep only the last `max_data_points` data points
        self.x_data = self.x_data[-self.max_data_points:]
        self.y_data1 = self.y_data1[-self.max_data_points:]
        self.y_data1_2 = self.y_data1_2[-self.max_data_points:]
        self.y_data2 = self.y_data2[-self.max_data_points:]
        self.y_data3 = self.y_data3[-self.max_data_points:]

    def update_plot(self, i):
        self.update_data_external()

        # Update plot data
        self.line1.set_data(self.x_data, self.y_data1)
        self.line1_2.set_data(self.x_data, self.y_data1_2)
        self.line2.set_data(self.x_data, self.y_data2)
        self.line3.set_data(self.x_data, self.y_data3)

        # Adjust plot limits (x-axis)
        if len(self.x_data) > 1:
            self.ax1.set_xlim(self.x_data[0], self.x_data[-1])
            self.ax2.set_xlim(self.x_data[0], self.x_data[-1])
            self.ax3.set_xlim(self.x_data[0], self.x_data[-1])

        # Adjust plot limits (y-axis) - optional
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        self.ax3.relim()
        self.ax3.autoscale_view()

        #return self.line1, self.line2, self.line3

    def animate(self):
        anim = animation.FuncAnimation(self.fig, self.update_plot, frames=1000, interval=10, blit=False)
        plt.show()
        #anim.save(filename="test.mp4")

# Function to run animation in a separate process
def run_animation(fig, ax1, ax2, ax3):
    #live_plot = LivePlot(fig, ax1, ax2, ax3)
    #live_plot.animate()
    pass

# Example usage with multiprocessing
if __name__ == '__main__':
    plot1 = LivePlot(mult=1)
    plot2 = LivePlot(mult=5)

    process1 = Process(target=plot1.animate)
    process2 = Process(target=plot2.animate)

    process1.start()
    process2.start()

    process1.join()
    process2.join()
