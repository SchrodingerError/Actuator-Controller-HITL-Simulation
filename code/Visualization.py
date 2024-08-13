import vpython
import time
from copy import deepcopy

import multiprocessing
import threading

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore


from Physics_Elements import Joint, Spring, SharedRigidActuator, SharedSpring, SharedJoint
from Sensor_Elements import SharedLoadCellJoint, SharedLoadCellSpring, SharedDisplacementSensor
from Controller_Input_Elements import SharedRigidActuatorController

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from Object_Sharing import SharedFloat


'''
TODO
2. Make this in a separate thread
'''
class Visualization():
    def __init__(self, stop_event, scene_settings:dict):
        self.stop_event = stop_event
        self.scene_settings = scene_settings

        self.attached_attributes = []
        self.shared_attributes_object_settings = []

        self.vpython_objects = []

    def draw_scene(self):
        vpython.scene = vpython.canvas(width=self.scene_settings["canvas_width"], height=self.scene_settings["canvas_height"])
        side = self.scene_settings["cube_size"]
        thk = self.scene_settings["wall_thickness"]

        wall_length = side + 2*thk

        

        wall_bottom = vpython.box(pos=vpython.vector(0, -thk/2, 0), size=vpython.vector(wall_length, thk, wall_length), color=vpython.color.gray(0.9))

        wall_left = vpython.box(pos=vpython.vector(-(side + thk)/2, side/2, 0), size=vpython.vector(thk, side, side), color=vpython.color.gray(0.7))
        wall_right = vpython.box(pos=vpython.vector((side + thk)/2, side/2, 0), size=vpython.vector(thk, side, side), color=vpython.color.gray(0.7))
        
        wall_back = vpython.box(pos=vpython.vector(0, (side)/2, -(side+ thk)/2), size=vpython.vector(wall_length, side, thk), color=vpython.color.gray(0.7))

    def attach_shared_attributes(self, shared_attributes, object_settings):
        self.attached_attributes.append(shared_attributes)
        self.shared_attributes_object_settings.append(object_settings)

        shared_attributes.set_connected_to_visualization(True)

    def generate_vpython_objects(self):
        for i in range(len(self.attached_attributes)):
            object_settings = self.shared_attributes_object_settings[i]
            
            if object_settings["type"] == "sphere":
                shared_attribute:Joint = self.attached_attributes[i]
                object_pos = deepcopy(shared_attribute.get_pos())
                object_pos = vpython.vector(object_pos[0], object_pos[1], object_pos[2])

                object_color = self.return_vpython_color(object_settings["color"])

                self.vpython_objects.append(vpython.sphere(pos=object_pos, radius=object_settings["radius"], color=object_color))
            elif object_settings["type"] == "helix":
                shared_attribute:Spring = self.attached_attributes[i]
                object_pos = deepcopy(shared_attribute.get_pos())
                object_pos = vpython.vector(object_pos[0], object_pos[1], object_pos[2])

                object_axis = deepcopy(shared_attribute.get_axis_vector())
                object_axis = vpython.vector(object_axis[0], object_axis[1], object_axis[2])

                object_color = self.return_vpython_color(object_settings["color"])

                self.vpython_objects.append(vpython.helix(pos=object_pos, axis=object_axis, radius=object_settings["radius"], thickness=object_settings["thickness"], color=object_color))
            elif object_settings["type"] == "cylinder":
                shared_attribute:Spring = self.attached_attributes[i]
                object_pos = deepcopy(shared_attribute.get_pos())
                object_pos = vpython.vector(object_pos[0], object_pos[1], object_pos[2])

                object_axis = deepcopy(shared_attribute.get_axis_vector())
                object_axis = vpython.vector(object_axis[0], object_axis[1], object_axis[2])

                object_color = self.return_vpython_color(object_settings["color"])

                self.vpython_objects.append(vpython.cylinder(pos=object_pos, axis=object_axis, radius=object_settings["radius"], color=object_color))
        
    def return_vpython_color(self, color_str):
        if color_str == "blue":
            return vpython.color.blue
        elif color_str == "black":
            return vpython.color.black
        elif color_str == "red":
            return vpython.color.red
        elif color_str == "green":
            return vpython.color.green
        elif color_str == "white":
            return vpython.color.white

    def update_scene(self):
        for i in range(len(self.vpython_objects)):
            element = self.vpython_objects[i]
            shared_attributes = self.attached_attributes[i]
            if isinstance(element, vpython.sphere):
                updated_pos = shared_attributes.get_pos()
                element.pos = deepcopy(vpython.vector(*(updated_pos)))
            elif isinstance(element, vpython.helix):
                updated_pos = shared_attributes.get_pos()
                element.pos = deepcopy(vpython.vector(*(updated_pos)))
                updated_axis = shared_attributes.get_axis_vector()
                element.axis = deepcopy(vpython.vector(*(updated_axis)))
            elif isinstance(element, vpython.cylinder):
                updated_pos = shared_attributes.get_pos()
                element.pos = deepcopy(vpython.vector(*(updated_pos)))
                updated_axis = shared_attributes.get_axis_vector()
                element.axis = deepcopy(vpython.vector(*(updated_axis)))

    def run_process(self):
        # Draw scene
        self.draw_scene()
        
        # Generate vpython objects from settings
        self.generate_vpython_objects()

        

        while self.stop_event.is_set() == False:
            # Update pos and axes of all vpython objects
            self.update_scene()



class Plotter():
    def __init__(self, plot_settings: dict, stop_event: multiprocessing.Event):
        self.plot_settings = plot_settings
        self.title = plot_settings["title"]
        self.window_length = self.plot_settings["window_length"]
        self.update_interval = self.plot_settings["update_interval"]
        self.pull_interval = self.plot_settings["pull_interval"]
        self.stop_event: multiprocessing.Event = stop_event

        self.attached_attributes = []
        self.shared_attributes_plot_settings = []

        self.pull_data_stop_event: threading.Event = None

        self.shared_x: SharedFloat = None

        self.plot_setup = {}

    def attach_shared_attributes(self, shared_attributes, plot_settings):
        self.attached_attributes.append(shared_attributes)
        self.shared_attributes_plot_settings.append(plot_settings)

        shared_attributes.set_connected_to_plotter(True)

    def attach_shared_x(self, shared_x):
        self.shared_x = shared_x
    
    def pull_data_thread(self):
        while self.pull_data_stop_event.is_set() == False:
            self.pull_data()
            time.sleep(self.pull_interval / 1000)

    def pull_data(self):
        self.raw_xdata.append(self.shared_x.get())

        for i in range(len(self.shared_attributes_plot_settings)):
            foo = self.attached_attributes[i]._getvalue()
            for key, value in self.shared_attributes_plot_settings[i].items():
                if isinstance(foo, SharedLoadCellJoint):
                    bar: SharedLoadCellJoint = self.attached_attributes[i]
                    if key == "force":
                        force_vector = bar.get_true_force_vector()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel: str = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[1])
                            elif subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[2])
                    elif key == "true_voltage":
                        voltage_scalar = bar.get_true_voltage_scalar()
                        for subkey, settings in value.items():
                            if subkey == "scalar":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(voltage_scalar)
                    elif key == "noisy_voltage":
                        voltage_scalar = bar.get_noisy_voltage_scalar()
                        for subkey, settings in value.items():
                            if subkey == "scalar":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(voltage_scalar)
                elif isinstance(foo, SharedLoadCellSpring):
                    bar: SharedLoadCellSpring = self.attached_attributes[i]
                    if key == "force":
                        force_vector = bar.get_true_force_vector()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[1])
                            elif subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[2])
                            elif subkey == "scalar":
                                force_scalar = bar.get_true_force_scalar()
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_scalar)
                    elif key == "true_voltage":
                        voltage_scalar = bar.get_true_voltage_scalar()
                        for subkey, settings in value.items():
                            if subkey == "scalar":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(voltage_scalar)
                    elif key == "noisy_voltage":
                        voltage_scalar = bar.get_noisy_voltage_scalar()
                        for subkey, settings in value.items():
                            if subkey == "scalar":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(voltage_scalar)
                elif isinstance(foo, SharedRigidActuator):
                    bar: SharedRigidActuator = self.attached_attributes[i]
                    if key == "pos":
                        pos_vector = bar.get_pos()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(pos_vector[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(pos_vector[1])
                            elif subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(pos_vector[2])
                    if key == "vel":
                        pos_vector = self.attached_attributes[i].get_vel()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(pos_vector[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(pos_vector[1])
                            elif subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(pos_vector[2])
                elif isinstance(foo, SharedRigidActuatorController):
                    bar: SharedRigidActuatorController = self.attached_attributes[i]
                    if key == "input_voltage":
                        input_voltage = bar.get_input_voltage()
                        for subkey, settings in value.items():
                            if subkey == "scalar":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(input_voltage)
                    elif key == "digital_input":
                        digital_input = bar.get_digital_command()
                        for subkey, settings in value.items():
                            if subkey == "scalar":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(digital_input)
                    elif key == "pos":
                        controlled_vel = bar.get_controlled_pos()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(controlled_vel[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(controlled_vel[1])
                            elif subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(controlled_vel[2])
                    elif key == "vel":
                        controlled_vel = bar.get_controlled_vel()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(controlled_vel[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(controlled_vel[1])
                            elif subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(controlled_vel[2])
                            if subkey == "scalar":
                                vel_scalar = bar.get_controlled_vel_scalar()
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(vel_scalar)
                    elif key == "disp":
                        disp_scalar = bar.get_controlled_length()
                        for subkey, settings in value.items():
                            if subkey == "scalar":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(disp_scalar)
                elif isinstance(foo, SharedDisplacementSensor):
                    bar: SharedDisplacementSensor = self.attached_attributes[i]
                    if key == "voltage":
                        for subkey, settings in value.items():
                            if subkey == "noisy":
                                voltage_scalar = bar.get_noisy_voltage()
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(voltage_scalar)
                            elif subkey == "true":
                                voltage_scalar = bar.get_true_voltage()
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(voltage_scalar)
                    if key == "disp":
                        disp_scalar = bar.get_displacement()
                        for subkey, settings in value.items():
                            if subkey == "scalar":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(disp_scalar)
                    elif key == "length":
                        length_scalar = bar.get_current_length()
                        for subkey, settings in value.items():
                            if subkey == "scalar":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(length_scalar)
                elif isinstance(foo, SharedSpring):
                    bar: SharedSpring = self.attached_attributes[i]
                    if key == "force":
                        force_vector = bar.get_spring_force()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[1])
                            elif subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[2])
                            elif subkey == "scalar":
                                force_scalar = bar.get_spring_force_scalar()
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_scalar)
                    elif key == "length":
                        length = bar.get_length()
                        for subkey, settings in value.items():
                            if subkey == "scalar":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(length)
                elif isinstance(foo, SharedJoint):
                    bar: SharedJoint = self.attached_attributes[i]
                    if key == "accel":
                        accel_vector = bar.get_accel()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(accel_vector[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(accel_vector[1])
                            elif subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(accel_vector[2])
                    elif key == "vel":
                        vel_vector = bar.get_vel()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(vel_vector[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(vel_vector[1])
                            elif subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(vel_vector[2])
                    elif key == "pos":
                        pos_vector = bar.get_pos()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(pos_vector[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(pos_vector[1])
                            if subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(pos_vector[2])
                    elif key == "force":
                        force_vector = bar.get_spring_force()
                        for subkey, settings in value.items():
                            if subkey == "x":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[0])
                            elif subkey == "y":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[1])
                            elif subkey == "z":
                                ylabel = settings["ylabel"]
                                (subplot, line) = self.raw_data_map[ylabel]
                                self.raw_ydata[subplot][line].append(force_vector[2])

                            
    def update_live_window(self):
        if self.stop_event.is_set():
            self.timer.stop()
            self.pull_data_stop_event.set()
            return
        
        final_time = self.raw_xdata[-1]
        target_time = final_time - self.window_length
        closest_index = min(range(len(self.raw_xdata)), key=lambda i: abs(self.raw_xdata[i] - target_time))
        self.window_xdata = self.raw_xdata[closest_index:]

        # Attach the data to the appropriate sublines in the subplots
        for i in range(self.num_plots):  
            for j in range(len(self.subplot_keys[i])):
                window_ydata = self.raw_ydata[i][j][closest_index:]
                if self.plot_settings["step_or_plot"] == "plot":
                    if len(self.window_xdata) == len(window_ydata):
                        self.subplot_curves[i][j].setData(self.window_xdata, window_ydata)
                    elif len(self.window_xdata) > len(window_ydata):
                        self.subplot_curves[i][j].setData(self.window_xdata[:-1], window_ydata)
                elif self.plot_settings["step_or_plot"] == "step":
                    if len(self.window_xdata) == len(window_ydata):
                        self.subplot_curves[i][j].setData(self.window_xdata, window_ydata[:-1])
                    elif len(self.window_xdata) > len(window_ydata):
                        self.subplot_curves[i][j].setData(self.window_xdata[:-1], window_ydata[:-1])

    def generate_plots(self):
        # Create the application
        self.app = QtWidgets.QApplication([])

        # Create a plot window
        self.win = pg.GraphicsLayoutWidget(show=True, title=self.title)
        self.win.resize(1000, 600)
        self.win.setWindowTitle(self.plot_settings["title"])

        self.num_plots = self.plot_settings["num_subplots"]

        self.plots = []

        for i in range(self.num_plots):
            plot = self.win.addPlot(title=self.plot_settings["subplots"][i]["ylabel"])
            plot.setLabel('left', self.plot_settings["subplots"][i]["ylabel"])
            plot.addLegend()
            self.plots.append(plot)

            # Move us to the next row for the subplot
            if i < self.num_plots - 1:
                self.win.nextRow()
        
    def generate_curves(self):
        self.subplot_keys = [[] for _ in range(self.num_plots)] # list for each subplot
        self.subplot_curves = [[] for _ in range(self.num_plots)] # list for each subplot
        self.raw_xdata = []
        self.raw_ydata = [[] for _ in range(self.num_plots)] # list for each subplot
        self.raw_data_map = {}
        for setting in self.shared_attributes_plot_settings:
            '''
            Each setting looks like {
                "accel": {
                    "x": {
                        "subplot": 1,
                        "ylabel": "Main Mass x-Accel"
                    },
                    "y": {
                        "subplot": 1,
                        "ylabel": "Main Mass y-Accel"
                    }
                },
                "vel": {
                    "x": {
                        "subplot": 2,
                        "ylabel": "Main Mass x-Vel"
                    }
                },
                "pos": {
                    "x": {
                        "subplot": 3,
                        "ylabel": "Main Mass x-Vel"
                    }
                }
            }
            '''
            for key, value in setting.items():
                ''' key would be like "accel"
                    value would be like {
                    "x": {
                        "subplot": 1,
                        "ylabel": "Main Mass x-Accel"
                    },
                    "y": {
                        "subplot": 1,
                        "ylabel": "Main Mass y-Accel"
                    }
                }
                '''
                for line, line_settings in value.items():
                    ''' line would be like "x"
                    line_settings would be like {
                        "subplot": 1,
                        "ylabel": "Main Mass x-Accel",
                        "color": "w"
                    }
                    '''
                    subplot = line_settings["subplot"]
                    label = line_settings["ylabel"]
                    color = line_settings.get("color", "w")  # Default to white if no color is specified
                    self.subplot_keys[subplot].append(label)
                    if self.plot_settings["step_or_plot"] == "plot":
                        self.subplot_curves[subplot].append(self.plots[subplot].plot(name=label, pen=pg.mkPen(color=color)))
                    elif self.plot_settings["step_or_plot"] == "step":
                        self.subplot_curves[subplot].append(self.plots[subplot].plot(name=label, stepMode=True, pen=pg.mkPen(color=color)))

                    self.raw_data_map[label] = (subplot, len(self.raw_ydata[subplot]))
                    self.raw_ydata[subplot].append([])

        self.window_xdata = []
        self.window_ydata = [[] for _ in range(self.num_plots)] # list for each subplot

    def show_plot(self):
        pass

    def run_process(self):
        # Generate the figure and subplots
        self.generate_plots()

        # Generate the curves
        self.generate_curves()

        self.pull_data_stop_event = threading.Event()
        self.pull_data_stop_event.clear()
        pull_data_thread = threading.Thread(target=self.pull_data_thread)
        pull_data_thread.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_live_window)
        self.timer.start(self.update_interval)

        # Start the Qt event loop
        QtWidgets.QApplication.instance().exec_()

        pull_data_thread.join()
        