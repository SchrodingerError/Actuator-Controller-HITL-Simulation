import numpy as np
import multiprocessing
from multiprocessing.managers import BaseManager

import time

# if TYPE_CHECKING:
from Visualization import Visualization, Plotter
from Simulation import BFSSimulation
from Physics_Elements import Joint, Spring, RigidActuator, SharedRigidActuator, SharedJoint, SharedSpring, SharedRigidActuator
from Object_Sharing import SharedFloat
from HITL_Controller import HITLController
from Sensor_Elements import SharedLoadCellJoint, LoadCellSpring, SharedLoadCellSpring, DisplacementSensor, SharedDisplacementSensor
from Controller_Input_Elements import RigidActuatorController, SharedRigidActuatorController




steel_elastic_modulus = 200E9 # 200GPa
gravity = 9.81 # m/s^2

# Stage setup
stage_mass = 500_000 # Mass in kilograms
stage_weight = stage_mass * gravity
max_thrust = 1.5E7 # Max thrust in Newtons. Currently 15_000_000 N
stage_diameter = 5.5 # meters
stage_height = 60 # Meters
wall_thickness = 0.03 # Meters
    
# Actuator setup
rope_area = 5E-4 # Area in m^2. Currently 5 cm^2
rope_force_at_neutral_actuator = (max_thrust - stage_weight) / 4.0

lateral_offset = 2 # Meters away from the stage wall
    
actuator_neutral_length = 2
actuator_min_length = 0.5
actuator_max_length = 2.5

actuator_controller_neutral_voltage = 0
actuator_controller_mps_per_volt = 1 # meters per second per volt for the RMS controller to command the sim

actuator_displacement_sensor_neutral_voltage = 0
actuator_displacement_sensor_neutral_length = 0

actuator_displacement_sensor_volts_per_meter = 3 # Volts per meter for the actuator displacement sensor to output
    

rope_volts_per_newton = 1E-6 # 1 volt is 1 million newtons

# Stage calculations for stiffness of rope supporting main mass
stage_wall_area = np.pi / 4.0 * (stage_diameter**2 - (stage_diameter - wall_thickness*2)**2)
stage_stiffness = steel_elastic_modulus * stage_wall_area / stage_height # k = EA/l
stage_unstretched_length = stage_height - stage_weight / stage_stiffness

# Actuator calculations
rope_stiffness = steel_elastic_modulus * rope_area / stage_height # Estimate due to stageheight != rope length but it is close enough
rope_unstretched_length = rope_force_at_neutral_actuator / rope_stiffness + 56.45
actuator_lateral_offset = stage_diameter / 2 + lateral_offset

actuator_angle = np.arctan(stage_height/actuator_lateral_offset)
actuator_parent_height = actuator_neutral_length * np.sin(actuator_angle)
actuator_parent_lateral_offset = actuator_neutral_length * np.cos(actuator_angle)

def sinusoid_force1(time):
    return 100_000*np.sin(0.1*time)
def sinusoid_force2(time):
    return 125_000*np.sin(0.2*time+0.3)

def rope_stiffness_func(length, unstretched_length):
    if length > unstretched_length:
        return rope_stiffness
    else:
        return 0

def stage_setup_1(_max_run_time=100):
    max_run_time = _max_run_time


    # SIM SETUP #############################################
    manager = BaseManager()
    BaseManager.register('SharedFloat', SharedFloat)
    BaseManager.register('SharedJoint', SharedJoint)
    BaseManager.register('SharedSpring', SharedSpring)
    BaseManager.register('SharedRigidActuator', SharedRigidActuator)

    BaseManager.register('SharedLoadCellJoint', SharedLoadCellJoint)
    BaseManager.register('SharedLoadCellSpring', SharedLoadCellSpring)

    BaseManager.register('SharedRigidActuatorController', SharedRigidActuatorController)

    BaseManager.register('SharedDisplacementSensor', SharedDisplacementSensor)

    
    manager.start()

    stop_event = multiprocessing.Event()
    stop_event.clear()

    shared_sim_time = manager.SharedFloat()
    shared_sim_time.set(0.0)

    
    physics_plotter = Plotter(stop_event=stop_event, plot_settings={
        "title": "Physics Plotter",
        "xlabel": "Time (s)",
        "num_subplots": 5,
        "step_or_plot": "plot",
        "subplots": [
            {
                "ylabel": "Accel (m/s/s)",
            },
            {
                "ylabel": "Vel (m/s)",
            },
            {
                "ylabel": "Pos (m)",
            },
            {
                "ylabel": "Pos 2 (m)",
            },
            {
                "ylabel": "Force (N)",
            },
        ],
        "window_length": 100,
        "pull_interval": 10, # in milliseconds
        "update_interval": 100, # In milliseconds
        })
    physics_plotter.attach_shared_x(shared_sim_time)

    sensor_plotter = Plotter(stop_event=stop_event, plot_settings={
        "title": "Sensor Plotter",
        "xlabel": "Time (s)",
        "num_subplots": 4,
        "step_or_plot": "plot",
        "subplots": [
            {
                "ylabel": "Spring Force (N)",
            },
            {
                "ylabel": "Actuator Length (m)",
            },
            {
                "ylabel": "Actuator Position Output (V)",
            },
            {
                "ylabel": "Load Cell Output (V)",
            },
        ],
        "window_length": 100, # Max data points to keep on the plot
        "pull_interval": 10, # in milliseconds
        "update_interval": 100, # In milliseconds.
        })
    sensor_plotter.attach_shared_x(shared_sim_time)

    controller_plotter = Plotter(stop_event=stop_event, plot_settings={
        "title": "Controller Plotter",
        "xlabel": "Time (s)",
        "num_subplots": 3,
        "step_or_plot": "plot",
        "subplots": [
            {
                "ylabel": "Controller Input (V)",
            },
            {
                "ylabel": "Digital Input"
            },
            {
                "ylabel": "Controlled Vel (m/s)",
            },
        ],
        "window_length": 100, # Max data points to keep on the plot
        "pull_interval": 10, # in milliseconds
        "update_interval": 100, # In milliseconds.
        })
    controller_plotter.attach_shared_x(shared_sim_time)

    visualizer = Visualization(stop_event=stop_event, scene_settings={
        "canvas_width": 1600,
        "canvas_height": 1000,
        "wall_thickness": 0.1,
        "cube_size": 100
    })

    hitl_controller = HITLController(stop_event=stop_event, settings={
        "pull_from_sim_period": 5, # Pull values for the sensors every XXX milliseconds. Shouldn't be faster than the update_period for the sim
        "plotting_update_period": 10, # Update the shared attributes (for plotting) every YYY milliseconds
    })


    # SIM ELEMENTS SETUP
    # Changes height to 59.78258707863 rather than 60m to that it is in equilibrium
    main_mass = Joint(np.array([0, 60, 0]), mass=stage_mass, fixed=False, name="Stage Mass", integration_method="adams-bashforth")
    main_mass_shared:SharedJoint = manager.SharedJoint()
    main_mass.attach_shared_attributes(main_mass_shared)
    main_mass.add_damping(mom_ratio=1.5)
    main_mass.add_gravity()
    main_mass.add_variable_force([sinusoid_force1, None, sinusoid_force2])
    physics_plotter.attach_shared_attributes(main_mass_shared, plot_settings={
        "accel": {
            "x": {
                "subplot": 0,
                "ylabel": "Main Mass x-Accel",
                "color": "r"
            },
            "y": {
                "subplot": 0,
                "ylabel": "Main Mass y-Accel",
                "color": "g"
            },
            "z": {
                "subplot": 0,
                "ylabel": "Main Mass z-Accel",
                "color": "b"
            },
        },
        "vel": {
            "x": {
                "subplot": 1,
                "ylabel": "Main Mass x-Vel",
                "color": "r"
            },
            "y": {
                "subplot": 1,
                "ylabel": "Main Mass y-Vel",
                "color": "g"
            },
            "z": {
                "subplot": 1,
                "ylabel": "Main Mass z-Vel",
                "color": "b"
            }
        },
        "pos": {
            "x": {
                "subplot": 2,
                "ylabel": "Main Mass x-Pos",
                "color": "r"
            },
            "y": {
                "subplot": 3,
                "ylabel": "Main Mass y-Pos",
                "color": "g"
            },
            "z": {
                "subplot": 2,
                "ylabel": "Main Mass z-Pos",
                "color": "b"
            }
        }
    })
    visualizer.attach_shared_attributes(main_mass_shared, object_settings={
        "type": "sphere",
        "color": "red",
        "radius": stage_diameter/2.0
    })

    support_spring_joint = Joint(np.array([0, stage_height*2, 0]), mass=0, fixed=True, name="Support Spring Joint")

    support_spring = Spring(parent_joint=main_mass, child_joint=support_spring_joint, unstretched_length=stage_unstretched_length, constant_stiffness=stage_stiffness, name="Support Spring")
    support_spring_shared_attributes:SharedSpring = manager.SharedSpring()
    support_spring.attach_shared_attributes(support_spring_shared_attributes)
    physics_plotter.attach_shared_attributes(support_spring_shared_attributes, plot_settings={
        "force": {
            "y": {
                "subplot": 4,
                "ylabel": "Support Spring y-Force",
                "color": "w"
            }
        }
    })


    # Setting up the actuators and ropes
    for i in range(4):
        if i == 0:
            floor_x_pos = actuator_lateral_offset
            floor_z_pos = 0
            parent_x_pos = actuator_lateral_offset - actuator_parent_lateral_offset
            parent_z_pos = 0
            name = "East Actuator"
            name2 = "East Spring"
            color = "r"
        elif i == 1:
            floor_x_pos = 0
            floor_z_pos = actuator_lateral_offset
            parent_x_pos = 0
            parent_z_pos = actuator_lateral_offset - actuator_parent_lateral_offset
            name = "South Actuator"
            name2 = "South Spring"
            color = "g"
        elif i == 2:
            floor_x_pos = -1*actuator_lateral_offset
            floor_z_pos = 0
            parent_x_pos = -1*actuator_lateral_offset + actuator_parent_lateral_offset
            parent_z_pos = 0
            name = "West Actuator"
            name2 = "West Spring"
            color = "b"
        elif i == 3:
            floor_x_pos = 0
            floor_z_pos = -1*actuator_lateral_offset
            parent_x_pos = 0
            parent_z_pos = -1*actuator_lateral_offset + actuator_parent_lateral_offset
            name = "North Actuator"
            name2 = "North Spring"
            color = "w"

        actuator_spring_joint = Joint(np.array([parent_x_pos, actuator_parent_height, parent_z_pos]), mass=0.001, fixed=False, name=f"{name} Spring Joint")
        actuator_floor_joint = Joint(np.array([floor_x_pos, 0, floor_z_pos]), mass=0.001, fixed=False, name=f"{name} Floor Joint")

        #rope = Spring(parent_joint=main_mass, child_joint=actuator_spring_joint, unstretched_length=rope_unstretched_length, constant_stiffness=rope_stiffness, name=name2)
        rope = Spring(parent_joint=main_mass, child_joint=actuator_spring_joint, unstretched_length=rope_unstretched_length, stiffness_func=rope_stiffness_func, name=name2)
        rope_shared:SharedSpring = manager.SharedSpring()
        rope.attach_shared_attributes(rope_shared)
        visualizer.attach_shared_attributes(rope_shared, object_settings={
            "type": "helix",
            "color": "white",
            "radius": 0.1,
            "thickness": 0.1
        })

        rope_lc = LoadCellSpring(sensor_settings={
            "name": f"{name2} LC",
            "mV/V": 1000,
            "excitation": 10,
            "full_scale_force": (1/rope_volts_per_newton * 10),
            "output_channel": i*4 + 3,
            "noise_settings": {
                "static_error_band": 0.15,      # % of FSO
                "non-linearity": 0.15,          # % of FSO
                "hysteresis": 0.07,             # % of FSO
                "repeatability": 0.02,          # % of FSO
                "thermal_error": 0.005,         # % of reading per degree F
                "temperature_offset": 10        # Degrees F off from calibration temperature 
            }
        })
        rope_lc.attach_sim_source(rope_shared)
        rope_lc_shared_attributes = manager.SharedLoadCellSpring()
        rope_lc.attach_shared_attributes(rope_lc_shared_attributes)
        hitl_controller.attach_load_cell(rope_lc)
        sensor_plotter.attach_shared_attributes(rope_lc_shared_attributes, plot_settings={
            "force": {
                "scalar": {
                    "subplot": 0,
                    "ylabel": f"{name2} LC Force",
                    "color": color
                }
            },
            "noisy_voltage": {
                "scalar": {
                    "subplot": 3,
                    "ylabel": f"{name2} LC Noisy-Voltage",
                    "color": color
                }
            },
        })  

        actuator = RigidActuator(actuator_spring_joint, actuator_floor_joint, name=name, max_length=actuator_max_length, min_length=actuator_min_length, control_code=1)
        actuator_shared_attributes:SharedRigidActuator = manager.SharedRigidActuator()
        actuator.attach_shared_attributes(actuator_shared_attributes)
        visualizer.attach_shared_attributes(actuator_shared_attributes, object_settings={
            "type": "cylinder",
            "color": "black",
            "radius": 0.3
        })

        shared_actuator_spring_joint: SharedJoint = manager.SharedJoint()
        actuator_spring_joint.attach_shared_attributes(shared_actuator_spring_joint)

        shared_actuator_floor_joint: SharedJoint = manager.SharedJoint()
        actuator_floor_joint.attach_shared_attributes(shared_actuator_floor_joint)

        actuator_disp_sensor = DisplacementSensor(parent_joint=shared_actuator_spring_joint, child_joint=shared_actuator_floor_joint, sensor_settings={
            "name": f"{name} Sensor",
            "output_channel": i*4 + 2,
            "volts_per_meter": actuator_displacement_sensor_volts_per_meter,
            "neutral_length": actuator_displacement_sensor_neutral_length, # Length at neutral voltage
            "neutral_voltage": actuator_displacement_sensor_neutral_voltage,
            "max_length": actuator_max_length,
            "noise_settings": {
                "static_error_band": 0.15,      # % of FSO
                "non-linearity": 0.15,          # % of FSO
                "hysteresis": 0.07,             # % of FSO
                "repeatability": 0.02,          # % of FSO
                "thermal_error": 0.005,         # % of reading per degree F
                "temperature_offset": 10        # Degrees F off from calibration temperature 
            }
        })
        actuator_disp_sensor_shared:SharedDisplacementSensor = manager.SharedDisplacementSensor()
        actuator_disp_sensor.attach_shared_attributes(actuator_disp_sensor_shared)
        sensor_plotter.attach_shared_attributes(actuator_disp_sensor_shared, plot_settings={
            "length": {
                    "scalar": {
                        "subplot": 1,
                        "ylabel": f"{name} Length",
                        "color": color
                    }
            },
            "voltage": {
                "noisy": {
                    "subplot": 2,
                    "ylabel": f"{name} Noisy-Voltage",
                    "color": color
                }
            },
        })
        hitl_controller.attach_displacement_sensor(actuator_disp_sensor)

        actuator_controller = RigidActuatorController(controller_settings={
            "name": "Main Actuator Controller",
            "analog_input_channel": i,
            "digital_input_channel": i,
            "units_per_volt": actuator_controller_mps_per_volt, # How far the actuator moves per input volt (m/s or m/s/s if controlling vel or accel)
            "neutral_length": actuator_neutral_length, # Length at neutral voltage
            "neutral_voltage": actuator_controller_neutral_voltage, # Voltage to go to neutral length
            "controls_pos/vel/accel": 1, # 0 = controls pos, 1 = controls vel, 2 = controls accel,
            "max_length": actuator_max_length,
            "min_length": actuator_min_length
        })
        actuator_controller_shared_attributes = manager.SharedRigidActuatorController() # For plotting
        actuator_controller.attach_shared_attributes(actuator_controller_shared_attributes)

        actuator_controller.attach_sim_target(actuator_shared_attributes) # So the controller can update sim elements

        hitl_controller.attach_rigid_actuator_controller(actuator_controller)
        controller_plotter.attach_shared_attributes(actuator_controller_shared_attributes, plot_settings={
            "input_voltage": {
                "scalar": {
                    "subplot": 0,
                    "ylabel": f"{name} Controller Voltage",
                    "color": color
                }
            },
            "digital_input": {
                "scalar": {
                    "subplot": 1,
                    "ylabel": f"{name} Controller CTRL Input",
                    "color": color
                }
            },
            "vel": {
                "x": {
                    "subplot": 2,
                    "ylabel": f"{name} Controlled x-Vel",
                    "color": color
                }
            }
        })


    # PROCESSES #############################################################
    # Make the simulation process
    simulation = BFSSimulation(parent_joint=main_mass, settings={
        "duration": max_run_time,
        "delta_t": None,
        "shared_update_period": 0.1, # not used
        "plotting_update_period": 0.01,
        "sensor_update_period": 0.01,
        "controller_pull_period": 0.01
    })
    simulation.attach_shared_time(shared_sim_time)
    simulation_process = multiprocessing.Process(target=simulation.run_process)

    # Start the processes
    # Start the visualization process
    visualization_process = multiprocessing.Process(target=visualizer.run_process)
    visualization_process.start()
    
    # Start the physics_plotter process
    physics_plotter_process = multiprocessing.Process(target=physics_plotter.run_process)
    physics_plotter_process.start()

    # Start the sensor physics_plotter process
    sensor_plotter_process = multiprocessing.Process(target=sensor_plotter.run_process)
    sensor_plotter_process.start()

    # Start the sensor physics_plotter process
    controller_plotter_process = multiprocessing.Process(target=controller_plotter.run_process)
    controller_plotter_process.start()

    # Start the HITL interface process
    hitl_process = multiprocessing.Process(target=hitl_controller.run_process)
    hitl_process.start()

    time.sleep(5)
    simulation_process.start()

    # Join the processes
    simulation_process.join()
    stop_event.set()

    visualization_process.join()
    physics_plotter_process.join()
    sensor_plotter_process.join()
    controller_plotter_process.join()
    hitl_process.join()

    # Close the manager
    manager.shutdown()


def main():
    max_run_time = 10000
    #multiprocessed_double_mass_spring(100)
    #single_spring(max_run_time)

    stage_setup_1(max_run_time)


if __name__ == "__main__":
    main()

