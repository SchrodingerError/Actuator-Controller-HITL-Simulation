import numpy as np
import time
import multiprocessing
from multiprocessing.managers import BaseManager


from Visualization import Visualization, Plotter
from Simulation import BFSSimulation
from Physics_Elements import Joint, Spring, RigidActuator, SharedJoint, SharedSpring, SharedRigidActuator

from HITL_Controller import HITLController
from Controller_Input_Elements import RigidActuatorController, SharedRigidActuatorController
from Sensor_Elements import DisplacementSensor, LoadCellSpring, SharedDisplacementSensor, SharedLoadCellSpring

from Object_Sharing import SharedFloat





def main():
    '''Setting up the BaseManager so data can be shared between processes'''
    base_manager = BaseManager()

    BaseManager.register('SharedFloat', SharedFloat)
    BaseManager.register('SharedJoint', SharedJoint)
    BaseManager.register('SharedSpring', SharedSpring)

    BaseManager.register('SharedRigidActuator', SharedRigidActuator)

    BaseManager.register('SharedRigidActuatorController', SharedRigidActuatorController)

    BaseManager.register('SharedDisplacementSensor', SharedDisplacementSensor)
    BaseManager.register('SharedLoadCellSpring', SharedLoadCellSpring)

    base_manager.start()






    '''Creating instances for plotter and visualizer'''
    # Setup the plotter and visualization processes
    stop_event = multiprocessing.Event()
    stop_event.clear()

    # Create a synchronized time between the processes so the plotter and physics sim are in sync
    shared_sim_time:SharedFloat = base_manager.SharedFloat()
    shared_sim_time.set(0.0) # Set the initial time = 0.0 seconds


    # Create a real-time plotter for physics plotting.
    physics_plotter = Plotter(
        stop_event = stop_event,
        plot_settings = {
            "title": "Physics Plotter",
            "xlabel": "Time (s)",
            "num_subplots": 4,
            "step_or_plot": "plot",
            "subplots": [
                {
                    "ylabel": "Position (m)"
                },
                {
                    "ylabel": "Velocity (m/s)"
                },
                {
                    "ylabel": "Accel (m/s/s)"
                },
                {
                    "ylabel": "Force (N)"
                },
            ],
            "window_length": 10,               # Keep 100 seconds visible
            "pull_interval": 10,                # Pull data every 10 millisecond
            "update_interval": 100              # Update the graph every 100 milliseconds
        }
    )
    physics_plotter.attach_shared_x(shared_sim_time)

    # Create a real-time plotter for the controller input plotting.
    controller_plotter = Plotter(
        stop_event = stop_event,
        plot_settings = {
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
                    "ylabel": "Controlled Pos (m)",
                },
            ],
            "window_length": 10,               # Keep 100 seconds visible
            "pull_interval": 10,                # Pull data every 10 millisecond
            "update_interval": 100              # Update the graph every 100 milliseconds
        }
    )
    controller_plotter.attach_shared_x(shared_sim_time)

    # Create a real-time plotter for the sensor output plotting
    sensor_plotter = Plotter(stop_event=stop_event, plot_settings={
        "title": "Sensor Plotter",
        "xlabel": "Time (s)",
        "num_subplots": 3,
        "step_or_plot": "plot",
        "subplots": [
            {
                "ylabel": "Actuator Length (m)",
            },
            {
                "ylabel": "Load Cell Force (N)",
            },
            {
                "ylabel": "Sensor Output (V)",
            },
        ],
        "window_length": 10, # Max data points to keep on the plot
        "pull_interval": 10, # in milliseconds
        "update_interval": 100, # In milliseconds.
        })
    sensor_plotter.attach_shared_x(shared_sim_time)

    # Create a 3D visualization for the entire model
    visualizer = Visualization(
        stop_event = stop_event,
        scene_settings = {
            "canvas_width": 1600,
            "canvas_height": 1000,
            "wall_thickness": 0.1,
            "cube_size": 20
        }
    )


    '''Creating an instance for the HITLController'''
    hitl_controller = HITLController(
        stop_event = stop_event,
        settings = {
            "pull_from_sim_period": 5,
            "plotting_update_period": 10
        }
    )



    '''Setting up physics simulation'''
    # Create the physics elements
    main_mass = Joint(
        pos = np.array([0, 10, 0]),
        mass = 5,
        fixed = False,
        name = "Main mass",
        integration_method = "adams-bashforth"
    )
    main_mass.add_damping(mom_ratio=0.0)

    # Since we want to plot the physics of the mass we need shared attributes for it
    shared_main_mass: SharedJoint = base_manager.SharedJoint()
    main_mass.attach_shared_attributes(shared_main_mass)
    physics_plotter.attach_shared_attributes(
        shared_attributes = shared_main_mass,
        plot_settings = {
            "pos": {
                "x": {
                    "subplot": 0,
                    "ylabel": "Main Poss x-Pos",
                    "color": "r"
                }
            },
            "vel": {
                "x": {
                    "subplot": 1,
                    "ylabel": "Main Poss x-Vel",
                    "color": "g"
                }
            },
            "accel": {
                "x": {
                    "subplot": 2,
                    "ylabel": "Main Poss x-Accel",
                    "color": "b"
                }
            }
        }
    )
    visualizer.attach_shared_attributes(
        shared_attributes = shared_main_mass,
        object_settings = {
            "type": "sphere",
            "color": "red",
            "radius": 0.5,
        }
    )
    
    # Before we create a spring, we need to create the Joint on the wall for the Spring to mount to
    north_wall_joint = Joint(
        pos = np.array([10, 15, 0]),
        mass = 1,                           # Does not matter because it is fixed
        fixed = True,                       # We do not want it to ever move
        name = "North Wall Joint"
    )

    north_spring = Spring(
        parent_joint = main_mass,
        child_joint = north_wall_joint,
        unstretched_length = 13,
        constant_stiffness = 100,
        name = "North Spring"
    )
    shared_north_spring: SharedSpring = base_manager.SharedSpring()
    north_spring.attach_shared_attributes(shared_north_spring)
    visualizer.attach_shared_attributes(
        shared_attributes = shared_north_spring,
        object_settings = {
            "type": "helix",
            "color": "white",
            "radius": 0.5,
            "thickness": 0.1
        }
    )

    west_wall_joint = Joint(
        pos = np.array([0, 15, -10]),
        mass = 1,                           # Does not matter because it is fixed
        fixed = True,                       # We do not want it to ever move
        name = "West Wall Joint"
    )

    west_spring = Spring(
        parent_joint = main_mass,
        child_joint = west_wall_joint,
        unstretched_length = 9,
        constant_stiffness = 100,
        name = "West Spring"
    )
    shared_west_spring: SharedSpring = base_manager.SharedSpring()
    west_spring.attach_shared_attributes(shared_west_spring)
    visualizer.attach_shared_attributes(
        shared_attributes = shared_west_spring,
        object_settings = {
            "type": "helix",
            "color": "white",
            "radius": 0.5,
            "thickness": 0.1
        }
    )

    # Setting up the actuator. This requires the "grounded_joint" to fix to. We also want a spring in between the actuator and main_mass
    actuator_joint = Joint(
        pos = np.array([-5, 10, 0]),
        mass = 1,                           # Does not matter because it is fixed to an actuator
        fixed = False,                      # We do not want it to ever move
        name = "Actuator Joint"
    )
    
    actuator_spring = Spring(
        parent_joint = main_mass,
        child_joint = actuator_joint,
        unstretched_length = 7,
        constant_stiffness = 150,
        name = "Actuator Spring"
    )
    shared_actuator_spring: SharedSpring = base_manager.SharedSpring()
    actuator_spring.attach_shared_attributes(shared_actuator_spring)
    physics_plotter.attach_shared_attributes(
        shared_attributes = shared_actuator_spring,
        plot_settings = {
            "force": {
                "scalar": {
                    "subplot": 3,
                    "ylabel": "Actuator Spring Force",
                    "color": "b"
                }
            },
        }
    )
    visualizer.attach_shared_attributes(
        shared_attributes = shared_actuator_spring,
        object_settings = {
            "type": "helix",
            "color": "white",
            "radius": 0.5,
            "thickness": 0.1
        }
    )

    south_wall_joint = Joint(
        pos = np.array([-10, 10, 0]),
        mass = 1,                           # Does not matter because it is fixed
        fixed = True,                       # We do not want it to ever move
        name = "South Wall Joint"
    )

    rigid_actuator = RigidActuator(
        parent_joint = actuator_joint,
        grounded_joint = south_wall_joint,
        name = "Rigid Actuator",
        max_length = 8,
        min_length = 2,
        control_code = 0                    # Position Control
    )
    shared_rigid_actuator: SharedRigidActuator = base_manager.SharedRigidActuator()
    rigid_actuator.attach_shared_attributes(shared_rigid_actuator)
    visualizer.attach_shared_attributes(
        shared_attributes = shared_rigid_actuator,
        object_settings = {
            "type": "cylinder",
            "color": "black",
            "radius": 0.5,
        }
    )

    # We have an actuator, but it currently does nothing without the RigidActuatorController
        # We will add the Controller and a DisplacementSensor for something in the real-world to control with
    rigid_actuator_controller = RigidActuatorController(
        controller_settings = {
            "name": "Rigid Actuator Controller",
            "analog_input_channel": 0,
            "digital_input_channel": 0,
            "units_per_volt": 2,                    # 2 meters per volt
            "neutral_length": 5,
            "neutral_voltage": 2.5,                 # At 2.5V, go to neutral_length of 5
            "controls_pos/vel/accel": 0,            # Controls position
            "max_length": 8,
            "min_length": 2
        }
    )
    rigid_actuator_controller.attach_sim_target(shared_rigid_actuator)
    hitl_controller.attach_rigid_actuator_controller(rigid_actuator_controller)

    shared_rigid_actuator_controller: SharedRigidActuatorController = base_manager.SharedRigidActuatorController()
    rigid_actuator_controller.attach_shared_attributes(shared_rigid_actuator_controller)
    controller_plotter.attach_shared_attributes(
        shared_attributes = shared_rigid_actuator_controller,
        plot_settings = {
            "input_voltage": {
                "scalar": {
                    "subplot": 0,
                    "ylabel": f"Actuator Controller Voltage",
                    "color": "r"
                }
            },
            "digital_input": {
                "scalar": {
                    "subplot": 1,
                    "ylabel": f"Actuator Controller CTRL Input",
                    "color": "r"
                }
            },
            "disp": {
                "scalar": {
                    "subplot": 2,
                    "ylabel": f"Actuator Controller Displacement",
                    "color": "r"
                }
            }
        }
    )

    # For the displacement sensor, we need 2 shared joints. 1 is the grounded joint of the actuator. the other is the parent
    shared_south_wall_joint: SharedJoint = base_manager.SharedJoint()
    south_wall_joint.attach_shared_attributes(shared_south_wall_joint)
    shared_actuator_joint: SharedJoint = base_manager.SharedJoint()
    actuator_joint.attach_shared_attributes(shared_actuator_joint)
    rigid_actuator_displacement_sensor = DisplacementSensor(
        parent_joint = shared_actuator_joint,
        child_joint = shared_south_wall_joint,
        sensor_settings = {
            "name": f"Actuator Displacement Sensor",
            "output_channel": 0,
            "volts_per_meter": 1,
            "neutral_length": 0,            
            "neutral_voltage": 0,           # Voltage at neutral length
            "max_length": 8,
        }
    )
    shared_rigid_actuator_displacement_sensor: SharedDisplacementSensor = base_manager.SharedDisplacementSensor()
    rigid_actuator_displacement_sensor.attach_shared_attributes(shared_rigid_actuator_displacement_sensor)
    sensor_plotter.attach_shared_attributes(shared_attributes=shared_rigid_actuator_displacement_sensor,
        plot_settings = {
            "length": {
                    "scalar": {
                        "subplot": 0,
                        "ylabel": f"Actuator Displacement Length",
                        "color": "r"
                    }
            },
            "voltage": {
                "true": {               # true or noisy because we didn't add noise
                    "subplot": 2,
                    "ylabel": f"Actuator Displacement Voltage",
                    "color": "r"
                }
            },
        }
    )
    hitl_controller.attach_displacement_sensor(rigid_actuator_displacement_sensor)


    '''Second actuator'''

    bottom_actuator_joint = Joint(
        pos = np.array([0, 5, 0]),
        mass = 1,                           # Does not matter because it is fixed to an actuator
        fixed = False,                      # We do not want it to ever move
        name = "Bottom Actuator Joint"
    )
    
    bottom_actuator_spring = Spring(
        parent_joint = main_mass,
        child_joint = bottom_actuator_joint,
        unstretched_length = 7.5,
        constant_stiffness = 1000,
        name = "Bottom Actuator Spring"
    )
    shared_bottom_actuator_spring: SharedSpring = base_manager.SharedSpring()
    bottom_actuator_spring.attach_shared_attributes(shared_bottom_actuator_spring)
    physics_plotter.attach_shared_attributes(
        shared_attributes = shared_bottom_actuator_spring,
        plot_settings = {
            "force": {
                "scalar": {
                    "subplot": 3,
                    "ylabel": "Bottom Actuator Spring Force",
                    "color": "r"
                }
            },
        }
    )
    visualizer.attach_shared_attributes(
        shared_attributes = shared_bottom_actuator_spring,
        object_settings = {
            "type": "helix",
            "color": "white",
            "radius": 0.5,
            "thickness": 0.1
        }
    )


    # LoadCell on the bottom spring
    bottom_spring_loadcell = LoadCellSpring(sensor_settings = {
        "name": "Bottom Spring LC",
        "mV/V": 1000,
        "excitation": 10,
        "full_scale_force": 5000,      # What is the max force on the load cell
        "output_channel": 8          # The output channel that the load cell outputs through the MCC3114. Must be between [0, 15]
        }
    )
    bottom_spring_loadcell.attach_sim_source(shared_bottom_actuator_spring)                   # Point the LoadCellSpring to pull physics values from the SharedSpring
    shared_bottom_spring_loadcell: SharedLoadCellSpring = base_manager.SharedLoadCellSpring()
    bottom_spring_loadcell.attach_shared_attributes(shared_bottom_spring_loadcell)
    hitl_controller.attach_load_cell(bottom_spring_loadcell)

    sensor_plotter.attach_shared_attributes(shared_attributes = shared_bottom_spring_loadcell,
        plot_settings = {
            "force": {
            "scalar": {
                "subplot": 1,     # Must be in range [0, num_subplots-1]
                "ylabel": "Bottom Spring LC-Force",
                "color": "g"        # White is default if not listed. "b", "g", "r", "c", "m", "y", "k", "w"
            },
        },
        "noisy_voltage": {
            "scalar": {
                "subplot": 2,     # Must be in range [0, num_subplots-1]
                "ylabel": "Bottom Spring LC-Voltage",
                "color": "g"        # White is default if not listed. "b", "g", "r", "c", "m", "y", "k", "w"
            }
        }
        }    
    )
    


    bottom_wall_joint = Joint(
        pos = np.array([0, 0, 0]),
        mass = 1,                           # Does not matter because it is fixed
        fixed = True,                       # We do not want it to ever move
        name = "Bottom Wall Joint"
    )

    bottom_rigid_actuator = RigidActuator(
        parent_joint = bottom_actuator_joint,
        grounded_joint = bottom_wall_joint,
        name = "Bottom Rigid Actuator",
        max_length = 10,
        min_length = 1,
        control_code = 1                    # Position Control
    )
    shared_bottom_rigid_actuator: SharedRigidActuator = base_manager.SharedRigidActuator()
    bottom_rigid_actuator.attach_shared_attributes(shared_bottom_rigid_actuator)
    visualizer.attach_shared_attributes(
        shared_attributes = shared_bottom_rigid_actuator,
        object_settings = {
            "type": "cylinder",
            "color": "black",
            "radius": 0.5,
        }
    )

    # We have an actuator, but it currently does nothing without the RigidActuatorController
        # We will add the Controller and a DisplacementSensor for something in the real-world to control with
    bottom_rigid_actuator_controller = RigidActuatorController(
        controller_settings = {
            "name": "Bottom Rigid Actuator Controller",
            "analog_input_channel": 1,
            "digital_input_channel": 2,
            "units_per_volt": 2,                    # 2 meters per volt
            "neutral_length": 1.5,
            "neutral_voltage": 2.5,                 # At 2.5V, go to neutral_length of 5
            "controls_pos/vel/accel": 1,            # Controls position
            "max_length": 10,
            "min_length": 1
        }
    )
    bottom_rigid_actuator_controller.attach_sim_target(shared_bottom_rigid_actuator)
    hitl_controller.attach_rigid_actuator_controller(bottom_rigid_actuator_controller)

    shared_bottom_rigid_actuator_controller: SharedRigidActuatorController = base_manager.SharedRigidActuatorController()
    bottom_rigid_actuator_controller.attach_shared_attributes(shared_bottom_rigid_actuator_controller)
    controller_plotter.attach_shared_attributes(
        shared_attributes = shared_bottom_rigid_actuator_controller,
        plot_settings = {
            "input_voltage": {
                "scalar": {
                    "subplot": 0,
                    "ylabel": f"Bottom Actuator Controller Voltage",
                    "color": "b"
                }
            },
            "digital_input": {
                "scalar": {
                    "subplot": 1,
                    "ylabel": f"Bottom Actuator Controller CTRL Input",
                    "color": "b"
                }
            },
            "disp": {
                "scalar": {
                    "subplot": 2,
                    "ylabel": f"Bottom Actuator Controller Displacement",
                    "color": "b"
                }
            }
        }
    )

    # For the displacement sensor, we need 2 shared joints. 1 is the grounded joint of the actuator. the other is the parent
    shared_bottom_wall_joint: SharedJoint = base_manager.SharedJoint()
    bottom_wall_joint.attach_shared_attributes(shared_bottom_wall_joint)
    shared_bottom_actuator_joint: SharedJoint = base_manager.SharedJoint()
    bottom_actuator_joint.attach_shared_attributes(shared_bottom_actuator_joint)
    bottom_rigid_actuator_displacement_sensor = DisplacementSensor(
        parent_joint = shared_bottom_actuator_joint,
        child_joint = shared_bottom_wall_joint,
        sensor_settings = {
            "name": f"Bottom Actuator Displacement Sensor",
            "output_channel": 1,
            "volts_per_meter": 1,
            "neutral_length": 0,            
            "neutral_voltage": 0,           # Voltage at neutral length
            "max_length": 8,
        }
    )
    shared_bottom_rigid_actuator_displacement_sensor: SharedDisplacementSensor = base_manager.SharedDisplacementSensor()
    bottom_rigid_actuator_displacement_sensor.attach_shared_attributes(shared_bottom_rigid_actuator_displacement_sensor)
    sensor_plotter.attach_shared_attributes(shared_attributes=shared_bottom_rigid_actuator_displacement_sensor,
        plot_settings = {
            "length": {
                    "scalar": {
                        "subplot": 0,
                        "ylabel": f"Bottom Actuator Displacement Length",
                        "color": "b"
                    }
            },
            "voltage": {
                "true": {               # true or noisy because we didn't add noise
                    "subplot": 2,
                    "ylabel": f"Bottom Actuator Displacement Voltage",
                    "color": "b"
                }
            },
        }
    )
    hitl_controller.attach_displacement_sensor(bottom_rigid_actuator_displacement_sensor)

    

    '''Create the physics simulation'''
    simulation = BFSSimulation(
        parent_joint = main_mass,           # Because spring is a child of main_mass, it will be reached by BFS
        settings = {
            "duration": 1000,               # Run the sim for 1000 seconds
            "delta_t": None,                # Run in real-time
            "plotting_update_period": 0.01, # Update the plottable elements every 0.01 seconds
            "sensor_update_period": 0.01,      # Update the sensor elements every second (because we have none, this doesn't matter)
            "controller_pull_period": 0.01     # Update the controller elements every seconds (again, we have none)
        }
    )
    simulation.attach_shared_time(shared_sim_time)
    




    '''Setup the processes and run them'''
    physics_plotter_process = multiprocessing.Process(target=physics_plotter.run_process)
    controller_plotter_process = multiprocessing.Process(target=controller_plotter.run_process)
    sensor_plotter_process = multiprocessing.Process(target=sensor_plotter.run_process)
    visualization_process = multiprocessing.Process(target=visualizer.run_process)
    simulation_process = multiprocessing.Process(target=simulation.run_process)

    hitl_controller_process = multiprocessing.Process(target=hitl_controller.run_process)
    
    physics_plotter_process.start()
    controller_plotter_process.start()
    sensor_plotter_process.start()
    visualization_process.start()

    hitl_controller_process.start()

    time.sleep(5)                           # Give the plotters and visualizer time to startup before the physics sim runs.

    # Join the process
    simulation_process.start()
    simulation_process.join()               # This blocks until the simulation finishes


    physics_plotter_process.join()
    controller_plotter_process.join()
    sensor_plotter_process.join()
    visualization_process.join()

    hitl_controller_process.join()

    # Close the manager
    base_manager.shutdown()

if __name__ == "__main__":
    main()