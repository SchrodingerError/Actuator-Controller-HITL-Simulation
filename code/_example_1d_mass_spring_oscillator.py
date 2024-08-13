import numpy as np
import time
import multiprocessing
from multiprocessing.managers import BaseManager


from Visualization import Visualization, Plotter
from Simulation import BFSSimulation
from Physics_Elements import Joint, Spring, SharedJoint, SharedSpring
from Object_Sharing import SharedFloat





def main():
    '''Setting up the BaseManager so data can be shared between processes'''
    base_manager = BaseManager()

    BaseManager.register('SharedFloat', SharedFloat)
    BaseManager.register('SharedJoint', SharedJoint)
    BaseManager.register('SharedSpring', SharedSpring)

    base_manager.start()






    '''Creating instances for plotter and visualizer'''
    # Setup the plotter and visualization processes
    stop_event = multiprocessing.Event()
    stop_event.clear()

    # Create a synchronized time between the processes so the plotter and physics sim are in sync
    shared_sim_time:SharedFloat = base_manager.SharedFloat()
    shared_sim_time.set(0.0) # Set the initial time = 0.0 seconds


    # Create a real-time plotter for physics plotting. 1 for the mass and the other for the spring
    mass_plotter = Plotter(
        stop_event = stop_event,
        plot_settings = {
            "title": "Joint Plotter",
            "xlabel": "Time (s)",
            "num_subplots": 3,
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
                }
            ],
            "window_length": 100,               # Keep 100 seconds visible
            "pull_interval": 10,                # Pull data every 10 millisecond
            "update_interval": 100              # Update the graph every 100 milliseconds
        }
    )
    mass_plotter.attach_shared_x(shared_sim_time)

    spring_plotter = Plotter(
        stop_event = stop_event,
        plot_settings = {
            "title": "Spring Plotter",
            "xlabel": "Time (s)",
            "num_subplots": 2,
            "step_or_plot": "plot",
            "subplots": [
                {
                    "ylabel": "Length (m)"
                },
                {
                    "ylabel": "Force (N)"
                }
            ],
            "window_length": 100,               # Keep 100 seconds visible
            "pull_interval": 10,                # Pull data every 10 milliseconds
            "update_interval": 100              # Update the graph every 100 milliseconds
        }
    )
    spring_plotter.attach_shared_x(shared_sim_time)
    

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






    '''Setting up physics simulation'''
    # Create the physics elements
    main_mass = Joint(
        pos = np.array([0, 5, 0]),
        mass = 5,
        fixed = False,
        name = "Main mass",
        integration_method = "adams-bashforth"
    )

    # Before we create a spring, we need to create the Joint on the wall for the Spring to mount to
    wall_joint = Joint(
        pos = np.array([-10, 5, 0]),
        mass = 1,                           # Does not matter because it is fixed
        fixed = True,                       # We do not want it to ever move
        name = "Wall Joint"
    )


    spring = Spring(
        parent_joint = main_mass,
        child_joint = wall_joint,
        unstretched_length = 13,
        constant_stiffness = 100,
        name = "Spring"
    )






    '''Adding items to be plotted'''
    # Since we want to plot the physics of the mass and spring, we need shared attributes for them
    shared_main_mass: SharedJoint = base_manager.SharedJoint()
    main_mass.attach_shared_attributes(shared_main_mass)

    shared_spring: SharedSpring = base_manager.SharedSpring()
    spring.attach_shared_attributes(shared_spring)

    # Attach the shared elements to the plotter
    mass_plotter.attach_shared_attributes(
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

    spring_plotter.attach_shared_attributes(
        shared_attributes = shared_spring,
        plot_settings = {
            "length": {
                "scalar": {
                    "subplot": 0,
                    "ylabel": "Spring Length",
                    "color": "r"
                }
            },
            "force": {
                "x": {
                    "subplot": 1,
                    "ylabel": "Spring x-Force",
                    "color": "g"
                }
            }
        }
    )







    '''Adding items to be visualized'''
    # We aready have shared_main_mass and shared_spring from the plotting, so there is no need to make more shared elements
    visualizer.attach_shared_attributes(
        shared_attributes = shared_main_mass,
        object_settings = {
            "type": "sphere",
            "color": "red",
            "radius": 0.5,
        }
    )
    
    visualizer.attach_shared_attributes(
        shared_attributes = shared_spring,
        object_settings = {
            "type": "helix",
            "color": "white",
            "radius": 0.5,
            "thickness": 0.1
        }
    )




    

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
    mass_plotter_process = multiprocessing.Process(target=mass_plotter.run_process)
    spring_plotter_process = multiprocessing.Process(target=spring_plotter.run_process)
    visualization_process = multiprocessing.Process(target=visualizer.run_process)
    simulation_process = multiprocessing.Process(target=simulation.run_process)

    mass_plotter_process.start()
    spring_plotter_process.start()
    visualization_process.start()

    time.sleep(5)                           # Give the plotters and visualizer time to startup before the physics sim runs.

    # Join the process
    simulation_process.start()
    simulation_process.join()               # This blocks until the simulation finishes


    mass_plotter_process.join()
    spring_plotter_process.join()
    visualization_process.join()

    # Close the manager
    base_manager.shutdown()

if __name__ == "__main__":
    main()