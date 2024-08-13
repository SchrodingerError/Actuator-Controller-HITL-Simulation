import time
import threading
import numpy as np

from typing import List
from Visualization import Visualization
from Physics_Elements import Joint, Spring, RigidActuator

class BFSSimulation():
    def __init__(self, parent_joint: Joint, settings:dict):
        if not isinstance(parent_joint, Joint):
            raise TypeError(f"parent_joint for BFSSimulation is not a Joint")
        
        self.parent_joint = parent_joint
        self.duration = settings["duration"]
        self.delta_t = settings["delta_t"]
        self.plotting_update_period = settings["plotting_update_period"]
        self.sensor_update_period = settings["sensor_update_period"]
        self.controller_pull_period = settings["controller_pull_period"]

        self.joints: List[Joint] = []
        self.springs: List[Spring] = []
        self.actuators: List[RigidActuator] = []
        self.rigid_actuators: List[RigidActuator] = []

        self.get_joints_bfs()
        #self.get_parent_joints_for_every_joint()

        # Plotting or viz elements are updated slower than sensor
        self.plotting_or_viz_only_shared = []
        self.sensor_shared = []
        self.controller_shared = []
        self.sort_shared_attributes_into_frequencies()

        self.vis:Visualization = None

        self.attached_shared_time:float = None

        self.sim_time:float = 0.0

    def is_in_joints(self, joint: Joint):
        for j in self.joints:
            if joint.is_same(j):
                return True
        return False
    
    def is_in_queue(self, joint: Joint, queue):
        for j in queue:
            if joint.is_same(j):
                return True
        return False

    def get_joints_bfs(self):
        queue: List[Joint] = []

        queue.append(self.parent_joint)

        while queue:
            parent_joint: Joint
            parent_joint = queue.pop(0)
            if parent_joint is None:
                continue
            if not parent_joint.is_in_list(self.joints):
                self.joints.append(parent_joint)

            connected_springs = parent_joint.get_connected_springs()
            
            for spring in connected_springs:
                if not spring.is_in_list(self.springs):
                    self.springs.append(spring)
                child_joint = spring.get_child_joint()
                if not self.is_in_joints(child_joint) and not self.is_in_queue(child_joint, queue):
                    queue.append(child_joint)
            
            connected_actuators = parent_joint.get_connected_actuators()
            actuator: RigidActuator = None
            for actuator in connected_actuators:
                if actuator not in self.actuators:
                    self.rigid_actuators.append(actuator)
                    if actuator.get_child_joint() not in self.joints and actuator.get_child_joint() not in queue:
                        queue.append(actuator.get_child_joint())


    def get_parent_joints_for_every_joint(self):
        for joint in self.joints:
            joint.find_parent_joints()

    def get_child_joints_for_every_joint(self):
        for joint in self.joints:
            joint.find_child_joints()

    def update_actuators_from_controller(self):
        for rigid_actuator in self.rigid_actuators:
            rigid_actuator.update_from_controller()
    
    def pull_actuator_positions_thread(self):
        while self.spare_stop_event.is_set() == False:
            self.update_actuators_from_controller()
            time.sleep(self.controller_pull_period)

    def update_sensor_attributes(self):
        '''Updates the joints, springs, and actuators that are attached to sensors'''
        for obj in self.sensor_shared:
            obj.update_shared_attributes()

    def update_sensor_attributes_thread(self):
        while self.spare_stop_event.is_set() == False:
            self.update_sensor_attributes()
            time.sleep(self.sensor_update_period)

    def update_plotting_or_viz_only_attributes(self):
        '''Updates the joints, springs, and actuators that are only attached for plotting or visualization'''
        for obj in self.plotting_or_viz_only_shared:
            obj.update_shared_attributes()

    def update_plotting_or_viz_only_attributes_thread(self):
        while self.spare_stop_event.is_set() == False:
            self.update_plotting_or_viz_only_attributes()
            time.sleep(self.plotting_update_period)

    def sort_shared_attributes_into_frequencies(self):
        '''For each shared attribute connected to anything, put them in lists so we know how fast we need to update them.'''
        for joint in self.joints:
            shared = joint.shared_attributes
            if shared is None:
                continue
            plot = shared.get_connected_to_plotter()
            vis = shared.get_connected_to_visualization()
            sens = shared.get_connected_to_sensor()
            contr = shared.get_connected_to_controller()

            if sens:
                self.sensor_shared.append(joint)
            elif plot or vis:
                self.plotting_or_viz_only_shared.append(joint)
            
            if contr:
                self.controller_shared.append(joint)
        
        for spring in self.springs:
            shared = spring.shared_attributes
            if shared is None:
                continue
            plot = shared.get_connected_to_plotter()
            vis = shared.get_connected_to_visualization()
            sens = shared.get_connected_to_sensor()
            contr = shared.get_connected_to_controller()

            if sens:
                self.sensor_shared.append(spring)
            elif plot or vis:
                self.plotting_or_viz_only_shared.append(spring)
            
            if contr:
                self.controller_shared.append(spring)

        for actuator in self.rigid_actuators:
            shared = actuator.shared_attributes
            if shared is None:
                continue
            plot = shared.get_connected_to_plotter()
            vis = shared.get_connected_to_visualization()
            sens = shared.get_connected_to_sensor()
            contr = shared.get_connected_to_controller()

            if sens:
                self.sensor_shared.append(actuator)
            elif plot or vis:
                self.plotting_or_viz_only_shared.append(actuator)
            
            if contr:
                self.controller_shared.append(actuator)

    def attach_shared_time(self, attached_shared_time):
        self.attached_shared_time = attached_shared_time

    def run_process(self):

        time_values = []

        self.sim_time = 0
        #time_last = time.perf_counter()
        time_last = time.time()
        count = 0

        self.spare_stop_event = threading.Event()
        self.spare_stop_event.clear()
        pull_actuator_thread = threading.Thread(target=self.pull_actuator_positions_thread)
        pull_actuator_thread.start()
        update_plotting_or_viz_only_thread = threading.Thread(target=self.update_plotting_or_viz_only_attributes_thread)
        update_plotting_or_viz_only_thread.start()
        update_sensor_thread = threading.Thread(target=self.update_sensor_attributes_thread)
        update_sensor_thread.start()

        while self.sim_time < self.duration:
            count += 1
            if self.delta_t is None:
                #new_time = time.perf_counter()
                new_time = time.time()
                delta_time = new_time - time_last
                time_last = new_time
            
            for spring in self.springs:
                spring.calc_spring_force()
            
            joint: Joint
            for joint in self.joints:
                if joint.fixed == True:
                    continue
                # Get joint forces
                joint.calc_net_force(self.sim_time)
                # Get joint accels.
                joint.calc_accel()
                # Integrate joint vel and pos
                if self.delta_t is None:
                    joint.integrate_statespace(delta_time)
                else:
                    joint.integrate_statespace(self.delta_t)

            
            time_values.append(self.sim_time)

            if self.delta_t is None:
                self.sim_time += delta_time
            else:
                self.sim_time += self.delta_t

            # Update the shared sim time as fast as possible
            self.attached_shared_time.set(self.sim_time)
        
        self.spare_stop_event.set()
        pull_actuator_thread.join()
        update_plotting_or_viz_only_thread.join()
        update_sensor_thread.join()

        print(f"Average delta_t = {self.sim_time / count}")