import time
from mcculw.ul import ignore_instacal

import threading

from Physics_Elements import Joint, Spring, StateSpace
from MCC_Interface import MCC3114, MCC202

from typing import List
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from Sensor_Elements import LoadCell, SharedLoadCellJoint, LoadCellSpring, SharedLoadCellSpring, DisplacementSensor
    from Controller_Input_Elements import RigidActuatorController

class HITLController():
    def __init__(self, stop_event, settings: dict):
        self.stop_event = stop_event
        self.settings = settings

        self.parse_settings()
        
        self.load_cells: List[LoadCell] = []
        #self.displacement_actuators: List[DisplacementSensor] = []
        self.displacement_sensors: List[DisplacementSensor] = []

        self.rigid_actuator_controllers: List[RigidActuatorController] = []

    def parse_settings(self):
        '''Parse the settings to get the settings for the MCC devices'''
        self.pull_from_sim_period = self.settings["pull_from_sim_period"] / 1000.0
        self.updated_plotting_shared_attributes_period = self.settings["plotting_update_period"] / 1000.0

    def attach_load_cell(self, load_cell: 'LoadCell'):
        self.load_cells.append(load_cell)

    def attach_displacement_sensor(self, sensor: 'DisplacementSensor'):
        self.displacement_sensors.append(sensor)

    def attach_rigid_actuator_controller(self, controller: 'RigidActuatorController'):
        self.rigid_actuator_controllers.append(controller)

    def update_from_sim(self):
        # Updates the local sensors based on the sim values
        for lc in self.load_cells:
            lc.pull_from_sim()
        # for rigid_actuator in self.displacement_actuators:
        #     rigid_actuator.pull_from_sim()
        for sensor in self.displacement_sensors:
            sensor.pull_from_sim()

    def update_from_sim_thread(self):
        while self.spare_stop_event.is_set() == False:
            self.update_from_sim()
            time.sleep(self.pull_from_sim_period / 1000)

    def update_internal_sensor_attributes(self):
        '''Updates internal attributes. This should be updated as fast as possible because it generates
        sensor noise.'''
        for lc in self.load_cells:
            lc.update_internal_attributes()
        # for rigid_actuator in self.displacement_actuators:
        #     rigid_actuator.update_internal_attributes()
        for sensor in self.displacement_sensors:
            sensor.update_internal_attributes()
    
    def update_plotting_shared_attributes(self):
        for lc in self.load_cells:
            lc.update_shared_attributes()
        for rigid_actuator_controller in self.rigid_actuator_controllers:
            rigid_actuator_controller.update_shared_attributes()
        # for rigid_actuator in self.displacement_actuators:
        #     rigid_actuator.update_shared_attributes()
        for sensor in self.displacement_sensors:
            sensor.update_shared_attributes()

    def update_plotting_thread(self):
        while self.spare_stop_event.is_set() == False:
            self.update_plotting_shared_attributes()
            time.sleep(self.updated_plotting_shared_attributes_period / 1000)

    def update_mcc3114(self):
        for lc in self.load_cells:
            channel = lc.sensor_settings["output_channel"]
            voltage = lc.get_noisy_voltage_scalar()
            self.mcc3114.voltage_write(channel, voltage)
        for sensor in self.displacement_sensors:
            channel = sensor.sensor_settings["output_channel"]
            voltage = sensor.get_noisy_voltage_scalar()
            self.mcc3114.voltage_write(channel, voltage)

    def read_from_mcc202(self):
        for rigid_actuator_controller in self.rigid_actuator_controllers:
            input_channel:int = rigid_actuator_controller.get_input_channel()
            voltage = self.mcc202.voltage_read(channel=input_channel)
            digital_channel:int = rigid_actuator_controller.get_digital_channel()
            digital_command = self.mcc202.digital_read(channel=digital_channel)
            
            rigid_actuator_controller.set_digital_command(digital_command)
            rigid_actuator_controller.set_input_voltage(voltage)
            rigid_actuator_controller.set_controlled_attribute()

    def update_sim_targets(self):
        for rigid_actuator_controller in self.rigid_actuator_controllers:
            rigid_actuator_controller.update_sim_target()

    def controller_loop(self):
        while self.stop_event.is_set() == False:
            self.loop_time = time.time()

            # Update the internal attributes of the sensors
            self.update_internal_sensor_attributes()

            # Update MCC3114 (analog output)
            self.update_mcc3114()

            # Get readings from the MCC202
            self.read_from_mcc202()

            # Update the shared actuators for the sim to respond to
            self.update_sim_targets()
            

    def start_mcc(self):
        ignore_instacal() # ONLY CALL ONCE
        self.mcc3114 = MCC3114()
        self.mcc202 = MCC202()

    def run_process(self):
        self.start_mcc()

        # Make the threads for interacting with shared elements
        self.spare_stop_event = threading.Event()
        self.spare_stop_event.clear()

        updating_plotting_elements_thread = threading.Thread(target=self.update_plotting_thread)
        update_from_sim_thread = threading.Thread(target=self.update_from_sim_thread)

        updating_plotting_elements_thread.start()
        update_from_sim_thread.start()
        
        self.controller_loop()

        self.spare_stop_event.set()

        updating_plotting_elements_thread.join()
        update_from_sim_thread.join()

