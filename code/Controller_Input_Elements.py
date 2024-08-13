import numpy as np

from Physics_Elements import SharedRigidActuator

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from Object_Sharing import SharedFloat
    from Physics_Elements import StateSpace, Spring

class RigidActuatorController():
    def __init__(self, controller_settings: dict):
        self.controller_settings = controller_settings

        self.attached_sim_target: SharedRigidActuator = None

        self.shared_attributes: SharedRigidActuatorController = None # For plotting if desired

        self.parse_settings()

        self.input_voltage: float = 0.0
        self.digital_command: int = 0 # 0 means hold position, 1 means control using velocity

        self.controlled_length: float = 0.0
        self.controlled_vel_scalar: float = 0.0
        self.controlled_pos = np.array([0.0, 0.0, 0.0])
        self.controlled_vel = np.array([0.0, 0.0, 0.0])

    def parse_settings(self):
        self.name = self.controller_settings["name"]
        self.input_channel = self.controller_settings["analog_input_channel"]
        self.digital_channel = self.controller_settings["digital_input_channel"]
        self.max_length = self.controller_settings["max_length"]
        self.min_length = self.controller_settings["min_length"]
        self.neutral_length = self.controller_settings.get("neutral_length", 0)
        self.controls_pos_vel_accel = self.controller_settings["controls_pos/vel/accel"]

    def calc_neutral_pos(self):
        delta_pos_from_child = -1 * self.neutral_length * self.r_unit_vector
        self.neutral_pos = self.attached_sim_target.get_child_statespace().get_pos() + delta_pos_from_child

    def get_input_channel(self) -> int:
        return self.input_channel
    def get_digital_channel(self) -> int:
        return self.digital_channel

    def attach_sim_target(self, target: 'SharedRigidActuator'):
        if not isinstance(target._getvalue(), SharedRigidActuator):
            raise TypeError(f"target for RigidActuatorController {self.name} is not a SharedRigidActuator")
        
        self.attached_sim_target = target
        self.r_unit_vector = self.get_r_unit_vector()
        self.calc_neutral_pos()
        self.controlled_pos = self.neutral_pos
        self.attached_sim_target.set_parent_statespace_pos(self.controlled_pos)

    def attach_shared_attributes(self, shared_attributes: 'SharedRigidActuatorController'):
        if not isinstance(shared_attributes._getvalue(), SharedRigidActuatorController):
            raise TypeError(f"shared_attributes for RigidActuatorController {self.name} is not a SharedRigidActuatorController")
        self.shared_attributes = shared_attributes
        

    def get_r_unit_vector(self):
        return self.attached_sim_target.get_r_unit_vector()

    def set_input_voltage(self, voltage: float):
        '''After reading a voltage, set it to this controller element'''
        self.input_voltage = voltage

    def set_digital_command(self, command: int):
        self.digital_command = command

    def set_controlled_attribute(self):
        if self.controls_pos_vel_accel == 0: # Control pos
            self.controlled_length = (self.input_voltage - self.controller_settings["neutral_voltage"]) * self.controller_settings["units_per_volt"] + self.neutral_length
            self.set_controlled_pos(self.controlled_length)
        elif self.controls_pos_vel_accel == 1: # Controls vel
            self.set_controlled_vel()
        elif self.controls_pos_vel_accel == 2: # Controls accel
            self.set_controlled_accel()

    def set_controlled_vel(self):
        # Check if the controlled vel would put us outside our max or min displacement
            # if it is, set controlled vel to 0 and set our controlled pos manually
        current_length = self.attached_sim_target.get_length()
        self.controlled_vel_scalar = (self.input_voltage - self.controller_settings["neutral_voltage"]) * self.controller_settings["units_per_volt"]
        if current_length > self.max_length:
            self.set_controlled_pos(self.max_length)
            if self.controlled_vel_scalar > 0:
                self.controlled_vel_scalar = 0
        elif current_length < self.min_length:
            self.set_controlled_pos(self.min_length)
            if self.controlled_vel_scalar < 0:
                self.controlled_vel_scalar = 0
        self.r_unit_vector = self.get_r_unit_vector()
        controlled_vel = -1*self.r_unit_vector * self.controlled_vel_scalar # -1 * r vector becaus r vector describes child - parent
        self.controlled_vel = controlled_vel

    def set_controlled_accel(self): 
        pass

    def set_controlled_pos(self, length:float):
        self.r_unit_vector = self.get_r_unit_vector()
        if length > self.max_length:
            self.controlled_length = self.max_length
        elif length < self.min_length:
            self.controlled_length = self.min_length
        delta_pos_from_child = self.controlled_length * -1*self.r_unit_vector # -1 * r vector becaus r vector describes child - parent
        child_pos = self.attached_sim_target.get_child_statespace().get_pos()
        self.controlled_pos = delta_pos_from_child + child_pos

    def update_shared_attributes(self):
        if self.shared_attributes is not None:
            self.shared_attributes.set_digital_command(self.digital_command)
            self.shared_attributes.set_input_voltage(self.input_voltage)
            
            self.shared_attributes.set_controlled_pos(self.controlled_pos)
            self.shared_attributes.set_controlled_vel(self.controlled_vel)
            self.shared_attributes.set_controlled_length(self.controlled_length)
            self.shared_attributes.set_controlled_vel_scalar(self.controlled_vel_scalar)
        

    def update_sim_target(self):
        '''Update the sim target with the values we input from the controller'''
        if self.controls_pos_vel_accel == 0: # Control pos
            if self.digital_command == 1: # We are actively controlling the actuator
                self.attached_sim_target.set_parent_statespace_pos(self.controlled_pos)
        elif self.controls_pos_vel_accel == 1: # Controls vel
            if self.digital_command == 0: # We are NOT actively controlling the actuator
                self.attached_sim_target.set_parent_statespace_vel([0,0,0])
            else: # We are actively controlling the vel
                self.attached_sim_target.set_parent_statespace_vel(self.controlled_vel)
        elif self.controls_pos_vel_accel == 2: # Controls accel
            pass
        


class SharedRigidActuatorController():
    def __init__(self):
        self.input_voltage: float = 0.0
        self.digital_command: int = 0

        self.controlled_length: float = 0.0
        self.controlled_vel_scalar: float = 0.0
        self.controlled_pos = np.array([0.0, 0.0, 0.0])
        self.controlled_vel = np.array([0.0, 0.0, 0.0])

        self.connected_to_plotter: bool = False
        self.connected_to_visualization: bool = False
        self.connected_to_sensor: bool = False
        self.connected_to_controller: bool = False

    def set_input_voltage(self, voltage: float):
        self.input_voltage = voltage
    def set_digital_command(self, command: int):
        self.digital_command = command
    def set_controlled_pos(self, new_pos):
        self.controlled_pos = new_pos
    def set_controlled_vel(self, new_vel):
        self.controlled_vel = new_vel
    def set_controlled_length(self, disp: float) -> None:
        self.controlled_length = disp
    def set_controlled_vel_scalar(self, vel_scalar: float) -> None:
        self.controlled_vel_scalar = vel_scalar

    def set_connected_to_plotter(self, state: bool) -> None:
        self.connected_to_plotter = state
    def set_connected_to_visualization(self, state: bool) -> None:
        self.connected_to_visualization = state
    def set_connected_to_sensor(self, state: bool) -> None:
        self.connected_to_sensor = state
    def set_connected_to_controller(self, state: bool) -> None:
        self.connected_to_controller = state
    

    def get_connected_to_plotter(self) -> bool:
        return self.connected_to_plotter
    def get_connected_to_visualization(self) -> bool:
        return self.connected_to_visualization
    def get_connected_to_sensor(self) -> bool:
        return self.connected_to_sensor
    def get_connected_to_controller(self) -> bool:
        return self.connected_to_controller

    def get_controlled_pos(self):
        return self.controlled_pos
    def get_controlled_vel(self):
        return self.controlled_vel
    def get_controlled_length(self) -> float:
        return self.controlled_length
    def get_controlled_vel_scalar(self) -> float:
        return self.controlled_vel_scalar


    def get_input_voltage(self) -> float:
        return self.input_voltage
    def get_digital_command(self) -> int:
        return self.digital_command
