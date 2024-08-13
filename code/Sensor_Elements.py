import numpy as np
from math import log, copysign

from Physics_Elements import SharedRigidActuator, SharedJoint, SharedSpring

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from Object_Sharing import SharedFloat
    from Physics_Elements import StateSpace, Spring



'''
TODO
1.
2. Better noise generation
    Non-linearity, hysteresis, background noise, AC noise
'''

class LoadCell():
    '''Provides net force feedback.'''
    def __init__(self, sensor_settings: dict={}):
        self.sensor_settings = sensor_settings

        self.attached_sim_source = None

        self.shared_attributes: SharedLoadCell = None # For plotting if desired

        self.true_force_vector = np.array([0.0, 0.0, 0.0]) # Current real force on the joint
        self.true_force_scalar: float = 0.0
        self.last_true_force_scalars: list[float] = []           # Stores the last true force for hysteresis reasons
        self.true_voltage_scalar:float = 0.0

        self.noisy_force_vector = np.array([0.0, 0.0, 0.0])
        self.noisy_force_scalar: float = 0.0
        self.noisy_voltage_scalar: float = 0.0

        self.parse_settings()

        self.calc_noise_parameters()

    def parse_settings(self):
        self.name = self.sensor_settings["name"]
        self.scale = self.sensor_settings["mV/V"] / 1000.0
        self.excitation = self.sensor_settings["excitation"]
        self.full_scale_force = self.sensor_settings["full_scale_force"]
        self.output_channel = self.sensor_settings["output_channel"]

        self.noise_settings = self.sensor_settings.get("noise_settings", dict())

        self.full_scale_voltage = self.scale * self.excitation

    def calc_noise_parameters(self):
        # Calculate static error
        self.static_error_stdev = self.full_scale_voltage * self.noise_settings.get("static_error_band", 0) / 100

        # Non-linearity
        self.nonlinearity_exponent = self.calc_nonlinearity_exponent()

        # Hysteresis
        self.hysteresis = self.noise_settings.get("hysteresis", 0) / 100

        # Repeatability
        self.repeatability_offset = np.random.normal(0, self.full_scale_voltage * self.noise_settings.get("repeatability", 0) / 100)

        # Thermal applied at a given true voltage
        self.thermal_error = self.noise_settings.get("thermal_error", 0) / 100 * self.noise_settings.get("temperature_offset", 0)


    def calc_nonlinearity_exponent(self):
        voltage = 10
        if self.full_scale_voltage != 1:
            self.full_scale_voltage
        error = self.noise_settings.get("non-linearity", 0) / 100
        b = log(1 + error, voltage)
        return b

    def attach_sim_source(self, sim_source):
        self.attached_sim_source = sim_source

    def attach_shared_attributes(self, shared_attributes):
        self.shared_attributes = shared_attributes

    def update_shared_attributes(self):
        if self.shared_attributes is not None:
            self.shared_attributes.set_true_force_vector(self.true_force_vector)
            self.shared_attributes.set_true_force_scalar(self.true_force_scalar)
            self.shared_attributes.set_true_voltage_scalar(self.true_voltage_scalar)

            self.shared_attributes.set_noisy_voltage_scalar(self.noisy_voltage_scalar)

    def update_internal_attributes(self):
        self.calc_sensor_force_scalar()
        self.calc_true_voltage_scalar()

        # Calc noise
        self.calc_noisy_voltage_scalar()

    def pull_from_sim(self):
        '''Gets the real net force from the shared attribute updated by the sim'''
        # Must be overridden
        pass
    
    def calc_sensor_force_scalar(self):
        self.last_true_force_scalars.append(self.true_force_scalar)
        self.last_true_force_scalars = self.last_true_force_scalars[-5:]
        self.true_force_scalar = ((self.true_force_vector[0])**2 + (self.true_force_vector[1])**2 + (self.true_force_vector[2])**2)**0.5

    def get_sensor_force_scalar(self):
        return self.true_force_scalar

    def get_sensor_force_vector(self):
        return self.true_force_vector
    
    def get_true_voltage_scalar(self) -> float:
        return self.true_voltage_scalar
    
    def get_noisy_voltage_scalar(self) -> float:
        return self.noisy_voltage_scalar
    
    def calc_true_voltage_scalar(self):
        force_fraction = self.true_force_scalar / self.full_scale_force
        full_force_voltage = self.excitation * self.scale
        self.true_voltage_scalar = force_fraction * full_force_voltage

    def calc_noisy_voltage_scalar(self):
        # Calculate static error
        static_error = np.random.normal(0, self.static_error_stdev)
        # Non-linearity
        if self.true_voltage_scalar > 1E-5:
            nonlinearity_multiplier = self.true_voltage_scalar**(self.nonlinearity_exponent)
        else:
            nonlinearity_multiplier = 1
        # Hysteresis
        average_last = np.average(self.last_true_force_scalars)
        #hysteresis_error = (self.true_force_scalar - average_last) / self.true_force_scalar * self.hysteresis * self.full_scale_voltage
        hysteresis_error = copysign(1, self.true_force_scalar - average_last) * self.hysteresis * self.full_scale_voltage
        # Repeatability
        # self.repeatability_offset

        # Thermal
        thermal_error = self.thermal_error * self.true_voltage_scalar

        self.noisy_voltage_scalar = self.true_voltage_scalar*nonlinearity_multiplier + static_error + hysteresis_error + self.repeatability_offset + thermal_error

class LoadCellJoint(LoadCell):
    def __init__(self, sensor_settings: dict={}):
        super().__init__(sensor_settings=sensor_settings)
        self.attached_sim_source: SharedJoint = None
    
    def attach_sim_source(self, sim_source: 'SharedJoint'):
        if not isinstance(sim_source._getvalue(), SharedJoint):
            raise TypeError(f"sim_source for LoadCellJoint {self.name} is not a SharedJoint")
        self.attached_sim_source = sim_source

    def pull_from_sim(self):
        self.true_force_vector = self.attached_sim_source.get_force()

class LoadCellSpring(LoadCell):
    '''Provides feedback on the force along the axis of a spring.'''
    def __init__(self, sensor_settings: dict={}):
        super().__init__(sensor_settings=sensor_settings)
        self.attached_sim_source: SharedSpring = None

        self.unstretched_length: float = 0.0
        self.true_length: float = 0.0

    def attach_sim_source(self, sim_source: 'SharedSpring'):
        if not isinstance(sim_source._getvalue(), SharedSpring):
            raise TypeError(f"sim_source for LoadCellSpring {self.name} is not a SharedSpring")
        self.attached_sim_source = sim_source
        self.unstretched_length = sim_source.get_unstretched_length()

    def pull_from_sim(self):
        self.true_force_vector = self.attached_sim_source.get_spring_force()
        self.calc_sensor_force_scalar()
        self.true_length = self.attached_sim_source.get_length()

    def calc_sensor_force_scalar(self):
        self.last_true_force_scalars.append(self.true_force_scalar)
        self.last_true_force_scalars = self.last_true_force_scalars[-5:]
        self.true_force_scalar = ((self.true_force_vector[0])**2 + (self.true_force_vector[1])**2 + (self.true_force_vector[2])**2)**0.5

        if self.true_length >= self.unstretched_length: # Tension is positive
            self.true_force_scalar *= 1
        else:
            self.true_force_scalar*= -1

    def calc_true_voltage_scalar(self):
        '''Include the possibility to be negative if in tension'''
        force_fraction = self.true_force_scalar / self.full_scale_force
        full_force_voltage = self.excitation * self.scale
        self.true_voltage_scalar = force_fraction * full_force_voltage



class SharedLoadCell():
    def __init__(self):
        self.true_force_vector = np.array([0.0, 0.0, 0.0]) # Current real force on the joint
        self.true_force_scalar: float = 0.0
        self.true_voltage_scalar: float = 0.0

        self.noisy_force_vector = np.array([0.0, 0.0, 0.0]) # Current real force on the joint
        self.noisy_force_scalar: float = 0.0
        self.noisy_voltage_scalar: float = 0.0

        self.connected_to_plotter: bool = False
        self.connected_to_visualization: bool = False
        self.connected_to_sensor: bool = False
        self.connected_to_controller: bool = False

    def set_true_force_vector(self, true_force_vector):
        self.true_force_vector = true_force_vector
    
    def get_true_force_vector(self) -> float:
        return self.true_force_vector
    
    def set_true_force_scalar(self, true_force_scalar):
        self.true_force_scalar = true_force_scalar
    
    def get_true_force_scalar(self) -> float:
        return self.true_force_scalar
    
    def set_true_voltage_scalar(self, true_voltage_scalar):
        self.true_voltage_scalar = true_voltage_scalar
    
    def get_true_voltage_scalar(self) -> float:
        return self.true_voltage_scalar
    
    def set_noisy_voltage_scalar(self, noisy_voltage_scalar):
        self.noisy_voltage_scalar = noisy_voltage_scalar
    
    def get_noisy_voltage_scalar(self) -> float:
        return self.noisy_voltage_scalar
    
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

class SharedLoadCellJoint(SharedLoadCell):
    def __init__(self):
        super().__init__()

class SharedLoadCellSpring(SharedLoadCell):
    def __init__(self):
        super().__init__()



class DisplacementSensor():
    '''Measures the displacement between 2 joints.
    The output is the non-negative scalar distance between parent and child'''
    def __init__(self, parent_joint: 'SharedJoint', child_joint: 'SharedJoint', sensor_settings: dict={}):
        self.sensor_settings = sensor_settings
        self.parse_settings()
        self.calc_noise_parameters()

        if not isinstance(parent_joint._getvalue(), SharedJoint):
            raise TypeError(f"parent_joint for DisplacementSensor {self.name} is not a SharedJoint")
        if not isinstance(child_joint._getvalue(), SharedJoint):
            raise TypeError(f"child_joint for DisplacementSensor {self.name} is not a SharedJoint")

        parent_joint.set_connected_to_sensor(True)
        child_joint.set_connected_to_sensor(True)

        
        self.sim_source_parent:SharedJoint = parent_joint
        self.sim_source_child:SharedJoint = child_joint
        
    
        self.shared_attributes: SharedDisplacementSensor = None # For plotting if desired

        self.length: float = 0.0                                # distance from child to parent
        self.true_disp_scalar: float = 0.0                      # length - neutral_length
        self.last_true_disp_scalars: list[float] = []           # Stores the last true displacements for hysteresis reasons
        self.calc_sensor_true_disp()

        self.true_voltage: float = 0.0
        self.noisy_voltage: float = 0.0

    def parse_settings(self):
        self.name = self.sensor_settings["name"]
        self.scale = self.sensor_settings["volts_per_meter"]
        self.output_channel = self.sensor_settings["output_channel"]
        self.neutral_length = self.sensor_settings["neutral_length"]
        self.neutral_voltage = self.sensor_settings.get("neutral_voltage", 0)

        self.noise_settings = self.sensor_settings.get("noise_settings", dict())

        self.full_scale_voltage = self.scale * (self.sensor_settings["max_length"] - self.neutral_length)

    def calc_noise_parameters(self):
        # Calculate static error
        self.static_error_stdev = self.full_scale_voltage * self.noise_settings.get("static_error_band", 0) / 100

        # Non-linearity
        self.nonlinearity_exponent = self.calc_nonlinearity_exponent()

        # Hysteresis
        self.hysteresis = self.noise_settings.get("hysteresis", 0) / 100

        # Repeatability
        self.repeatability_offset = np.random.normal(0, self.full_scale_voltage * self.noise_settings.get("repeatability", 0) / 100)

        # Thermal applied at a given true voltage
        self.thermal_error = self.noise_settings.get("thermal_error", 0) / 100 * self.noise_settings.get("temperature_offset", 0)
    
    def calc_nonlinearity_exponent(self):
        voltage = 10
        if self.full_scale_voltage != 1:
            self.full_scale_voltage
        error = self.noise_settings.get("non-linearity", 0) / 100
        b = log(1 + error, voltage)
        return b
    
    def attach_shared_attributes(self, shared_attributes):
        if not isinstance(shared_attributes._getvalue(), SharedDisplacementSensor):
            raise TypeError(f"shared_attributes for DisplacementSensor {self.name} is not a SharedDisplacementSensor")
        self.shared_attributes = shared_attributes
        self.shared_attributes.set_neutral_length(self.neutral_length)

    def pull_from_sim(self):
        '''Gets the real displacement from the shared attribute updated by the sim'''
        parent_pos = self.sim_source_parent.get_pos()
        child_pos = self.sim_source_child.get_pos()
        
        axis_vector = child_pos - parent_pos

        self.length = ((axis_vector[0])**2 + (axis_vector[1])**2 + (axis_vector[2])**2)**0.5

    def calc_sensor_true_disp(self):
        self.last_true_disp_scalars.append(self.true_disp_scalar)
        self.last_true_disp_scalars = self.last_true_disp_scalars[-5:]
        self.true_disp_scalar = self.length - self.neutral_length

    def calc_true_voltage(self):
        self.true_voltage = self.true_disp_scalar * self.scale

    def calc_noisy_voltage(self):
        # Calculate static error
        static_error = np.random.normal(0, self.static_error_stdev)
        # Non-linearity
        if self.true_voltage > 1E-5:
            nonlinearity_multiplier = self.true_voltage**(self.nonlinearity_exponent)
        else:
            nonlinearity_multiplier = 1
        # Hysteresis
        average_last = np.average(self.last_true_disp_scalars)
        #hysteresis_error = (self.true_force_scalar - average_last) / self.true_force_scalar * self.hysteresis * self.full_scale_voltage
        hysteresis_error = copysign(1, self.true_disp_scalar - average_last) * self.hysteresis * self.full_scale_voltage
        # Repeatability
        # self.repeatability_offset

        # Thermal
        thermal_error = self.thermal_error * self.true_voltage

        self.noisy_voltage = self.true_voltage*nonlinearity_multiplier + static_error + hysteresis_error + self.repeatability_offset + thermal_error

    def update_shared_attributes(self):
        if self.shared_attributes is not None:
            self.shared_attributes.set_current_length(self.length)
            self.shared_attributes.set_true_voltage(self.true_voltage)
            self.shared_attributes.set_noisy_voltage(self.noisy_voltage)
            self.shared_attributes.set_displacement(self.true_disp_scalar)

    def update_internal_attributes(self):
        self.calc_sensor_true_disp()
        self.calc_true_voltage()
        self.calc_noisy_voltage()

    def get_true_voltage_scalar(self) -> float:
        return self.true_voltage
    
    def get_noisy_voltage_scalar(self) -> float:
        return self.noisy_voltage

class SharedDisplacementSensor():
    def __init__(self):
        self.neutral_length: float = 0.0
        self.length: float = 0.0
        self.displacement: float = 0.0

        self.true_voltage: float = 0.0
        self.noisy_voltage: float = 0.0

        self.connected_to_plotter: bool = False
        self.connected_to_visualization: bool = False
        self.connected_to_sensor: bool = False
        self.connected_to_controller: bool = False
    
    def set_neutral_length(self, new:float):
        self.neutral_length = new

    def set_current_length(self, new:float):
        self.length = new
    
    def set_displacement(self, new:float):
        self.displacement = new

    def set_true_voltage(self, new:float):
        self.true_voltage = new

    def set_noisy_voltage(self, new:float):
        self.noisy_voltage = new

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
    
    def get_neutral_length(self) -> float:
        return self.neutral_length
    
    def get_displacement(self) -> float:
        return self.displacement

    def get_current_length(self) -> float:
        return self.length
    
    def get_true_voltage(self) -> float:
        return self.true_voltage

    def get_noisy_voltage(self) -> float:
        return self.noisy_voltage