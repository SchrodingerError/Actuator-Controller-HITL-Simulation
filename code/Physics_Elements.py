from typing import List, Callable
import numpy as np


from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from Physics_Elements import StateSpace, Joint, Spring, SharedRigidActuator, SharedSpring

ZERO_VECTOR = np.array([0.0, 0.0, 0.0])


'''
TODO
1. Change spring_force to just force
    Include getters and setters, lest important for attributes
2. Add Object_Sharing objects to this file
'''


def stiffness_function_const_then_rigid(length, unstretched_length):
    percent_length = 0.9 # For the first bit of length in compression, it follors k*x
    k = 1000

    if length < (1-percent_length)* unstretched_length:
        # It is really compressed, so make k = k *10
        k = k + k*10/(1 + np.exp(-10*(unstretched_length - length - percent_length*unstretched_length)))
    return k

def damping_force_x(mass: 'Joint', t):
    x_vel = mass.statespace.get_vel()[0]
    a = 1
    b = 0.01
    return x_vel*a*-1 + x_vel**2 * b*-1

def damping_force_y(mass: 'Joint', t):
    y_vel = mass.statespace.get_vel()[1]
    a = 1
    b = 0.01
    return y_vel*a*-1 + y_vel**2 * b*-1

def damping_force_z(mass: 'Joint', t):
    z_vel = mass.statespace.get_vel()[2]
    a = 1
    b = 0.01
    return z_vel*a*-1 + z_vel**2 * b*-1

'''
TODO
'''
class StateSpace():
    def __init__(self, pos=np.copy(ZERO_VECTOR), vel=np.copy(ZERO_VECTOR), accel=np.copy(ZERO_VECTOR), force=np.copy(ZERO_VECTOR), integration_method="simple"):
        self.pos = pos
        self.vel = vel
        self.accel = accel

        self.force = force

        self.integration_method = self.parse_integration_method(integration_method)

        self.max_saved = 2
        self.previous_positions: list[list[float, float, float]] = []
        self.previous_vel: list[list[float, float, float]] = []
        self.previous_accel: list[list[float, float, float]] = []

        self.verlet_constants = [
            [],
            [],
            [-1, 2, 1],
            [0, -1, 2, 1],
            [1/11, -4/11, -6/11, 20/11, 12/11],
        ]

    def parse_integration_method(self, method_str):
        if method_str == "simple":
            return 1
        elif method_str == "verlet":
            return 2
        elif method_str == "adams-bashforth":
            return 3


    def integrate(self, delta_t):
        '''
        NOTE: This is run after accel has been updated. 
        '''
        if self.integration_method == 1:
            self.vel = self.vel + self.accel * delta_t
            self.pos = self.pos + self.vel * delta_t
        elif self.integration_method == 2:
            # Verlet integration
            self.save_last()
            if delta_t >= 0.005:
                self.vel = self.vel + self.accel * delta_t
                self.pos = self.pos + self.vel * delta_t
            else:
                self.vel = self.vel + self.accel * delta_t
                self.verlet_integration(delta_t)
        elif self.integration_method == 3: # Adams-Bashforth
            self.save_last()
            if delta_t >= 0.005:
                self.vel = self.vel + self.accel * delta_t
                self.pos = self.pos + self.vel * delta_t
            else:
                self.adams_bashforth_integration(delta_t)

    def print(self):
        print(f"Pos: {self.pos}, Vel: {self.vel}, Accel: {self.accel}")

    def adams_bashforth_integration(self, delta_t):
        '''
        Testing.
        2 points seems to damp the system. 3 does better at maintaining. Above 3 it is worse
        '''
        num_prev_accel = len(self.previous_accel)
        match num_prev_accel:
            case 0 | 1:
                self.vel = self.vel + self.accel * delta_t
            case 2:
                self.vel = self.vel + delta_t * (1.5 * self.previous_accel[-1] - 0.5 * self.previous_accel[-2])
            case 3:
                self.vel = self.vel + delta_t * (23/12 * self.previous_accel[-1] - 4/3 * self.previous_accel[-2] + 5/12 * self.previous_accel[-3])
            case 4:
                self.vel = self.vel + delta_t * (55/24 * self.previous_accel[-1] - 59/24 * self.previous_accel[-2] + 37/24 * self.previous_accel[-3] - 9/24 * self.previous_accel[-4])
            case 5:
                self.vel = self.vel + delta_t * ((1901 * self.previous_accel[-1] - 2774 * self.previous_accel[-2] + 2616 * self.previous_accel[-3] - 1274 * self.previous_accel[-4] + 251 * self.previous_accel[-5]) / 720)
            case _:
                self.vel = self.vel + self.accel * delta_t
        self.pos = self.pos + self.vel * delta_t

    def verlet_integration(self, delta_t):
        num_prev_positions = len(self.previous_positions)
        match num_prev_positions:
            case 0 | 1:
                self.pos = self.pos + self.vel * delta_t
            case 2:
                new_pos = np.array([0.0, 0.0, 0.0])
                for i in range(2):
                    new_pos += self.verlet_constants[2][i] * self.previous_positions[i]
                new_pos += self.verlet_constants[2][-1] * delta_t * self.accel * delta_t
                self.pos = new_pos
            case 3:
                new_pos = np.array([0.0, 0.0, 0.0])
                for i in range(3):
                    new_pos += self.verlet_constants[3][i] * self.previous_positions[i]
                new_pos += self.verlet_constants[3][-1] * delta_t * self.accel * delta_t
                self.pos = new_pos
            case 4:
                new_pos = np.array([0.0, 0.0, 0.0])
                for i in range(4):
                    new_pos += self.verlet_constants[4][i] * self.previous_positions[i]
                new_pos += self.verlet_constants[4][-1] * delta_t * self.accel * delta_t
                self.pos = new_pos
            case _:
                self.pos = self.pos + self.vel * delta_t

    def save_last(self):
        self.previous_positions.append(np.copy(self.pos))
        self.previous_positions = self.previous_positions[-1*self.max_saved:]
        self.previous_vel.append(np.copy(self.vel))
        self.previous_vel = self.previous_vel[-1*self.max_saved:]
        self.previous_accel.append(np.copy(self.accel))
        self.previous_accel = self.previous_accel[-1*self.max_saved:]

    def save_last_statespace(self):
        if self.last_statespace is None:
            self.last_statespace = StateSpace()
        self.last_statespace.pos = np.copy(self.pos)
        self.last_statespace.vel = np.copy(self.vel)
        self.last_statespace.accel = np.copy(self.accel)
        
    def set_pos(self, new_pos):
        self.pos = new_pos
    def set_vel(self, new_vel):
        self.vel = new_vel
    def set_accel(self, new_accel):
        self.accel = new_accel
    def set_force(self, new_force):
        self.force = new_force
    
    def get_pos(self):
        return self.pos
    def get_vel(self):
        return self.vel
    def get_accel(self):
        return self.accel
    def get_force(self):
        return self.force

class Joint():
    def __init__(self, pos=np.copy(ZERO_VECTOR), mass: float=0.1, fixed: bool=False, name: str="Joint", integration_method:str="simple"):
        self.name = name
        self.statespace = StateSpace(pos, ZERO_VECTOR, ZERO_VECTOR, ZERO_VECTOR, integration_method=integration_method)

        self.connected_springs: List[Spring] = []
        self.constant_external_forces: List[List[float, float, float]] = []
        
        self.connected_actuators: List[RigidActuator] = []

        self.variable_external_forces = [[], [], []]

        self.fixed = fixed
        self.connected_to_rigid_actuator = False
        self.mass = mass

        self.parent_joints: List[Joint] = []
        self.child_joints: List[Joint] = []

        self.damping = False

        self.sharing_attributes = False
        self.shared_attributes = None

    def is_same(self, other_joint: 'Joint'):
        if self.name == other_joint.get_name():
            return True
        return False

    def get_statespace(self):
        return self.statespace

    def get_pos(self):
        return self.statespace.get_pos()
    
    def get_vel(self):
        return self.statespace.get_vel()
    
    def get_accel(self):
        return self.statespace.get_accel()

    def set_state_space_pos(self, new_pos):
        self.statespace.set_pos(new_pos)

    def set_state_space_vel(self, new_vel):
        self.statespace.set_vel(new_vel)
    
    def set_connected_to_rigid_actuator(self, state):
        self.connected_to_rigid_actuator = state

    def get_connected_springs(self):
        return self.connected_springs
    
    def get_connected_actuators(self):
        return self.connected_actuators
    
    def get_name(self):
        return self.name

    def add_spring(self, new_spring):
        self.connected_springs.append(new_spring)

    def add_actuator(self, new_actuator):
        self.connected_actuators.append(new_actuator)
    
    def add_constant_force(self, new_force):
        self.constant_external_forces.append(new_force)

    def integrate_statespace(self, delta_t):
        self.statespace.integrate(delta_t)

    def add_gravity(self):
        gravity_force = np.array([
            0,
            -9.81 * self.mass,
            0
        ])
        self.add_constant_force(gravity_force)

    def add_variable_force(self, force_vect: list):
        if force_vect[0] is not None:
            self.variable_external_forces[0].append(force_vect[0])
        if force_vect[1] is not None:
            self.variable_external_forces[1].append(force_vect[1])
        if force_vect[2] is not None:
            self.variable_external_forces[2].append(force_vect[2])

    def calc_net_spring_force(self):
        net_spring_force = 0
        for spring in self.get_connected_springs():
            if self.is_same(spring.get_parent_joint()):
                spring_force = spring.get_spring_force() # This is the force the spring exerts on the joint ASSUMING self is parent_joint of the spring
            else: 
                spring_force = -1*spring.get_spring_force()
            net_spring_force += spring_force
        return net_spring_force

    def calc_net_constant_external_force(self):
        net_external_force = np.copy(ZERO_VECTOR)
        for force in self.constant_external_forces:
            net_external_force += force    
        return net_external_force
    
    def calc_net_variable_external_force(self, t):
        net_variable_external_force = np.array([0.0, 0.0, 0.0])
        for i in range(3):
            for func in self.variable_external_forces[i]:
                force = func(t)
                net_variable_external_force[i] += force
        return net_variable_external_force
    
    def calc_net_force(self, t):
        net_force = np.copy(ZERO_VECTOR)
        if len(self.constant_external_forces) != 0:
            net_force += self.calc_net_constant_external_force()
        if len(self.variable_external_forces) != 0:
            net_force += self.calc_net_variable_external_force(t)
        
        net_spring_force = self.calc_net_spring_force()
        
        if self.damping == True:
            net_force += self.calc_damping(net_spring_force)

        net_force += net_spring_force

        self.statespace.set_force(net_force)
        return self.statespace.get_force()
    
    def add_damping(self, vel_ratio=0.25, mom_ratio=None):
        self.damping = True
        if mom_ratio is not None:
            self.damping_vel_ratio = self.mass * mom_ratio
        else:
            self.damping_vel_ratio = vel_ratio

    def calc_damping(self, spring_force):
        vel_damping = self.damping_vel_ratio * (self.statespace.get_vel()) * -1

        return vel_damping

    def calc_accel(self):
        if self.fixed == False and self.connected_to_rigid_actuator == False:
            self.statespace.set_accel(self.statespace.get_force() / self.mass)
    
    def is_in_list(self, joints: list['Joint']):
        for j in joints:
            if self.is_same(j):
                return True
        return False

    def find_parent_joints(self):
        for spring in self.connected_springs:
            if self.is_same(spring.get_child_joint()):
                parent_joint = spring.get_parent_joint()
                if not parent_joint.is_in_list(self.parent_joints):
                    self.parent_joints.append(parent_joint)

    def find_child_joints(self):
        for spring in self.connected_springs:
            if self.is_same(spring.get_parent_joint()):
                child_joint = spring.get_child_joint()
                if not child_joint.is_in_list(self.child_joints):
                    self.child_joints.append(child_joint)

    def print_attributes(self):
        print(f"Joint: {self.name}")
        print(f"Force: {self.statespace.get_force()}")
        print(f"Accel: {self.statespace.get_accel()}")
        print(f"Vel: {self.statespace.get_vel()}")
        print(f"Pos: {self.statespace.get_pos()}")

    def attach_shared_attributes(self, shared_attributes: 'SharedJoint'):
        if not isinstance(shared_attributes._getvalue(), SharedJoint):
            raise TypeError(f"shared_attributes for Joint {self.name} is not a SharedJoint")
        self.sharing_attributes = True
        self.shared_attributes = shared_attributes

        self.shared_attributes.set_name(f"{self.name}")

        self.update_shared_attributes()

    def update_shared_attributes(self):
        if self.shared_attributes is not None:
            self.shared_attributes.set_statespace(self.statespace)

class Spring():
    def __init__(self, parent_joint: 'Joint', child_joint: 'Joint', unstretched_length: float=0, constant_stiffness: float = None, stiffness_func: Callable[[float, float], float]=None, name: str="Spring"):
        self.name = name

        self.parent_joint = parent_joint
        self.child_joint = child_joint
        
        self.unstretched_length = unstretched_length
        self.stiffness_func = stiffness_func
        self.constant_stiffness = constant_stiffness
        self.current_stiffness = 0.0

        self.vector = self.calc_r_vector()
        self.length = self.calc_length()
        self.unit_vector: list[float] = self.calc_r_unit_vector()
        self.spring_force_scalar: float = 0.0
        self.spring_force: list[float] = self.calc_spring_force()
        

        self.parent_joint.add_spring(self)
        self.child_joint.add_spring(self)

        self.sharing_attributes = False
        self.shared_attributes: 'SharedSpring' = None

    def get_name(self):
        return self.name

    def get_parent_joint(self):
        return self.parent_joint
    
    def get_child_joint(self):
        return self.child_joint
    
    def get_spring_force(self):
        return self.spring_force
    
    def calc_r_vector(self):
        return self.child_joint.get_statespace().get_pos() - self.parent_joint.get_statespace().get_pos()

    def calc_length(self):
        return ((self.vector[0])**2 + (self.vector[1])**2 + (self.vector[2])**2) ** 0.5
        
    def calc_r_unit_vector(self):
        return self.vector / self.length
    
    def calc_stiffness(self):
        if self.stiffness_func is not None:
            return self.stiffness_func(self.length, self.unstretched_length)
        else:
            return self.constant_stiffness
    
    def calc_spring_force(self):
        '''Positive force is in tension. Aka, the spring PULLS on the other object'''
        self.length = self.calc_length()
        del_length = self.length - self.unstretched_length
        self.current_stiffness = self.calc_stiffness()
        self.spring_force_scalar = del_length * self.current_stiffness

        self.vector = self.calc_r_vector()
        self.r_unit_vector = self.calc_r_unit_vector()
        self.spring_force = self.spring_force_scalar * self.r_unit_vector
        return self.spring_force
    
    def print_attributes(self):
        print(f"Spring: {self.name}")
        print(f"Length: {self.length}")
        print(f"Spring Force: {self.spring_force}")
    
    def get_pos(self):
        return self.parent_statespace.get_pos()
    def get_axis_vector(self):
        return self.child_statespace.get_pos() - self.parent_statespace.get_pos()

    def attach_shared_attributes(self, shared_attributes: 'SharedSpring'):
        if not isinstance(shared_attributes._getvalue(), SharedSpring):
            raise TypeError(f"shared_attributes for Spring {self.name} is not a SharedSpring")
        self.sharing_attributes = True
        self.shared_attributes = shared_attributes

        self.update_shared_attributes()
    
    def update_shared_attributes(self):
        if self.shared_attributes is not None:
            self.shared_attributes.set_child_statespace(self.child_joint.get_statespace())
            self.shared_attributes.set_parent_statespace(self.parent_joint.get_statespace())
            self.shared_attributes.set_spring_force(self.spring_force)
            self.shared_attributes.set_spring_force_scalar(self.spring_force_scalar)
            self.shared_attributes.set_length(self.length)
            self.shared_attributes.set_unstretched_length(self.unstretched_length)
            self.shared_attributes.set_stiffness(self.current_stiffness)

    def is_same(self, other_spring: 'Spring'):
        if self.name == other_spring.get_name():
            return True
        return False


    def is_in_list(self, springs: list['Spring']):
        for j in springs:
            if self.is_same(j):
                return True
        return False

'''
It connects 2 joints. 
1 joint is completely fixed. Cannot move. Ever. It will be referred to as "grounded".
The other joint is contrained to the grounded joint. The actuator controls the vector displacement <x,y,z> from the grounded joint
'''
class RigidActuator():
    def __init__(self, parent_joint: 'Joint', grounded_joint: 'Joint', name: str = "Rigid Actuator", max_length:float=1, min_length:float=0.1, control_code=0):
        self.name = name

        self.parent_joint = parent_joint
        self.grounded_joint = grounded_joint # Same as child joint

        self.parent_joint.add_actuator(self)
        self.parent_joint.set_connected_to_rigid_actuator(True)
        self.grounded_joint.add_actuator(self)
        self.grounded_joint.set_connected_to_rigid_actuator(True)

        self.max_length = max_length
        self.min_length = min_length
        self.control_code = control_code # 0 for pos, 1 for vel, 2 for accel

        self.calc_r_unit_vector()

        self.shared_attributes: 'SharedRigidActuator' = None

    def get_name(self):
        return self.name

    def get_parent_joint(self):
        return self.parent_joint
    
    def get_child_joint(self):
        return self.grounded_joint
    
    def is_same(self, other: 'RigidActuator'):
        if self.name == other.get_name():
            return True
        return False

    def is_in_list(self, actuators: list['RigidActuator']):
        for j in actuators:
            if self.is_same(j):
                return True
        return False
    
    def calc_r_vector(self):
        return self.grounded_joint.get_statespace().get_pos() - self.parent_joint.get_statespace().get_pos()
    
    def calc_length(self):
        return ((self.vector[0])**2 + (self.vector[1])**2 + (self.vector[2])**2) ** 0.5
        
    def calc_r_unit_vector(self):
        self.vector = self.calc_r_vector()
        self.length = self.calc_length()
        return self.vector / self.length
    
    def get_pos(self):
        return self.parent_joint.get_statespace().get_pos()
    
    def get_axis_vector(self):
        return self.grounded_joint.get_statespace().get_pos() - self.parent_joint.get_statespace().get_pos()
    
    def get_r_unit_vector(self):
        return self.calc_r_unit_vector()

    def update_shared_attributes(self):
        # These 2 lines are used to make sure we set the internal properties correctly
        self.vector = self.calc_r_vector()
        self.length = self.calc_length()
        # END
        if self.shared_attributes is not None:
            self.shared_attributes.set_child_statespace(self.grounded_joint.get_statespace())
            self.shared_attributes.set_parent_statespace(self.parent_joint.get_statespace())
            self.shared_attributes.set_length(self.length)

    def attach_shared_attributes(self, shared_attributes: 'SharedRigidActuator'):
        if not isinstance(shared_attributes._getvalue(), SharedRigidActuator):
            raise TypeError(f"shared_attributes for RigidActuator {self.name} is not a SharedRigidActuator")
        self.sharing_attributes = True
        self.shared_attributes = shared_attributes

        self.update_shared_attributes()

    def set_parent_pos(self, new_pos):
        '''Set the position of the parent joint's statestpace'''
        self.parent_joint.set_state_space_pos(new_pos)

    def update_from_controller(self):
        if self.control_code == 0: # Controller controls pos
            self.set_pos_to_controlled_pos()
        elif self.control_code == 1: # Controller controls vel
            self.set_vel_to_controlled_vel()
        elif self.control_code == 2: # Controls controls accel
            pass

    def set_vel_to_controlled_vel(self):
        controlled_vel = self.shared_attributes.get_vel()
        self.parent_joint.set_state_space_vel(controlled_vel)

    def set_pos_to_controlled_pos(self):
        controlled_pos = self.shared_attributes.get_pos()
        self.parent_joint.set_state_space_pos(controlled_pos)


class SharedPhysicsElement():
    def __init__(self) -> None:
        self.name: str = None
        self.statespace: StateSpace = None

        self.connected_to_plotter: bool = False
        self.connected_to_visualization: bool = False
        self.connected_to_sensor: bool = False
        self.connected_to_controller: bool = False
    
    def set_name(self, name:str) -> None:
        self.name = name
    def set_statespace(self, statespace:StateSpace) -> None:
        self.statespace = statespace
    def set_connected_to_plotter(self, state: bool) -> None:
        self.connected_to_plotter = state
    def set_connected_to_visualization(self, state: bool) -> None:
        self.connected_to_visualization = state
    def set_connected_to_sensor(self, state: bool) -> None:
        self.connected_to_sensor = state
    def set_connected_to_controller(self, state: bool) -> None:
        self.connected_to_controller = state

    def get_name(self) -> str:
        return self.name
    def get_statespace(self) -> StateSpace:
        return self.statespace
    def get_pos(self) -> List[float]:
        return self.statespace.get_pos()
    def get_vel(self) -> List[float]:
        return self.statespace.get_vel()
    def get_accel(self) -> List[float]:
        return self.statespace.get_accel()
    def get_force(self) -> List[float]:
        return self.statespace.get_force()
    def get_connected_to_plotter(self) -> bool:
        return self.connected_to_plotter
    def get_connected_to_visualization(self) -> bool:
        return self.connected_to_visualization
    def get_connected_to_sensor(self) -> bool:
        return self.connected_to_sensor
    def get_connected_to_controller(self) -> bool:
        return self.connected_to_controller
    
class SharedJoint(SharedPhysicsElement):
    def _init__(self) -> None:
        super().__init__()

class SharedSpring(SharedPhysicsElement):
    '''self.statespace is the statespace of the parent'''
    def _init__(self) -> None:
        super().__init__()
        self.child_statespace: StateSpace = None

        self.unstretched_length: float = None
        self.length: float = None
        self.stiffness: float = None

        self.spring_force: float = None

    def set_parent_statespace(self, statespace:StateSpace) -> None:
        self.statespace = statespace
    def set_child_statespace(self, statespace:StateSpace) -> None:
        self.child_statespace = statespace
    def set_unstretched_length(self, unstretched_length: float) -> None:
        self.unstretched_length = unstretched_length
    def set_length(self, length: float) -> None:
        self.length = length
    def set_stiffness(self, stiffness: float) -> None:
        self.stiffness = stiffness
    def set_spring_force(self, set_spring_force: float) -> None:
        self.spring_force = set_spring_force
    def set_spring_force_scalar(self, scalar_force: float) -> None:
        self.spring_force_scalar = scalar_force
    
    def get_parent_statespace(self) -> StateSpace:
        return self.statespace
    def get_child_statespace(self) -> StateSpace:
        return self.child_statespace
    def get_unstretched_length(self) -> float:
        return self.unstretched_length
    def get_length(self) -> float:
        return self.length
    def get_stiffness(self) -> float:
        return self.stiffness
    def get_spring_force(self) -> list[float]:
        return self.spring_force
    def get_spring_force_scalar(self) -> float:
        return self.spring_force_scalar
    def get_axis_vector(self) -> List[float]:
        return self.child_statespace.get_pos() - self.statespace.get_pos()


class SharedRigidActuator(SharedPhysicsElement):
    '''self.statespace is the statespace of the parent'''
    def _init__(self) -> None:
        super().__init__()
        self.child_statespace: StateSpace = None

        self.length: float = None

    def set_parent_statespace(self, statespace:StateSpace) -> None:
        self.statespace = statespace
    def set_parent_statespace_pos(self, new):
        self.statespace.set_pos(new)
    def set_parent_statespace_vel(self, new):
        self.statespace.set_vel(new)
    def set_child_statespace(self, statespace:StateSpace) -> None:
        self.child_statespace = statespace
    def set_length(self, length: float) -> None:
        self.length = length
    
    def get_parent_statespace(self) -> StateSpace:
        return self.statespace
    def get_child_statespace(self) -> StateSpace:
        return self.child_statespace
    def get_length(self) -> float:
        return self.length
    def get_axis_vector(self) -> List[float]:
        return self.child_statespace.get_pos() - self.statespace.get_pos()
    def get_r_unit_vector(self):
        return self.calc_r_unit_vector()
    def calc_r_vector(self):
        return self.child_statespace.get_pos() - self.statespace.get_pos()
    def calc_length(self):
        return ((self.vector[0])**2 + (self.vector[1])**2 + (self.vector[2])**2) ** 0.5  
    def calc_r_unit_vector(self):
        self.vector = self.calc_r_vector()
        self.length = self.calc_length()
        return self.vector / self.length