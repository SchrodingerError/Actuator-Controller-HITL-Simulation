import math
import numpy as np
from typing import List
import matplotlib.pyplot as plt

ZERO_VECTOR = np.array([0.0, 0.0, 0.0])



class StateSpace():
    def __init__(self, pos=np.copy(ZERO_VECTOR), vel=np.copy(ZERO_VECTOR), accel=np.copy(ZERO_VECTOR), force=np.copy(ZERO_VECTOR)):
        self.pos = pos
        self.vel = vel
        self.accel = accel

        self.force = force

    def get_pos(self):
        return self.pos
    def get_vel(self):
        return self.vel
    def get_accel(self):
        return self.accel
    def get_force(self):
        return self.force
    
    def set_pos(self, new_pos):
        self.pos = new_pos
    def set_vel(self, new_vel):
        self.vel = new_vel
    def set_accel(self, new_accel):
        self.accel = new_accel
    def set_force(self, new_force):
        self.force = new_force

    def integrate(self, delta_t):
        self.vel = self.vel + self.accel * delta_t
        self.pos = self.pos + self.vel * delta_t

    def print(self):
        print(f"Pos: {self.pos}, Vel: {self.vel}, Accel: {self.accel}")


class Joint():
    def __init__(self, pos=np.copy(ZERO_VECTOR), fixed: bool=False):
        self.statespace = StateSpace(pos, ZERO_VECTOR, ZERO_VECTOR, ZERO_VECTOR)

        self.connected_springs: List[Spring] = []
        self.connected_masses: List[Mass] = []
        self.connected_rigid_bodies: List[RigidBody] = []
        self.external_forces: List[List[float, float, float]] = []

        self.fixed = fixed

        self.parent_joints: List[Joint] = []
        self.child_joints: List[Joint] = []
    
    def get_pos(self):
        return self.statespace.get_pos()
    def get_vel(self):
        return self.statespace.get_vel()
    def get_accel(self):
        return self.statespace.get_accel()
    def get_force(self):
        return self.statespace.get_force()
    
    def get_connected_springs(self):
        return self.connected_springs
    def get_connected_masses(self):
        return self.connected_masses
    def get_connected_rigid_bodies(self):
        return self.connected_rigid_bodies
    
    def is_fixed(self):
        return self.fixed
    
    def add_spring(self, new_spring):
        self.connected_springs.append(new_spring)

    def add_mass(self, new_mass):
        self.connected_masses.append(new_mass)

    def add_rigid_body(self, new_rigid_body):
        self.connected_rigid_bodies.append(new_rigid_body)
    
    def add_extenal_force(self, new_force = ZERO_VECTOR):
        self.external_forces.append(new_force)

    def calc_net_spring_force(self):
        net_spring_force = 0
        for spring in self.connected_springs:
            spring_force = spring.calc_spring_force() # This is the force the spring exerts on the joint ASSUMING self is parent_joint of the spring
            net_spring_force += spring_force
        return net_spring_force

    
    def calc_net_external_force(self):
        net_external_force = np.copy(ZERO_VECTOR)
        for force in self.external_forces:
            net_external_force += force
        return net_external_force
    
    def calc_net_force(self):
        net_external_force = self.calc_net_external_force()
        net_spring_force = self.calc_net_spring_force()
        self.statespace.force = net_external_force + net_spring_force
        return self.statespace.force
    
    def integrate_accel_vel(self, delta_t: float=0.1):
        self.statespace.integrate(delta_t)

    def set_accel(self, accel):
        self.statespace.set_accel(accel)

    def find_parent_joints(self):
        for spring in self.connected_springs:
            if spring.child_joint == self:
                if spring.parent_joint not in self.parent_joints:
                    self.parent_joints.append(spring.parent_joint)

        for mass in self.connected_masses:
            if mass.child_joint == self:
                if mass.parent_joint not in self.parent_joints:
                    self.parent_joints.append(mass.parent_joint)

    def find_child_joints(self):
        for spring in self.connected_springs:
            if spring.parent_joint == self:
                if spring.child_joint not in self.child_joints:
                    self.child_joints.append(spring.child_joint)

        for mass in self.connected_masses:
            if mass.parent_joint == self:
                if mass.child_joint not in self.child_joints:
                    self.child_joints.append(mass.child_joint)

class Spring():
    def __init__(self, parent_joint: Joint, child_joint: Joint, zero_length: float=0, stiffness: float=0):
        self.parent_joint = parent_joint
        self.child_joint = child_joint
        
        self.zero_length = zero_length
        self.stiffness = stiffness

        self.vector = self.calc_r_vector()
        self.length = self.calc_length()
        self.unit_vector = self.calc_r_unit_vector()
        self.spring_force = self.calc_spring_force()

        self.parent_joint.add_spring(self)
        self.child_joint.add_spring(self)
    
    def calc_r_vector(self):
        self.vector = self.child_joint.get_pos() - self.parent_joint.get_pos()
        return self.vector

    def calc_length(self):
        self.vector = self.calc_r_vector()
        return np.linalg.norm(self.vector)
        
    def calc_r_unit_vector(self):
        self.vector = self.calc_r_vector()
        self.length = self.calc_length()
        self.unit_vector = self.vector / self.length
        return self.unit_vector
    
    def calc_spring_force(self):
        '''Positive force is in tension. Aka, the spring PULLS on the other object'''
        self.length = self.calc_length()
        del_length = self.length - self.zero_length
        spring_force = del_length * self.stiffness

        self.r_unit_vector = self.calc_r_unit_vector()
        self.spring_force = spring_force * self.r_unit_vector
        return self.spring_force
        

class Mass():
    '''
    A mass is a point mass located in the center of 2 joints.
    It cannot exert a force
    '''
    def __init__(self, parent_joint: Joint, child_joint: Joint, mass: float=0):
        self.parent_joint = parent_joint
        self.child_joint = child_joint

        self.mass = mass

        self.parent_joint.add_mass(self)
        self.child_joint.add_mass(self)


        self.statespace = StateSpace((child_joint.get_pos() + parent_joint.get_pos()) / 2.0,
                                     (child_joint.get_vel() + parent_joint.get_vel()) / 2.0,
                                     (child_joint.get_accel() + parent_joint.get_accel()) / 2.0,
                                     (child_joint.get_force() + parent_joint.get_force()) / 2.0)

    def set_accel(self, accel):
        self.statespace.set_accel(accel)

    def integrate_accel_vel(self, delta_t: float=0.1):
        self.statespace.integrate(delta_t)



class Simulation():
    def __init__(self, parent_joint: Joint, duration: float, delta_t: float):
        self.parent_joint = parent_joint
        self.duration = duration
        self.delta_t = delta_t

        self.joints: List[Joint] = []
        self.masses: List[Mass] = []
        self.springs: List[Spring] = []
        self.rigid_bodies: List[RigidBody] = []

    def get_all_nodes_and_edges(self):
        '''
        Do a BFS to get all of the joints (nodes) and springs/masses (edges) in the system
        '''
        queue: List[Joint] = []

        queue.append(self.parent_joint)

        while queue:
            node: Joint
            node = queue.pop(0)
            if node not in self.joints:
                self.joints.append(node)
            
            connected_springs = node.get_connected_springs()
            connected_masses = node.get_connected_masses()
            connected_rigid_bodies = node.get_connected_rigid_bodies()
            
            for spring in connected_springs:
                if spring not in self.springs:
                    self.springs.append(spring)

                if spring.child_joint not in self.joints and spring.child_joint not in queue:
                    queue.append(spring.child_joint)

            for mass in connected_masses:
                if mass not in self.masses:
                    self.masses.append(mass)
                if mass.child_joint not in self.joints and mass.child_joint not in queue:
                    queue.append(mass.child_joint)
            
            for rigid_body in connected_rigid_bodies:
                if rigid_body not in self.rigid_bodies:
                    self.rigid_bodies.append(rigid_body)
                if rigid_body.child_joint not in self.joints and rigid_body.child_joint not in queue:
                    queue.append(rigid_body.child_joint)

    def print_components(self):
        print("Joints:")
        count = 0
        for joint in self.joints:
            print(f"Accel: {joint.get_accel()}")
            print(f"Vel: {joint.get_vel()}")
            print(f"Pos: {joint.get_pos()}")
            print()
        
        print("Masses:")
        count = 0
        for mass in self.masses:
            print(f"Accel: {mass.statespace.get_accel()}")
            print(f"Vel: {mass.statespace.get_vel()}")
            print(f"Pos: {mass.statespace.get_pos()}")
            print()
        
        print("Springs:")
        count = 0
        for spring in self.springs:
            print(f"Spring Force: {spring.spring_force}")
            print(f"Spring Length: {spring.length}")
            print()
        
        print("Rigid Bodies:")
        count = 0
        for rigid_body in self.rigid_bodies:
            print(f"Transfer Force: {rigid_body.force}")
            print()

    def calc_force_at_every_joint(self):
        '''
        At every joint, calculate the net force at each joint (this accounts for springs and external forces).
        '''
        for joint in self.joints:
            joint.calc_net_force()

    def calc_rigid_body_force(self):
        for body in self.rigid_bodies:
            body.force = 0

    def calc_accel_at_every_mass(self):
        '''
        Using the sum of the forces at the 2 joints on each mass, we calc the accel of the mass
        '''
        for mass in self.masses:
            net_force = mass.parent_joint.get_force() + mass.child_joint.get_force()
            accel = net_force / mass.mass
            mass.set_accel(accel)

    def assign_joint_accel(self):
        '''
        If the joint is fixed, accel = 0
        '''
        net_joint_accel = np.copy(ZERO_VECTOR)
        for joint in self.joints:
            if joint.is_fixed() == True:
                continue

            for mass in joint.get_connected_masses():
                net_joint_accel += mass.statespace.get_accel()
            joint.set_accel(net_joint_accel)

    def integrate_timestep(self):
        self.calc_force_at_every_joint()
        self.calc_accel_at_every_mass()

        self.assign_joint_accel()

        for joint in self.joints:
            joint.integrate_accel_vel(self.delta_t)

        for mass in self.masses:
            mass.integrate_accel_vel(self.delta_t)



    def run(self):
        mass_pos_values = []
        time_values = []
        t = 0
        while t < self.duration:
            self.integrate_timestep()
            self.print_components()
            
            mass_pos_values.append(self.masses[0].statespace.get_pos())
            time_values.append(t)

            print("*"*100)
            t += self.delta_t

        plt.close()
        # Plot the data
        plt.figure(figsize=(12, 8))

        plt.plot(time_values, mass_pos_values)
        plt.title('Position vs Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Position')
        plt.show()


class BFSSimulation():
    def __init__(self, parent_joint: Joint, duration: float, delta_t: float):
        self.parent_joint = parent_joint
        self.duration = duration
        self.delta_t = delta_t

        self.joints: List[Joint] = []

        self.get_joints_bfs()
        self.get_parent_joints_for_every_joint()


    def get_joints_bfs(self):
        queue: List[Joint] = []

        queue.append(self.parent_joint)
        visited_springs: List[Spring] = []
        visited_masses: List[Spring] = []

        while queue:
            node: Joint
            node = queue.pop(0)
            if node not in self.joints:
                self.joints.append(node)

            connected_springs = node.get_connected_springs()
            connected_masses = node.get_connected_masses()
            
            for spring in connected_springs:
                if spring not in visited_springs:
                    visited_springs.append(spring)

                if spring.child_joint not in self.joints and spring.child_joint not in queue:
                    queue.append(spring.child_joint)

            for mass in connected_masses:
                if mass not in visited_masses:
                    visited_masses.append(mass)
                if mass.child_joint not in self.joints and mass.child_joint not in queue:
                    queue.append(mass.child_joint)

    def get_parent_joints_for_every_joint(self):
        for joint in self.joints:
            joint.find_parent_joints()

    def get_child_joints_for_every_joint(self):
        for joint in self.joints:
            joint.find_child_joints()
            
    def integrate_joints(self):
        for joint in self.joints:
            joint.calc_net_force()

def main():
    joint_left_mass1 = Joint(np.array([0,0,0]), fixed=False)
    joint_mass1_spring1 = Joint(np.array([0,0,0]), fixed=False)

    joint_spring1_rigidbody1 = Joint(np.array([10,0,0]), fixed=False)
    joint_rigidbody1_spring2 = Joint(np.array([10,0,0]), fixed=False)

    joint_right_spring2 = Joint(np.array([20,0,0]), fixed=True)

    mass1 = Mass(joint_left_mass1, joint_mass1_spring1, 5)
    spring1 = Spring(joint_mass1_spring1, joint_spring1_rigidbody1, 10, 10)
    #rigidbody1 = RigidBody(joint_spring1_rigidbody1, joint_rigidbody1_spring2, 0.0)
    spring2 = Spring(joint_rigidbody1_spring2, joint_right_spring2, 10, 100)

    joint_left_mass1.add_extenal_force(np.array([5, 0, 0]))

    simulation = BFSSimulation(joint_left_mass1, 1, 0.1)


    

    simulation.get_all_nodes_and_edges()

    simulation.run()

if __name__ == "__main__":
    main()
    