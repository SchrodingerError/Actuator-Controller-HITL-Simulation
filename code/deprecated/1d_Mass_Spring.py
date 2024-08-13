import matplotlib.pyplot as plt
from copy import deepcopy

class StateSpace():
    def __init__(self, pos, vel, accel):
        self.pos = pos
        self.vel = vel
        self.accel = accel

    def print(self):
        print(f"Pos: {self.pos}, Vel: {self.vel}, Accel: {self.accel}")

class Block():
    def __init__(self):    
        self.mass = 5

        self.statespace = StateSpace(5,0,0)
        
        self.net_force = 0


class Spring():
    def __init__(self):
        self.mass = 1

        self.pos1 = 0
        self.pos2 = 10
        self.vel = 0
        self.accel = 0

        self.length = self.pos2 - self.pos1
        self.zero_length = 10

        self.k = 10
        self.del_l = self.length - self.zero_length
        self.force = self.del_l * self.k
    
    def get_force(self):
        self.length = self.pos2 - self.pos1
        self.del_l = self.length - self.zero_length
        self.force = self.del_l * self.k
        return self.force

applied_force = 0

block = Block()
spring = Spring()


# Initialize lists to store data for plotting
t_values = []
pos_values = []
vel_values = []
accel_values = []
force_net_values = []
spring_force_values = []
del_l_values = []


del_t = 0.1
t = 0
while t < 10:
    spring.pos1 = block.statespace.pos
    spring.force = spring.get_force()

    block.net_force = applied_force + spring.force
    block.statespace.accel =  block.net_force / block.mass
    
    block.statespace.vel += block.statespace.accel * del_t
    block.statespace.pos += block.statespace.vel  * del_t
    
    # Store data for plotting
    t_values.append(t)
    pos_values.append(block.statespace.pos)
    vel_values.append(block.statespace.vel)
    accel_values.append(block.statespace.accel)
    force_net_values.append(block.net_force)
    spring_force_values.append(spring.force)
    del_l_values.append(spring.del_l)

    t += del_t

print(max(pos_values))

# Plot the data
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(t_values, pos_values)
plt.plot(t_values, del_l_values)
plt.title('Position vs Time')
plt.legend(['Position', 'Delta Length'])
plt.xlabel('Time (s)')
plt.ylabel('Position')

plt.subplot(3, 1, 2)
plt.plot(t_values, vel_values)
plt.title('Velocity vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity')

plt.subplot(3, 1, 3)
plt.plot(t_values, accel_values)
plt.plot(t_values, force_net_values)
plt.plot(t_values, spring_force_values)
plt.legend(['Acceleration', 'Net Force', 'Spring Force'])
plt.title('Acceleration, Net Force, and Spring Force vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Value')

plt.tight_layout()
plt.show()