import pickle
from PhysicsElements import StateSpace, ZERO_VECTOR, Joint, Spring
import numpy as np
from Object_Sharing import SharedJointList

def is_picklable(obj):
    try:
        pickle.dumps(obj)
        return True
    except pickle.PicklingError:
        return False

from multiprocessing.managers import BaseManager

def func(x,y):
    return 10

if __name__ == '__main__':
    BaseManager.register('Joint', Joint)
    BaseManager.register('Spring', Spring)

    manager = BaseManager()
    manager.start()

    north_wall_joint:Joint = manager.Joint(np.array([10,0,0]), mass=0.001, fixed=True, name="North Wall Joint")
    main_mass:Joint = manager.Joint(np.array([0,0,0]), mass=3, fixed=False, name="Blue Mass")
    middle_mass:Joint = manager.Joint(np.array([5,0,0]), mass=5, fixed=False, name="White Mass")
    spring1 = manager.Spring(parent_joint=middle_mass, child_joint=north_wall_joint, unstretched_length=7.5, stiffness_func=func, name="Spring 1")

    print(is_picklable(spring1))