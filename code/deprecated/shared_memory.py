import multiprocessing
from multiprocessing.managers import BaseManager
import numpy as np
import time
import random
    
from PhysicsElements import Joint, Spring, SpringWithMass, Actuator, StateSpace

class CustomManager(BaseManager):
    # nothing
    pass


def worker_process(shared_state_space):
    while True:
        print("Worker Process:")
        shared_state_space.print()
        time.sleep(0.1)

def worker_process2(shared_joint: Joint):
    while True:
        print("Worker Process:")
        statespace = shared_joint.get_state_space()
        print(statespace.print())
        time.sleep(0.1)

def worker_process3(shared_joint_list: list):
    while True:
        print("Worker Process:")
        statespace:StateSpace = shared_joint_list.at(0).get_state_space()
        statespace.print()
        time.sleep(0.1)

def statespace_changer(shared_joint_list: list):
    while True:
        rand_int = random.randint(0, 10)
        shared_joint_list.at(0).set_state_space_pos(np.array([rand_int,0,0]))
        print("Changed pos")
        time.sleep(1)

class SharedJointList():
    def __init__(self):
        self.list: list = []


    def append(self, new_joint: Joint):
        self.list.append(new_joint)

    def at(self, index: int):
        return self.list[index]
    



if __name__ == "__main__":
    # Create a multiprocessing manager
    # CustomManager.register('StateSpace', StateSpace)
    # CustomManager.register('Joint', Joint)
    # CustomManager.register('SharedJointList', SharedJointList)
    #BaseManager.register('StateSpace', StateSpace)
    BaseManager.register('Joint', Joint)
    BaseManager.register('SharedJointList', SharedJointList)

    with BaseManager() as manager:
        #shared_statespace = manager.StateSpace()

        shared_joint:Joint = manager.Joint(pos=np.array([0,0,-4]), mass=0.001, fixed=True, name="North Actuator Joint")
        
        joint_list:SharedJointList = manager.SharedJointList()

        rand_joint = manager.Joint(pos=np.array([0,2,15]), 
                                mass=0.001, 
                                fixed=True, 
                                name="Random Joint")
        joint_list.append(rand_joint)


        #proc = multiprocessing.Process(target=worker_process, args=(shared_statespace,))
        #proc = multiprocessing.Process(target=worker_process2, args=(shared_joint,))
        proc = multiprocessing.Process(target=worker_process3, args=(joint_list,))
        proc.start()

        changer_proc = multiprocessing.Process(target=statespace_changer, args=(joint_list,))
        changer_proc.start()
        time.sleep(2)

        #shared_statespace.set_pos(np.array([-8,-4,-2]))
        #shared_joint.set_state_space_pos((np.array([-8,-4,-2])))
        
        
        
        
        while True:
            print("Main: ", end="")
            joint_list.at(0).get_state_space().print()
            time.sleep(1)

        proc.join()
        changer_proc.join()
