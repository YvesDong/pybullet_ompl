import os.path as osp
import pybullet as p
import math
import sys
import pybullet_data
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
from my_donut_robot import MyDonutRobot
import matplotlib.pyplot as plt
import numpy as np

class DonutDemo():
    def __init__(self, numiter):
        self.obstacles = []

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # load robot
        robot_id = p.loadURDF("models/donut/donut.urdf", (0,0,0))
        self.robot = MyDonutRobot(robot_id)
        
        self.start = [0,0,4,0,1,0] # :3 pos // 3: rot [radian]
        self.goal = [0,0,0,0,0,0]
        
        # bisection search
        self.cover_zs = np.linspace(5.3,4.5,numiter)
        self.cover_z = None
        self.cage_depth = []

    def remove_cover(self):
        # for obstacle in self.obstacles:
        #     p.removeBody(obstacle)
        p.removeBody(self.obstacles[-1])
        self.obstacles.pop()

    def add_cover(self, i):
        # add cover
        self.add_box([0, 0, self.cover_zs[i]], [2, 2, 0.05])
        self.cover_z = self.cover_zs[i]

    def add_obstacles(self):
        # add box
        self.add_box([0, 0, 3], [2, 2, 0.05])
        
        # add outer wall
        self.add_box([1, 0, 3.5], [0.1, 1, .5])
        self.add_box([-1, 0, 3.5], [0.1, 1, .5])
        self.add_box([0, 1, 3.5], [1, 0.1, .5])
        self.add_box([0, -1, 3.5], [1, 0.1, .5])

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def visualize_path(self, path):
        path_z = np.array(path)[:,2]
        depth = np.around(np.max(path_z)-path_z[0], decimals=2)
        self.cage_depth.append([depth, self.cover_z])
        # plt.plot(path_z)
        # plt.xlabel('path node')
        # plt.ylabel('height')
        # plt.title('Depth of Energy-bounded Caging: {}'.format(depth))
        # plt.show()

    def demo(self):
        self.robot.set_state(self.start)
        res, path = self.pb_ompl_interface.plan(self.goal)
        if res:
            self.pb_ompl_interface.execute(path)
            self.visualize_path(path)
        return res, path


if __name__ == '__main__':
    NUMITER = 6 # no iter of bisection search
    env = DonutDemo(NUMITER)
    env.add_obstacles()

    for i in range(NUMITER):
        # PbOMPL init once again in each loop to update collision checking
        env.pb_ompl_interface = pb_ompl.PbOMPL(env.robot, env.obstacles)
        env.pb_ompl_interface.set_planner("RRT")
        env.add_cover(i)
        # store obstacles
        env.pb_ompl_interface.set_obstacles(env.obstacles)
        env.demo()
        env.remove_cover()
        print("obstacles: ", env.obstacles)
        print('cage_depth: ', env.cage_depth)
    p.disconnect()

    # visualize cover height - caging depth
    xy = np.array(env.cage_depth)
    cover_height = xy[:,1]
    cage_depth = xy[:,0]
    plt.plot(cover_height, cage_depth, '-o')
    plt.xlabel('cover height')
    plt.ylabel('caging depth')
    plt.title('Depth of Energy-bounded Caging in Bisection Search')
    plt.show()