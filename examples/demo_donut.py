import os.path as osp
import pybullet as p
import math
import sys
import pybullet_data
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
from my_donut_robot import MyDonutRobot

class DonutDemo():
    def __init__(self):
        self.obstacles = []

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # load robot
        robot_id = p.loadURDF("models/donut/donut.urdf", (0,0,0))
        robot = MyDonutRobot(robot_id)
        self.robot = robot

        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)
        self.pb_ompl_interface.set_planner("RRTStar")

        # add obstacles
        self.add_obstacles()

    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def add_obstacles(self):
        # add box
        self.add_box([1, 0, 0.7], [1.5, 1.5, 0.05])

        # store obstacles
        self.pb_ompl_interface.set_obstacles(self.obstacles)

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def demo(self):
        start = [0,0,0,0,0,0] # radian for rot
        goal = [1,1,3,0,1,0]

        self.robot.set_state(start)
        res, path = self.pb_ompl_interface.plan(goal)
        if res:
            self.pb_ompl_interface.execute(path)
        return res, path

if __name__ == '__main__':
    env = DonutDemo()
    env.demo()