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
    def __init__(self):
        self.obstacles = []

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # load robot
        robot_id = p.loadURDF("models/donut/donut.urdf", (0,0,0))
        self.robot = MyDonutRobot(robot_id)
        
        self.start = [0,0,3,0,1,0] # :3 pos // 3: rot [radian]
        self.goal = [0,0,0,0,0,0]
        
        self.max_z_escapes = [] # successful escapes

    def add_obstacles(self):
        # add box
        self.add_box([0, 0, 2], [1, 1, 0.01])
        
        # add outer wall
        self.add_box([1, 0, 2.5], [0.01, 1, .5])
        self.add_box([-1, 0, 2.5], [0.01, 1, .5])
        self.add_box([0, 1, 2.5], [1, 0.01, .5])
        self.add_box([0, -1, 2.5], [1, 0.01, .5])

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def visualize_path(self, path):
        path_z = np.array(path)[:,2]
        max_z_escape = np.max(path_z)
        self.max_z_escapes.append(max_z_escape)
        # depth = np.around(np.max(path_z)-path_z[0], decimals=2)
        # self.cage_depth.append([depth, self.cover_z])
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
        else:
            self.max_z_escapes.append(np.inf)
        return res, path


if __name__ == '__main__':
    # NUMITER = 6 # no iter of bisection search
    env = DonutDemo()
    env.add_obstacles()
    env.pb_ompl_interface = pb_ompl.PbOMPL(env.robot, env.obstacles)

    zupper = env.robot.joint_bounds[2][1]
    zlower = env.start[2]
    eps = np.inf
    eps_thres = 1e-2 # precision threshold
    zus = []
    zls = []
    epss = []
    idx = 0
    itercount = []

    # for i in range(NUMITER):
    while eps > eps_thres: 
        # data record
        zus.append(zupper)
        zls.append(zlower)
        itercount.append(idx)

        # set upper bound of searching
        env.pb_ompl_interface.reset_robot_state_bound()
        env.pb_ompl_interface.set_planner("RRT")
        
        # start planning
        env.demo()
        
        # update bounds
        curr_max_z = env.max_z_escapes[-1]
        if curr_max_z == np.inf: # no solution
            zlower = zupper
            zupper = np.min(env.max_z_escapes) # except infs, the target z is monotonically decreasing
        else:
            zupper = (curr_max_z-zlower) / 2. + zlower
            zlower = zlower
        eps = abs(zupper - zlower)
        
        # reset z upper bound
        env.robot.set_bisec_thres(zupper)
        idx += 1

        epss.append(eps)
        print("----------max_z_escapes: ", env.max_z_escapes)
        print('----------zupper, zlower, eps: ', zupper, zlower, eps)
        print("----------joint_bounds z: ", env.robot.joint_bounds[2])

    # shut down pybullet (GUI)
    p.disconnect()

    # visualize the convergence of caging depth
    escape_zs = [[i, esc] for i, esc in enumerate(env.max_z_escapes) if esc!=np.inf] # no infs
    escape_zs = np.array(escape_zs)
    iters, escs = escape_zs[:,0], escape_zs[:,1]
    esc = env.max_z_escapes
    
    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    ax1.plot(iters, escs, '-ro', label='max_z successful escapes') # max z's along successful escape paths
    ax1.plot(itercount, zus, '-b*', label='upper bounds')
    ax1.plot(itercount, zls, '--b*', label='lower bounds')
    ax2.plot(itercount, epss, '-g.', label='convergence epsilon')
    ax1.axhline(y=env.start[2], color='k', alpha=.3, linestyle='--', label='init_z object')
    
    ax1.set_xlabel('# iterations')
    ax1.set_ylabel('z_world')
    ax1.grid(True)
    ax2.set_ylabel('bound bandwidth')
    ax2.set_yscale('log')
    ax1.legend(), ax2.legend(loc='lower right')
    plt.title('Iterative bisection search of caging depth')
    plt.show()