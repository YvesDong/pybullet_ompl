import pybullet as p
from time import sleep
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.setGravity(0, 0, -10)

def add_box(box_pos, half_box_size):
    colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
    box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

    # self.obstacles.append(box_id)
    return box_id

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)

boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

basePosition = [0,0,2]
ballId = p.loadSoftBody("models/deformable_ball/ball.obj", 
                        simFileName = "models/deformable_ball/ball.vtk", 
                        basePosition = [0,0,2], 
                        scale = 0.5, mass = 4, useNeoHookean = 1, 
                        NeoHookeanMu = 400, NeoHookeanLambda = 600, 
                        NeoHookeanDamping = 0.001, useSelfCollision = 1, 
                        frictionCoeff = .5, collisionMargin = 0.001)

box_pos = [0, 0, 0]
box_id = add_box(box_pos, [5, 5, .1])
print('p.getNumJoints(id)', p.getJointInfo(box_id))
p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

#logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "perf.json")
h = np.linspace(basePosition[2], box_pos[2], 1000)
i = 0
while p.isConnected():
  # p.performCollisionDetection()
  p.stepSimulation()
  p.setJointMotorControl2(ballId, 2, 
                          p.POSITION_CONTROL, h[i],force=.005 * 240.)
  # print(len(p.getContactPoints(bodyA=ballId, bodyB=box_id)))
  # print(p.getClosestPoints(bodyA=ballId, bodyB=planeId, distance=0.))

  #there can be some artifacts in the visualizer window, 
  #due to reading of deformable vertices in the renderer,
  #while the simulators updates the same vertices
  #it can be avoided using
  #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  #but then things go slower
  # p.setGravity(0,0,-10)
  sleep(1./240.)
  i += 1
  if i == 999:
     break
#p.resetSimulation()
#p.stopStateLogging(logId)
