"""
Major Project - Thu 25 Nov, 2300
---
- Pybullet Camera Integration using 3D model from nicrusso7/rex-gym
- Please refer comments
"""
import pybullet as pb
import time
import pybullet_data
import numpy as np
from math import radians

#------------------------------------------------------------------
# PYBULLET SIMULATION
physicsClient = pb.connect(pb.GUI)

#------------------------------------------------------------------
# BASIC 3D PLANE
import pybullet_data
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = pb.loadURDF("plane.urdf")

#------------------------------------------------------------------
# GRAVITY
# just use 9.8 or 10 in negative z (x,y,z)
pb.setGravity(0,0,-10)

#------------------------------------------------------------------
# MODEL PROPERTIES
## Model-1 : Rex
modelStartPos = [0,0,0.3] # Starting/Spawning Position in x,y,z
# uncomment below to manipulate model rotation
# modelStartOri = pb.getQuaternionFromEuler([0,0,radians(180)])
modelId = pb.loadURDF("./urdf/rex.urdf", modelStartPos)

## Model-2 : SimpleCar
carStartPos = [-1,0,0.3]
carId = pb.loadURDF("car.urdf", carStartPos)

#------------------------------------------------------------------
# CAMERA PROPERTIES
fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
projection_matrix = pb.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

#------------------------------------------------------------------
# LINKS IN MODEL
## Idea is to spawn a camera mounted at one of the links,
## In this case the link is link 0 or chassis_front_link 
## link index & link name relation
_link_name_to_index = {pb.getBodyInfo(modelId)[0].decode('UTF-8'):-1,}
        
## Uncomment Below to print all the links in model for reference
# for _id in range(pb.getNumJoints(modelId)):
#     _name = pb.getJointInfo(modelId, _id)[12].decode('UTF-8')
#     print(str(_id), str(_name))

def getImage():
    # Center of mass position and orientation (of link-0, which is front link)
    # returns position, orientation, coinertialframeposition, and some more things?
    position, orientation, _, _, _, _ = pb.getLinkState(modelId, 1, computeForwardKinematics=True)
    rot_matrix = pb.getMatrixFromQuaternion(orientation)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)

    # print("###")
    # print(position)
    # print(orientation)

    # Initial vectors for camera
    # N-negative 1 given, (as model is in reverse ? keep looking into this)
    init_camera_vector = (-1, 0, 0) # -x-axis
    init_up_vector = (0, 0, 1) # z-axis
    
    # offset camera position so it doesn't cut the model
    # TODO : There's probably some bug in this, further testing required
    camList = list(position)
    camList[0] -= 0.2
    position = tuple(camList)
    
    # Rotated vectors
    # N- camera stuff trying to move camera beneath the center of mass
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)
    view_matrix = pb.computeViewMatrix(position, position + 0.1 * camera_vector, up_vector)
    img = pb.getCameraImage(100, 100, view_matrix, projection_matrix)
    return img

# Main loop
while True:
    pb.stepSimulation()
    # getImage2()
    getImage()
    time.sleep(1./60.)