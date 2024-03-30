import pybullet as p
import pybullet_data
import time
import numpy as np
import os

simulation_steps = 10000
# Start PyBullet in GUI mode
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load URDFs
p.setGravity(0, 0, -10)
# Load ground plane
planeId = p.loadURDF("plane.urdf")

# Load the hexapod URDF
hexapod = (os.path.dirname(os.path.abspath(__file__))+'/urdf/pexod.urdf')
# set the initial position of the hexapod at a slight offset from the ground
# to prevent any effects of collision with simulation
hexapodId = p.loadURDF(hexapod, [0.0,0.0,0.1])
# Main simulation parameters
timeStep = 1./240.
p.setTimeStep(timeStep)

N = 6
omega = np.ones(N)
K = 2.0
theta = np.array([0, np.pi, 0, np.pi, 0, np.pi])

hip_pitch_joints = [6, 12, 0, 3, 15, 9]
hip_yaw_joints = [7, 13, 1, 4, 16, 10]

def phase_interaction(theta, K, N):
    diff_matrix = theta[:, None] - theta
    return K / N * np.sum(np.sin(diff_matrix), axis=1)

for _ in range(simulation_steps):
    theta += timeStep * (omega + phase_interaction(theta, K, N))

    for leg_id in range(N):
        hip_pitch_angle = np.sin(theta[leg_id]) * 0.2
        hip_yaw_angle = np.cos(theta[leg_id]) * 0.2

        p.setJointMotorControl2(hexapodId, hip_pitch_joints[leg_id], p.POSITION_CONTROL, targetPosition=hip_pitch_angle)
        p.setJointMotorControl2(hexapodId, hip_yaw_joints[leg_id], p.POSITION_CONTROL, targetPosition=hip_yaw_angle)
    
    # Step simulation
    p.stepSimulation()
    time.sleep(timeStep)