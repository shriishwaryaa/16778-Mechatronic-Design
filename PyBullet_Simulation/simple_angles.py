#! /usr/bin/python3

import pybullet as p
import pybullet_data
import time

# Start PyBullet in GUI mode (or p.DIRECT for non-graphical version)
physicsClient = p.connect(p.GUI)

# Optional: set the gravity direction (here, downward along the z-axis)
p.setGravity(0, 0, -10)

# Load the ground plane (optional but common in simulations)
planeId = p.loadURDF(pybullet_data.getDataPath() + "/plane.urdf")

# Load your robot URDF
# Replace 'your_robot.urdf' with the path to your robot's URDF file
robotId = p.loadURDF("/urdf/dance_robot.urdf", basePosition=[0,0,0])

# Joint indices of the joints you want to control
# Replace these indices with the actual joint indices you want to control
joint_indices = [0, 1]  # Example: joint index 0 and 1

# Target positions for the joints in radians
# Adjust these values according to your robot's requirements
target_positions = [0.5, 0]  # Example: target positions for the two joints

# Set joint control to position control for each joint
for joint_index, target_position in zip(joint_indices, target_positions):
    p.setJointMotorControl2(bodyUniqueId=robotId,
                            jointIndex=joint_index,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=target_position)

# Run the simulation for a short period to see the effect
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect the PyBullet session
p.disconnect()
