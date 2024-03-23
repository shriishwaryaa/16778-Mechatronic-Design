#! /usr/bin/python3

import pybullet as p
import time
import pybullet_data

# Connect to PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, -10)

# Optionally, set an additional search path to find the URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load two URDF files
model1_id = p.loadURDF("plane.urdf", basePosition=[0, 0, 0])
model2_id = p.loadURDF("/urdf/dance_robot.urdf", basePosition=[0, 0, 0])

# Run the simulation for a few seconds
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./240)

    # Check for contacts between the two models
    contacts = p.getContactPoints(bodyA=model1_id, bodyB=model2_id)
    if contacts:
        print("Contact detected!")
        for contact in contacts:
            print(contact)  # Prints details about the contact points
    else:
        print("No contact yet")
# Disconnect
p.disconnect()
