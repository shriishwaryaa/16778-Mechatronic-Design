#! /usr/bin/python3

import os
import time
import math
from timeit import default_timer as timer
import subprocess 
import time
import numpy as np
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data

from controllers_module.hexapod_controller import HexapodController

class DanceRobotSimulator:
    def __init__(self, gui=False,
    urdf=(os.path.dirname(os.path.abspath(__file__))+'/urdf/dance_robot.urdf'),
    dt = 1.0/240.0,
    control_dt=0.05):
        self.GRAVITY = -9.81
        self.dt = dt
        self.control_dt = control_dt
        self.control_period = int(control_dt/dt)
        self.t = 0
        self.i = 0
        self.safety_turnover = True

        self.kp = 1.0/12.0
        self.kd = 0.4
        # only 2 yaw joints as of now
        self.angles = np.zeros(2)

        if gui:
            self.physics = bc.BulletClient(connection_mode=p.GUI)
            self.physics.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
            self.physics.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)
            self.physics.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            self.physics.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            self.physics.resetDebugVisualizerCamera(cameraDistance=1,
                            						cameraYaw=20,
			                             			cameraPitch=-20,
            			                			cameraTargetPosition=[1, -0.5, 0.8])
        else:
            self.physics= bc.BulletClient(connection_mode=DIRECT)

        self.physics.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.physics.resetSimulation()
        self.physics.setGravity(0,0,self.GRAVITY)
        self.physics.setTimeStep(self.dt)
        self.physics.setPhysicsEngineParameter(fixedTimeStep=self.dt)
        
        start_pos = [0,0,0.15]
        start_orientation = self.physics.getQuaternionFromEuler([0.0,0.0,0.0])
        
        self.planeId=self.physics.loadURDF("plane.urdf")
        self.botId = self.physics.loadURDF(urdf)
        self.physics.resetBasePositionAndOrientation(self.planeId,[0.0,0.0,0.0],[0.0,0.0,0.0,0.0])
        self.physics.resetBasePositionAndOrientation(self.botId,[0.0,0.0,0.0],[0.0,0.0,0.0,0.0])
        self.joint_list = self.make_joint_list(self.botId)

        self.leg_link_ids = [0, 1]
        self.descriptor = {0: [], 1 : []}

        self.physics.setRealTimeSimulation(0)
        jointFrictionForce=1

        for joint in range(self.physics.getNumJoints(self.botId)):
            self.physics.setJointMotorControl2(self.botId, joint,
            p.POSITION_CONTROL,
            force=jointFrictionForce)

        for t in range(0,100):
            self.physics.stepSimulation()
            self.physics.setGravity(0,0,self.GRAVITY)

    def destroy(self):
        try:
            self.physics.disconnect()
        except p.error as e:
            print("Warning (destructor of simulator):".e)

    def reset(self):
        assert(0), "not working for now"
        self.t = 0
        self.physics.resetSimulation()

    # Returns the position and orientationa
    def get_pos(self):
        return self.physics.getBasePositionAndOrientation(self.botId)

    def step(self, controller):
        if self.i % self.control_dt == 0:
            self.angles = controller.step(self)
            print("angles")
            print(self.angles)
        self.i += 1

        # Check if the roll and pitch are not too high
        error = False
        
        # Get current position and orientation
        self.euler = self.physics.getEulerFromQuaternion(self.get_pos()[1])
        if(self.safety_turnover):
            # Check the roll and pitch
            if((abs(self.euler[1]) >= math.pi/2) or 
            abs(self.euler[0]) >= math.pi/2):
                # print("ERRRRR")
                error = True

        # move the joints
        missing_joint_count = 0
        j = 0

        # Set the joint positions here
        for joint in self.joint_list:
            if joint==1000:
                missing_joint_count += 1
            else:
                # Fetch info about each joint
                info = self.physics.getJointInfo(self.botId, joint)
                # These are defined inside the urdf file
                lower_limit = info[8]
                upper_limit = info[9]
                max_force = info[10]
                max_velocity = info[11]
                pos = min(max(lower_limit, self.angles[j]), upper_limit)
                try:
                    self.physics.setJointMotorControl2(self.botId, joint,
                    p.POSITION_CONTROL,
                    positionGain=self.kp,
                    velocityGain=self.kd,
                    targetPosition=pos,
                    force=max_force,
                    maxVelocity=max_velocity)
                except e:
                    print("Could not set joint position")
            # print("HERE")
            j += 1

        # Get the contact points between the robot and the world plane
        contact_points = self.physics.getContactPoints(self.botId, self.planeId)
        link_ids = []

        if len(contact_points) > 0:
            print("Number of points of contact:")
            print(len(contact_points))

            for cn in contact_points:
                # This gives the ID of the link in contact with the ground
                linkid = cn[3]
                if linkid not in link_ids:
                    link_ids.append(linkid)

        for l in self.leg_link_ids:
            cns = self.descriptor[l]
            if l in link_ids:
                cns.append(1)
            else:
                cns.append(0)
            
            self.descriptor[l] = cns

        # Now we add gravity
        self.physics.setGravity(0,0,self.GRAVITY)

        # Step the simulation
        self.physics.stepSimulation()
        self.t += self.dt

        time.sleep(1.0/240.0)

        return error

    def get_joints_positions(self):
        p = np.zeros(len(self.joint_list))
        i = 0

        for joint in self.joint_list:
            p[i] = self.physics.getJointState(self.botId, joint)[0]
            i += 1

        return p

    def make_joint_list(self,botId):
        # These are defined in the urdf file
        joint_names = [b'left_hip_yaw',
                        b'right_hip_yaw']
        
        joint_list = []

        for n in joint_names:
            joint_found = False

            for joint in range(self.physics.getNumJoints(botId)):
                name = self.physics.getJointInfo(botId, joint)[1]

                if name == n:
                    joint_list += [joint]
                    joint_found = True
            
            if joint_found == False:
                joint_list += [1000]

        print("Joint List")
        print(joint_list)
        return joint_list

def test_ref_controller():
    # ctrl = [1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5]
    ctrl = [1, 0, 0.5, 0.25, 0.25, 0.5]
    simu = DanceRobotSimulator(gui=True)
    controller = HexapodController(ctrl)

    for i in range(0, int(3.0/simu.dt)):
        simu.step(controller)
        time.sleep(1.0/240)
    
        print("=>", simu.get_pos()[0])

    simu.destroy()

if __name__ == "__main__":
    for k in range(0,10):
        to=time.perf_counter()
        test_ref_controller()
        print(time.perf_counter()-t0,  "ms")


