# Copyright 2020 Simon Steinmann
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import numpy as np
from controller import Supervisor

# import custom scripts. Located in the same directory as this controller
from get_relative_position import RelativePositions
from pyikfast_module import inverseKinematics
from spawn_target import spawnTarget



# ------------------- CONFIGURATION -------------------------

# how many simulationsteps before calculating the next IK solution.
# For velocity control, a smaller value makes the movement smoother.
IKstepSize = 2

# This name is needed in the pyikfast_module.py to determine the joint2_offset (line 25)
protoName = 'vacuum'

# import the pyikfast solver module for your robot as "pyikfast_module"
import pyikfastvacuum as pyikfast_module

# Offset from toolSlot base, for which the IK solution is calculated.
# This can be useful, when attaching grippers on the robot.
# Offset = 0.2 is a fitting value for the UR10e with robotiq-3f gripper.
# Adding a "Transform" node to the toolSlot of the robot can be useful
# for figuring out how to adjust this offset.
offset = [0, 0, 0]
# -------------------- INITIALIZATION -----------------------

# Initialize the Webots Supervisor.
supervisor = Supervisor()
if not supervisor.getSupervisor():
    sys.exit('WARNING: Your robot is not a supervisor! Set the supervisor field to True and restart the controller.')
timeStep = int(supervisor.getBasicTimeStep())


# check if our world already has the TARGET node. If not, we spawn it.
target = supervisor.getFromDef('TARGET')
try:
    target.getPosition()
except Exception as e:
    print('No TARGET defined. Spawning TARGET sphere')
    spawnTarget(supervisor)

# --------------------------------------------------------------------
# Initialize the arm motors and sensors. This is a generic code block
# and works with any robotic arm.
n = supervisor.getNumberOfDevices()
motors = []
sensors = []
minPositions = []
maxPositions = []
for i in range(n):
    device = supervisor.getDeviceByIndex(i)
    print(device.getName(), '   - NodeType:', device.getNodeType())
    # if device is a rotational motor (uncomment line above to get a list of all robot devices)
    if device.getNodeType() in [54, 53]:
        motors.append(device)
        minPositions.append(device.getMinPosition())
        maxPositions.append(device.getMaxPosition())
        sensor = device.getPositionSensor()
        try:
            sensor.getName()
            sensors.append(sensor)
            sensor.enable(timeStep)
        except Exception as e:
            print('Rotational Motor: ' + device.getName() +
                  ' has no Position Sensor')
# --------------------------------------------------------------------

# Initialize our inverse kinematics module
ik = inverseKinematics(pyikfast_module, protoName)

# Initialize the RelativePositions module. 'TARGET' is the DEF of the spawned sphere.
# You can change this DEF to any other object. You can also initialilze several like this:
# RelPos_1 = RelativePositions(supervisor, 'TARGET1')
# RelPos_2 = RelativePositions(supervisor, 'TARGET2')
RelPos = RelativePositions(supervisor, 'TARGET')


# ---------------------- Main Loop -------------------------

print('-------------------------------------------------------')
print('Move or rotate the TARGET sphere to move the arm...')
while supervisor.step(IKstepSize * timeStep) != -1:
    # Get the target position relative to the robot base
    target_pos, target_rot = RelPos.get_pos()
    # Get a inverse kinematics solution with the desired position and rotation
    # target_pos is the desired position in [x, y, z] RELATIVE to the robot base
    # target_rot is the desired rotation as 3x3 rotation matrix RELATIVE to the robot base
    ikResults = ik.get_ik(target_pos, target_rot, offset=offset)
    print(ikResults, motors)
    # Set the motor positions, if we got a a valid result
    if len(ikResults) == 6:
        for i in range(len(ikResults)):
            motors[i].setPosition(ikResults[i])
