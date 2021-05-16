"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Supervisor

# Get reference to the robot.
robot = Supervisor()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 6.53
distanceSensorCalibrationConstant = 360

# --------------------------------------------------------------------
# Initialize the arm motors and sensors. This is a generic code block
# and works with any robotic arm.
n = robot.getNumberOfDevices()
motors = []
sensors = []
for i in range(n):
    device = robot.getDeviceByIndex(i)
    # print(device.getName(), '   - NodeType:', device.getNodeType())
    # if device is a rotational motor (uncomment line above to get a list of all robot devices)
    if device.getNodeType() == 54:
        motors.append(device)
        # Disable motor PID control mode. (change to velocity control)
        device.setPosition(float('inf'))
        sensor = device.getPositionSensor()
        sensor.enable(timeStep)
        sensors.append(sensor)
# --------------------------------------------------------------------

# Get frontal distance sensors.
ds_right = robot.getDevice("ds_right")
ds_left = robot.getDevice("ds_left")
# Enable distance sensors.
ds_right.enable(timeStep)
ds_left.enable(timeStep)

# Set ideal motor velocity.
initialVelocity = 0.7 * maxMotorVelocity

def set_motor_velocities(leftVelocity, rigtVelocity):
    motors[0].setVelocity(leftVelocity)
    motors[1].setVelocity(rigtVelocity)
    motors[2].setVelocity(leftVelocity)
    motors[3].setVelocity(rigtVelocity)

# Set the initial velocity of the left and right wheel motors.
set_motor_velocities(initialVelocity, initialVelocity)

    
while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    rightSensorValue = ds_right.getValue() / distanceSensorCalibrationConstant
    leftSensorValue = ds_left.getValue() / distanceSensorCalibrationConstant
    # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    leftVelocity = initialVelocity - leftSensorValue
    rigtVelocity = initialVelocity - rightSensorValue
    set_motor_velocities(leftVelocity, rigtVelocity)