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
    
camera = robot.getDevice('camera')
camera.enable(timeStep)
camera.recognitionEnable(timeStep)
camera.getRecognitionObjects()
lidar=robot.getDevice('lidar')
lidar.enable(timeStep)
lidar.getRangeImage()
lidar.enablePointCloud()
lidar_horz=lidar.getHorizontalResolution()
 
 
# --------------------------------------------------------------------

# Get frontal distance sensors.
# Set ideal motor velocity.

leftMotor = robot.getDevice("motor_left")
rightMotor = robot.getDevice("motor_right")

# Set the initial velocity of the left and right wheel motors.
#set_motor_velocities(initialVelocity,initialVelocity)

    

# Get frontal distance sensors.
outerLeftSensor = robot.getDevice("ds_left")
centralLeftSensor = robot.getDevice("ds_central_left")
centralSensor = robot.getDevice("ds_central")
centralRightSensor = robot.getDevice("ds_central_right")
outerRightSensor = robot.getDevice("ds_right")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 0.7 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant
    print("left:{} right:{} central_left:{} center_right:{} central:{}".format(outerLeftSensorValue,outerRightSensorValue,centralLeftSensorValue,centralRightSensorValue,centralSensorValue))
    # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 2)
    rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 2 - centralSensorValue)

    


