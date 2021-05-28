"""Braitenberg-based obstacle-avoiding robot controller."""


from controller import Supervisor
import cv2
import numpy as np


# Get reference to the robot.
robot = Supervisor()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 6
# For how many steps the robot turns when detecting an obstacle (higher number turns more)
turningSteps = 20
# Distance from object (furniture) the robot should stop at
stopDistance = 1


P_COEFFICIENT = 0.1



# --------------------------------------------------------------------


# Initialize the arm motors and sensors. This is a generic code block
# and works with any robotic arm. 
    
camera = robot.getDevice('camera')
camera.enable(timeStep)
camera.recognitionEnable(timeStep)

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
distanceSensors = [outerLeftSensor, centralLeftSensor, centralSensor, centralRightSensor, outerRightSensor]

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

<<<<<<< HEAD

def get_image_from_camera():
    """
    Take an image from the camera device and prepare it for OpenCV processing:
    - convert data type,
    - convert to RGB format (from BGRA), and
    - rotate & flip to match the actual image.
    """
    img = camera.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    return cv2.flip(img, 1)

while robot.step(timeStep) != -1:
=======
def avoid_collision():
>>>>>>> 5784eb2f559a5037cd3170f0b8af8f4bfcc40234
    # Read values from four distance sensors and calibrate.
    
    img = get_image_from_camera()

    # Segment the image by color in HSV color space
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img, np.array([50, 150, 0]), np.array([200, 230, 255]))

    # Find the largest segmented contour (red ball) and it's center
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    largest_contour = max(contours, key=cv2.contourArea)
    largest_contour_center = cv2.moments(largest_contour)
    center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])

    # Find error (ball distance from image center)
    error = camera.getWidth() / 2 - center_x

    # Use simple proportional controller to follow the ball
    leftMotor.setVelocity(- error * P_COEFFICIENT)
    rightMotor.setVelocity(error * P_COEFFICIENT)
    
    distances = []
    for sensor in distanceSensors:
        distances.append(sensor.getValue())
    # detect obstacles
    obstacle = False
    if min(distances) < 500:
        obstacle = True
        left_obstacle = distances[0] + distances[1] < distances[3] + distances[4]            
        right_obstacle = distances[0] + distances[1] > distances[3] + distances[4]
        # in case we only have obstacle in front, we still have to turn
        if left_obstacle is False and right_obstacle is False:
            left_obstacle = True
    else:
        left_obstacle = False
        right_obstacle = False    
    # print("left:{} right:{}".format(left_obstacle,right_obstacle))
    # initialize motor speeds at 50% of maxMotorVelocity.
    leftSpeed  = 0.5 * maxMotorVelocity
    rightSpeed = 0.5 * maxMotorVelocity
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0.5 * maxMotorVelocity
        rightSpeed = -0.5 * maxMotorVelocity
    elif right_obstacle:
        # turn left
        leftSpeed  = -0.5 * maxMotorVelocity
        rightSpeed = 0.5 * maxMotorVelocity
    return leftSpeed, rightSpeed, obstacle

while robot.step(timeStep) != -1:
    leftSpeed, rightSpeed, obstacle = avoid_collision()
    # OBJECT RECOGNITION -------------------
    recognitionObjects = camera.getRecognitionObjects()
    if len(recognitionObjects) > 0:
        recognitionPosition = recognitionObjects[0].get_position()
        recognitionSize = recognitionObjects[0].get_size()
        leftSpeed += recognitionPosition[0] + recognitionSize[0] / 2 - 0.2
        rightSpeed -= recognitionPosition[0] + recognitionSize[0] / 2 - 0.2
        if stopDistance + recognitionPosition[2] > 0:
            leftSpeed = rightSpeed = 0
        print(recognitionPosition, recognitionSize)

    

    
    
    
    
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    if obstacle:
        robot.step(turningSteps * timeStep)
    
    print(leftSpeed, rightSpeed)

    





    


