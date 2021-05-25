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
    print(device.getName(), '   - NodeType:', device.getNodeType())
    # if device is a rotational motor (uncomment line above to get a list of all robot devices)
    if device.getNodeType() == 54:
        motors.append(device)
        # Disable motor PID control mode. (change to velocity control)
        device.setPosition(float('inf'))
        sensor = device.getPositionSensor()
        sensor.enable(timeStep)
        sensors.append(sensor)

 
camera = robot.getDevice('camera')
camera.enable(timeStep)
camera.recognitionEnable(timeStep)
camera.getRecognitionObjects()
lidar=robot.getDevice('lidar')
lidar.enable(timeStep)

lidar.enablePointCloud()
lidar_horz=lidar.getHorizontalResolution()
 
 
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

# Set the initial velocity of the left and right wheel motors.
set_motor_velocities(initialVelocity,initialVelocity)

    

# feedback loop: step simulation until receiving an exit event
while robot.step(timeStep) != -1:
    
    # detect obstacles
    right_obstacle = ds_right.getValue() < 500.0
    left_obstacle =  ds_left.getValue() < 500.0 
    print("left:{} right:{}".format(left_obstacle,right_obstacle))
    # initialize motor speeds at 50% of maxMotorVelocity.
    leftVelocity  = 0.5 * maxMotorVelocity
    rigtVelocity = 0.5 * maxMotorVelocity
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftVelocity  = 0.5 * maxMotorVelocity
        rigtVelocity = -0.5 * maxMotorVelocity
    elif right_obstacle:
        # turn left
        leftVelocity  = -0.5 * maxMotorVelocity
        rigtVelocity = 0.5 * maxMotorVelocity

        
    # write actuators inputs
    set_motor_velocities(leftVelocity, rigtVelocity)

    




    


