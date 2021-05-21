"""python_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Camera

def run_robot(robot):
    timeStep=32     
    max_speed=6.28  

    #Motors
    left_motor=robot.getDevice('ps0')
    right_motor=robot.getDevice('ps1')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    ds_right=robot.getDevice('ds_right')
    ds_right.enable(timeStep)
    ds_left=robot.getDevice('ds_left')
    ds_left.enable(timeStep)
    # create the Robot instance.
    #robot = Robot()

        # ...
    # get the time step of the current world.
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getMotor('motorname')
    #  ds = robot.getDistanceSensor('dsname')
    #  ds.enable(timestep)

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        left_obstacle=ds_left.getValue()
        right_obstacle=ds_left.getValue()
        print("left:{} right:{}".format(left_obstacle,right_obstacle))

        left_speed=max_speed
        right_speed=max_speed

        if (left_obstacle < 80) :
            print("GO right")
            left_speed=0.5*max_speed
            right_speed=-0.5*max_speed
        elif (right_obstacle < 80) :
            print("GO left")
            left_speed=-0.5*max_speed
            right_speed=0.5*max_speed
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()

        # Process sensor data here.

        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        

    # Enter here exit cleanup code.
if__name__=="__main__":
    my_robot=Robot()
    run_robot(my_robot)