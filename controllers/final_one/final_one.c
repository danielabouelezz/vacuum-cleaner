/*
 * File:          final_one.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP 64
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23
#define DELTA 3.14 * 2
#define RANGE (1024 / 2)



double X 	= 0;
double Y 	= 0;
double D 	= 0;
double TIME = 0;

struct object_detection {
	int x_1;
	int x_2;
	int is_detected;
	
};



static double compute_odometry() 

{
    WbDeviceTag ps[2];
    char ps_names[2][10] = { "ps0", "ps1"};

       for (i = 0; i < 2; i++) {
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
}

	double  l =  wb_position_sensor_get_value('ps0');
	
	double  r =  wb_position_sensor_get_value('ps1');
	
	
	// distance covered by left wheel in meter
	double dl = l / ENCODER_RESOLUTION * WHEEL_RADIUS;
	
	// distance covered by right wheel in meter
	double dr = r / ENCODER_RESOLUTION * WHEEL_RADIUS;
	
	// delta orientation
	double da = (dr - dl) / AXLE_LENGTH;

	X = X + (((dr + dl) / 2) * cos(da));
	Y = Y + (((dr + dl) / 2) * sin(da)); 
	
	// printf("X: %3.3f m | Y: %3.3f m | D: %3.3f\n", X, Y, da);

	// printf("estimated change of orientation: %g rad.\n", da);
	
	return da;
}









// look at each pixel of the image from the camera
// - if the RED pixel is detected, return its location on x axis
// - otherwise return -1
struct object_detection goal_exist(WbDeviceTag camera) 
{
	// get the image from the camera
	const unsigned char * image = wb_camera_get_image(camera);
	
	// get the width of the image
	int image_width = wb_camera_get_width(camera);
	
	// get the height of the image
	int image_height = wb_camera_get_height(camera);
	
	int x = 0, y = 0, r = 0, g = 0, b = 0, first = 0;
	
	struct object_detection OB;
	OB.x_1 = -1;
	OB.x_2 = -1;
	OB.is_detected = -1;

	// for each pixel
	for (x = 0; x < image_width; x++)
	{
		for (y = 0; y < image_height; y++)
		{
			// get the intensity value of the current pixel on R channel
			r = wb_camera_image_get_red(image, image_width, x, y);
			
			// get the intensity value of the current pixel on G channel
			g = wb_camera_image_get_green(image, image_width, x, y);
			
			// get the intensity value of the current pixel on B channel
			b = wb_camera_image_get_blue(image, image_width, x, y);

			// check the current pixel is RED or not
			// - if the RED pixel is detected, return its location on x axis
			// NOTE: CHOOSE TOLERANCE VALUES CAREFULLY!
			if((r >= 140 && r <= 255) && (g == 0) && (b == 0) && (first == 0))
			{
				// printf("x: %d, y: %d | R = %d G = %d B = %d\n", x, y, r, g, b);
				OB.is_detected = 0;
				OB.x_1 = x;
				first = 1;
				// return x;
			}
			
			if((r >= 140 && r <= 255) && (g == 0) && (b == 0) && (first == 1))
			{
				// printf("x: %d, y: %d | R = %d G = %d B = %d\n", x, y, r, g, b);
				OB.x_2 = x;
				// return x;
			}
		}		
	}
	
	// printf("x_1: %d | x_2: %d | ? = %d\n", OB.x_1, OB.x_2, OB.is_detected);

	return OB;
}
//-----------//
// STATE - 0 //
//-----------//

// rotate and check around
// - if you detect the goal, rotate the robot to it (CHANGE STATE TO 1)
// - otherwise move randomly around (CHANGE STATE TO 3)
int detect_goal(WbDeviceTag camera, WbDeviceTag wheels[4], WbDeviceTag ds[2], double delta)
{
	int  left_speed = 1;
    int  right_speed = 1;
     int avoid_obstacle_counter = 0;
     int i;


    if (avoid_obstacle_counter > 0) {
      avoid_obstacle_counter--;
      left_speed = 1;
      right_speed = -1;
    } else {
      // read sensors outputs
      double ds_values[2];
      for (i = 0; i < 2; i++)
        ds_values[i] = wb_distance_sensor_get_value(ds[i]);

      // increase counter in case of obstacle
      if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
        avoid_obstacle_counter = 100;
    }

    // write actuators inputs
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    wb_motor_set_velocity(wheels[2], left_speed);
    wb_motor_set_velocity(wheels[3], right_speed);
	
	struct object_detection OB = goal_exist(camera);
	// rotate and check around
	// - if you detect the goal, rotate the robot to it (CHANGE STATE TO 1)
	if(OB.is_detected != -1)
	{
		return 1;
	}

	// CHECK DELTA - rotate one whole tour
	// - otherwise move randomly around (CHANGE STATE TO 3)
	if(delta > DELTA + D)
	{
		return 3;
	}
	
	return 0;
}
//-----------//
// STATE - 1 //
//-----------//

// check the goal whether it is in the center of the camera or not 
// - if the goal is in the center of the camera (CHANGE STATE TO 2)
// - otherwise continue to rotate the robot until the
int rotate_to_goal(WbDeviceTag camera,WbDeviceTag wheels[4])
{
	// get the width of the image
	int image_width = wb_camera_get_width(camera);
	
	// get the location of the RED pixel
	struct object_detection OB = goal_exist(camera);

	// - if you don't detect the goal, try to find it (CHANGE STATE TO 0)
	if(OB.is_detected == -1)
	{
		D = compute_odometry();
		return 0;
	}

	// calculate the center of the image
	int center = image_width / 2;

	int half = ((OB.x_2 - OB.x_1) / 2);

	// printf("C: %d - X_1: %d = %d ||| H: %d ||| X_2: %d - C: %d = %d\n", center, OB.x_1, (center - OB.x_1), half, OB.x_2, center, (OB.x_2 - center));

	// if the goal is in the center of the camera (CHANGE STATE TO 2)
	if((center - OB.x_1) <= half)// && (OB.x_2 - center) <= half)
	{
		return 2;
	}

	int direction = 1;
	
	// decide which side robots should return
	if((OB.x_1 - center) < 0)
	// if((OB.x_1 - center) > (OB.x_2 - center))
	{
		direction = -1;
	} 

	// rotate the robot slowly
		
	  wb_motor_set_velocity(wheels[0], direction*5);
             wb_motor_set_velocity(wheels[1], -direction*5);
             wb_motor_set_velocity(wheels[2], direction*5);
             wb_motor_set_velocity(wheels[3], -direction*5);

	// otherwise continue to rotate the robot until the object is in the center of the camera (SAME STATE)
	return 1;
}
//
// STATE - 2 //
//-----------//

// move to forward
// - if the robot reaches the goal, then stop it
// - if you detect the goal, rotate the robot to it (CHANGE STATE TO 1)
// - otherwise move to the goal until the robot reaches the goal
int move_to_goal( WbDeviceTag ds[2],WbDeviceTag wheels[4], WbDeviceTag camera)
{
	int i;
	
	double sensors_value[2];
	
	struct object_detection OB = goal_exist(camera);

	// get the sensors values
	for (i = 0; i < 2; i++) 
	{
		sensors_value[i] = wb_distance_sensor_get_value(ds[i]);
	}

	// move to forward
	 double left_speed = 1.0;
	 double right_speed = 1.0;
             wb_motor_set_velocity(wheels[0], left_speed);
             wb_motor_set_velocity(wheels[1], right_speed);
             wb_motor_set_velocity(wheels[2], left_speed);
             wb_motor_set_velocity(wheels[3], right_speed);
    
    


	// printf("P0: %f | P7: %f\n", sensors_value[0], sensors_value[7]);

	// - if the robot reaches the goal, then stop it
	// NOTE: CHOOSE TOLERANCE VALUES CAREFULLY! (500)
	if(sensors_value[0] > 500 || sensors_value[1] > 500)
	{
		 wb_motor_set_velocity(wheels[0], 0);
		 wb_motor_set_velocity(wheels[1], 0);
		 wb_motor_set_velocity(wheels[2], 0);
		 wb_motor_set_velocity(wheels[3], 0);
		   
		
		return 4;
	}

	// - if you detect the goal, rotate the robot to it (CHANGE STATE TO 1)
	if(OB.is_detected != -1)
	{
		return 1;
	}
	
	// - otherwise move to the goal until the robot reaches the goal
	return 2;
}
//-----------//
// STATE - 3 //
//-----------//
	
// move and check around
// - if you detect the goal, rotate the robot to it (CHANGE STATE TO 1)
// - otherwise continue to move randomly around (SAME STATE)	
int move_randomly(WbDeviceTag ds[2],WbDeviceTag wheels[4], WbDeviceTag camera)
{
	int i, j;
	
	double speed[2];
	
	double sensors_value[2];

	double braitenberg_coefficients[8][2] = { {150, -35}, {100, -15} };
	
	TIME = TIME + 1.0;
	
	printf("TIME: %f", TIME);

	// get the sensors values
	for (i = 0; i < 2; i++) 
	{
		sensors_value[i] = wb_distance_sensor_get_value(ds[i]);
	}
	
	for (i = 0; i < 2; i++) 
	{
		speed[i] = 0.0;
		  
		for (j = 0; j < 2; j++)
		{
			speed[i] += braitenberg_coefficients[j][i] * (1.0 - (sensors_value[j] / RANGE));
		}
	}
	
	// move and check around
	 wb_motor_set_velocity(wheels[0], speed[0]);
            wb_motor_set_velocity(wheels[1], speed[1]);
            wb_motor_set_velocity(wheels[2], speed[0]);
            wb_motor_set_velocity(wheels[3], speed[1]);

	struct object_detection OB = goal_exist(camera);
	
	// - if you detect the goal, rotate the robot to it (CHANGE STATE TO 1)
	if(OB.is_detected != -1)
	{
		return 1;
	}

	// - otherwise continue to move randomly around (SAME STATE)	
	return 3;
}



/*
 * You may want to add macros here.
 */


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
   WbDeviceTag camera = wb_robot_get_device("camera");
 wb_camera_enable(camera, 2 * TIME_STEP);
 wb_camera_recognition_enable(camera, TIME_STEP);
  int avoid_obstacle_counter = 0;
  int i, state = 0;
  double delta = 0;
  // initialise distance sensors
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }

  // initialise motors
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
WbDeviceTag ps[2];
char ps_names[2][10] = { "ps0", "ps1"};

 for (i = 0; i < 2; i++) {
  ps[i] = wb_robot_get_device(ps_names[i]);
  wb_distance_sensor_enable(ps[i], TIME_STEP);
}

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
  
  delta = compute_odometry();
		
		switch(state)
		{
			case 0: state = detect_goal(camera,wheels,ds, delta);	break;
			
			case 1: state = rotate_to_goal(camera,wheels); break;
			
			case 2: state = move_to_goal(ds,wheels, camera);	break;
			
			case 3: state = move_randomly(ds,wheels, camera);	break;
			
			default: break;											break;
		}
	}

	wb_robot_cleanup();

	return 0;
}

   