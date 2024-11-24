#include <webots/motor.h> // Motor library
#include <webots/robot.h> // Robot library
#include <webots/distance_sensor.h> // Distance sensor library
#include <webots/light_sensor.h> // Light sensor library
#include <stdio.h> // Standard C input and output functions
#include <stdbool.h> // Boolean functions

#define TIME_STEP 64 // Virtual time taken by robot to move from one step of the code to another in milliseconds
#define MAX_SPEED 6.28 // Highest speed for e-puck
#define PROXIMITY_THRESHOLD 80 // Sensor proxemity value
#define RIGHT_SENSOR_THRESHOLD 68 // For right sensor (used for dead end logic)
#define MAX_DEAD_ENDS 16 // Array size of 16 ensures the robot can gather light inensity from each dead end in one pass

int main(int argc, char **argv) // Main function, argc is argument count and argv is argument vector
{
  wb_robot_init(); // Used to initialize Webots controller
  
  // WbDeviceTag is used by the Webots controller to identify the device
  // wb_robot_get_device is a Webots API function that retrieves a device by name
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor"); // Initialize left motor
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor"); // Initialize right motor

  // wb_motor_set_position is a Webots API function that sets the target position for a motor
  // wb_motor_set_velocity sets the speed of the motor.
  // INFINITY tells Webots to ignore position control and instead enable velocity control mode
  // 0.0 is the desired speed in radians per second (rad/s)
  wb_motor_set_position(left_motor, INFINITY); // Set position of left motor at infity
  wb_motor_set_position(right_motor, INFINITY); // Set position of right motor at infity
  wb_motor_set_velocity(left_motor, 0.0); // Set initial velocity of left motor as 0.0 rad/s
  wb_motor_set_velocity(right_motor, 0.0); // Set initial velocity of left motor as 0.0 rad/s
  
  WbDeviceTag light_sensor[8]; // Creates an array to store the identifiers for all 8 light sensors (played safe and enabled all sensors)
  char light_sensor_name[50]; // Character array of size 50 to store the name of each light sensor temporarily.
  for (int j = 0; j < 8; j++) // For loop that loops 8 times (once for each proximity sensor). j is the variable used for sensor number
  {
    sprintf(light_sensor_name, "ls%d", j); // Stores sensor names (ls0 - ls7) in an array called light_sensor_name by using variable j
    light_sensor[j] = wb_robot_get_device(light_sensor_name); // Fetches the light sensor device with the name stored in light_sensor_name
    wb_light_sensor_enable(light_sensor[j], TIME_STEP); // Enables the light sensor at light_sensor[j]
  }
  
  WbDeviceTag prox_sensors[8]; // Creates an array to store the identifiers for all 8 proximity sensors (played safe and enabled all sensors)
  char prox_sensor_name[50]; // Character array of size 50 to store the name of each proximity sensor temporarily.
  for (int i = 0; i < 8; i++) // For loop that loops 8 times (once for each proximity sensor). i is the variable used for sensor number
  {
    sprintf(prox_sensor_name, "ps%d", i); // Stores sensor names (ps0 - ps7) in an array called prox_sensor_name by using variable i
    prox_sensors[i] = wb_robot_get_device(prox_sensor_name); // Fetches the proximity sensor device with the name stored in prox_sensor_name
    wb_distance_sensor_enable(prox_sensors[i], TIME_STEP); // Enables the proximity sensor at prox_sensors[i]
  }

  int dead_end_count = 0; // Integer variable dead_end_count initialized to 0
  bool in_dead_end = false; // Boolean variable in_dead_end is initialized to false
  double max_light_intensity = 0.0; // Double variable max_light_intensity initiaized to 0.0
  bool second_pass = false; // Boolean variable second_pass is initialized to false

  while (wb_robot_step(TIME_STEP) != -1) // wb_robot_step(TIME_STEP) advances the simulation by TIME_STEP milliseconds and != -1 keeps loop going
  {
   double light_value = wb_light_sensor_get_value(light_sensor[2]); // Reads values from light_sensor and assigns to light_value

    double prox_front = wb_distance_sensor_get_value(prox_sensors[7]); // Reads values from prox_sensors[7] and assigns to prox_front
    double prox_left = wb_distance_sensor_get_value(prox_sensors[4]); // Reads values from prox_sensors[5] and assigns to prox_left
    double prox_right = wb_distance_sensor_get_value(prox_sensors[2]); // Reads values from prox_sensors[2] and assigns to prox_right
    double prox_left_corner = wb_distance_sensor_get_value(prox_sensors[6]);
    double prox_right_corner = wb_distance_sensor_get_value(prox_sensors[1]);
    
    bool front_wall = prox_front > PROXIMITY_THRESHOLD; // Boolean for front sensor value above the proxemity threshold
    bool left_wall = prox_left > PROXIMITY_THRESHOLD; // Boolean for left sensor value above the proxemity threshold
    bool right_wall = prox_right > RIGHT_SENSOR_THRESHOLD; // Boolean for right sensor value above the right proxemity threshold
    bool left_corner_sense = prox_left_corner > PROXIMITY_THRESHOLD; // Boolean for left corner sensor value above the proxemity threshold
    bool right_corner_sense = prox_right_corner > RIGHT_SENSOR_THRESHOLD; // Boolean for right corner sensor value above the right proxemity threshold
    
    bool is_dead_end = (front_wall && left_wall && right_wall && right_corner_sense);
    // Boolean for dead end logic. A dead end is detected if the conditions for front_wall, left_wall, right_wall and right_corner_sense are true.

    if (is_dead_end) // If loop for when the conditions for a dead end is met
    {
      if (!in_dead_end && dead_end_count < MAX_DEAD_ENDS) 
      // The robot should only process a dead end if it is not already in a dead end and the recorded number of dead ends is less than MAX_DEAD_ENDS
      {
        if (!second_pass) // If loop for checking whether the robot is in the first pass (finding max intensity) or the second pass (returning to max intensity)
        // The if statement is for the robot not being in the second pass and the else statement is for the robot in the second pass
        {
          printf("Dead-end detected! Light intensity: %f\n", light_value); // Print light intensity when a dead end is detected
          
          if (light_value > max_light_intensity) // If statement used to replace the maximum light intensity when a higher light intensity is detected
          {
            max_light_intensity = light_value; // Changes variable max_light_intensity to new higher light intensity
          }
          dead_end_count++; // Adds one more dead end to array dead_end_count
        } 
        else // Second pass else statement
        {
          printf("Revisiting dead-end. Light intensity: %f\n", light_value); // Prints value of second pass light intensity
          if (light_value >= max_light_intensity) 
          // if statement to detect max light intensity dead end by stating that light at that point should be greated that or equal to max light intensity detected on first pass
          {
            printf("Stopping at dead end with light intensity: %f\n", light_value); // Prints message and max light intensity when above condition is met
            wb_motor_set_velocity(left_motor, 0.0); // Stops the left motor
            wb_motor_set_velocity(right_motor, 0.0); // Stops the right motor

            break; // Breaks loop
          }
        }

        in_dead_end = true; // Saves that the robot is currently in a dead end
        wb_robot_step(TIME_STEP * 10); // Delays robots actions by 10 TIME_STEP in ms. Used to avoid detecting same dead end multiple times
      }
    } 
    else 
    {
      in_dead_end = false; // Saves that robot is not currently in a dead end
    }

    if (!second_pass && dead_end_count >= MAX_DEAD_ENDS) // Checks whether robot is still in first pass and whether number of dead ends has reached maximum
    {
      printf("Completed first pass. Maximum light intensity: %f\n", max_light_intensity); 
      // Print message when first pass is completed. Also prints the maximum light intensity from the first pass
      printf("Starting second pass to revisit dead ends and sense maximum light intensity\n"); // Prints message that second pass is starting
      dead_end_count = 0; // Resets dead end count for second pass
      second_pass = true; // Sets second pass to true indicating the robot is in the second pass
    }

    double left_speed = MAX_SPEED; // Variable left_speed is set to MAX_SPEED
    double right_speed = MAX_SPEED; // Variable right_speed is set to MAX_SPEED

    if (front_wall) // If statement when a front wall is detected
    {
      left_speed = MAX_SPEED; // Left motor moves forward
      right_speed = -MAX_SPEED; // Right motor moves backward
      // Robot turns right
    } 
    else
    {
      if (left_wall) // If statement when a left wall is detected
      {
        left_speed = MAX_SPEED; // Left motor moves forward
        right_speed = MAX_SPEED; // Right motor moves forward
        // Robot moves forward
      } 
      else // Else statement for when no left wall is detected
      {
        left_speed = MAX_SPEED / 8; // Left motor moves slightly slower
        right_speed = MAX_SPEED; // Right motor moves forward
        // Robot turns slightly left to try find left wall again
      }
      if (left_corner_sense) // Left corner detection for smoother navigation and avoids robot getting stuck
      {
        left_speed = MAX_SPEED; // Left motor moves forward
        right_speed = MAX_SPEED / 8; // Right motor moves slightly slower
        // Robot turns slightly to the left
      }
    }

    wb_motor_set_velocity(left_motor, left_speed); // Applies robot speed to left motor
    wb_motor_set_velocity(right_motor, right_speed); // Applies robot speed to right motor
  }

  wb_robot_cleanup(); // Cleanup resources used

  return 0; // Exit
}
