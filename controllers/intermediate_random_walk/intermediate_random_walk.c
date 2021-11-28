/*
 * File:          intermediate_random_walk.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>


#define TIME_STEP 64
#define WHEEL_SPEED 2


// Define indexes of distance sensors
#define SENS_NUM 8
#define FR 0
#define FL 7

#define NINETY_DEG 3.14 * 3.14/4.42

// Define states
#define FORWARD 0
#define LEFT 1
#define RIGHT 2


// Declare robot devices
WbDeviceTag sensors[SENS_NUM];
WbDeviceTag left_motor, right_motor;
WbDeviceTag left_position_sensor;


int state = FORWARD;
double starting_pos = 0;
double target_turn  = 0;

static double rand_dec() {
  return (double)rand() / (double)RAND_MAX;
}

static void forward() {
  double fl_dist, fr_dist;
  bool right_obstacle, left_obstacle;
  
  fr_dist = wb_distance_sensor_get_value(sensors[FR]);
  fl_dist = wb_distance_sensor_get_value(sensors[FL]);
  
  right_obstacle = fr_dist > 80;
  left_obstacle  = fl_dist > 80;
  
  wb_motor_set_velocity(right_motor, WHEEL_SPEED);
  wb_motor_set_velocity(left_motor, WHEEL_SPEED);
  
  
  state        = left_obstacle ? RIGHT : (right_obstacle ? LEFT : FORWARD);
  starting_pos = wb_position_sensor_get_value(left_position_sensor);
  target_turn  = 0;
}

static void left() {
  double current_pos;
  bool turn_complete;
  
  if (target_turn == 0)
    target_turn = NINETY_DEG + rand_dec() * NINETY_DEG;
  
  current_pos   = wb_position_sensor_get_value(left_position_sensor);
  turn_complete = current_pos <= -target_turn + starting_pos;
  
  wb_motor_set_velocity(right_motor, WHEEL_SPEED);
  wb_motor_set_velocity(left_motor, -WHEEL_SPEED);
  
  state = turn_complete ? FORWARD : LEFT;
}

static void right() {
  double current_pos;
  bool turn_complete;
  
  if (target_turn == 0)
    target_turn = NINETY_DEG + rand_dec() * NINETY_DEG;
  
  current_pos   = wb_position_sensor_get_value(left_position_sensor);
  turn_complete = current_pos >= starting_pos + target_turn;
  
  wb_motor_set_velocity(right_motor, -WHEEL_SPEED);
  wb_motor_set_velocity(left_motor, WHEEL_SPEED);
  
  state = turn_complete ? FORWARD : RIGHT;
}

static void run() {
  switch(state) {
    case FORWARD:
      forward();
      break;
    case LEFT:
      left();
      break;
    case RIGHT:
      right();
      break;
    default:
      forward();
      break;
  }
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  srand(time(0));

  char sensor_name[] = "ps0";
  int idx;
  for (idx = 0; idx < SENS_NUM; idx++) {
    sensors[idx] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[idx], TIME_STEP);
    sensor_name[2]++;
  }
  
  left_motor  = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  wb_position_sensor_enable(left_position_sensor,TIME_STEP);


  while (wb_robot_step(TIME_STEP) != -1) {
    run();
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
