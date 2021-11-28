// Included libraries
#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 32   
#define SENS_NUM 8
#define WHEEL_SPEED 2

// Define motor locations
#define FR 0
#define FFR 1
#define R 2
#define L 5
#define FFL 6
#define FL 7

// Define FSM states
#define FIND_WALL 0
#define ROTATE 1
#define FOLLOW 2
#define RIGHT 3
#define LEFT 4


WbDeviceTag sensors[SENS_NUM];
WbDeviceTag left_motor, right_motor;

int state = FIND_WALL;

static void find_wall() {
  double speed, front_l, front_r;
  bool collision;
  
  front_l = wb_distance_sensor_get_value(sensors[FL]);
  front_r = wb_distance_sensor_get_value(sensors[FR]);
  
  collision = (front_l > 80 && front_r > 80);
  speed     = !collision * WHEEL_SPEED;
  state     = collision ? ROTATE : FIND_WALL;
  
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, speed);
}

static void rotate() {
  double speed, ffront_l, left;
  bool wall_on_left;
  
  ffront_l = wb_distance_sensor_get_value(sensors[FFL]);
  left     = wb_distance_sensor_get_value(sensors[L]);
  
  wall_on_left = left > 80 && ffront_l < 70;
  speed        = !wall_on_left * WHEEL_SPEED;
  state        = wall_on_left ? FOLLOW : ROTATE; 
  
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, -speed);
}

static void follow() {
  double front_l, front_r, ffront_l, left;
  bool too_close, too_far, collision;
  
  front_l  = wb_distance_sensor_get_value(sensors[FL]);
  front_r  = wb_distance_sensor_get_value(sensors[FR]);
  ffront_l = wb_distance_sensor_get_value(sensors[FFL]);
  left     = wb_distance_sensor_get_value(sensors[L]);
  
  too_close = left > 85 || ffront_l > 85;
  too_far   = !too_close && left < 80;
  collision = (front_l > 80 && front_r > 80);
  
  state = collision ? 
          ROTATE : 
          too_close ? 
          RIGHT : 
          too_far ? 
          LEFT : 
          FOLLOW;
          
  if (state == FOLLOW) {
    wb_motor_set_velocity(left_motor, WHEEL_SPEED);
    wb_motor_set_velocity(right_motor, WHEEL_SPEED);
  }
}

static void adjust_right() {
  double speed = WHEEL_SPEED;
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, speed-0.2);
  state = FOLLOW;
}

static void adjust_left() {
  double speed = 2;
  wb_motor_set_velocity(left_motor, speed-0.2);
  wb_motor_set_velocity(right_motor, speed);
  state = FOLLOW;
}

static void run() {
  switch(state) {
    case FIND_WALL:
      find_wall();
      break;
    case ROTATE:
      rotate();
      break;
    case FOLLOW:
      follow();
      break;
    case RIGHT:
      adjust_right();
      break;
    case LEFT:
      adjust_left();
      break;
    default:
      find_wall();
      break; 
  }
}


int main() {
  wb_robot_init();
  
  char sensor_name[]= "ps0";
  int i;
  for(i = 0; i < SENS_NUM; i++){
    sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
    sensor_name[2]++;
  }

  // get a handler to the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 1);
  wb_motor_set_velocity(right_motor, 1);
  
  double reading;
  while (wb_robot_step(TIME_STEP) != -1) {
    reading = wb_distance_sensor_get_value(sensors[FL]);
    run();
  }

  wb_robot_cleanup();

  return 0;
}
