/*
 * File:          intermediate_scanning_walk.c
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
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Timestep
#define TIME_STEP 64
#define WHEEL_SPEED 2
#define PI 3.14
#define KL PI/2.2684
#define KR PI/2.26

// States
#define FORWARD 0
#define UTURN 1
#define RIGHT 2
#define LEFT 3

// Forward states
#define L 0
#define R 1
#define L2R 2
#define R2L 3

WbDeviceTag sensor_fr, sensor_fl, sensor_r, sensor_l;
WbDeviceTag motor_r, motor_l;
WbDeviceTag pos_sensor_l;

double starting_pos;
int state = FORWARD, f_state = L;

char* state_names[]   = {"Forward","U-turn","Right","Left"};
char* f_state_names[] = {"Left", "Right", "Left to Right", "Right to Left"};
static bool obstacle_front() {
  double dist_fr, dist_fl;
  dist_fr = wb_distance_sensor_get_value(sensor_fr);
  dist_fl = wb_distance_sensor_get_value(sensor_fl);
  
  return dist_fr > 80 || dist_fl > 80;
}

static bool obstacle_right() {
  return wb_distance_sensor_get_value(sensor_r) > 80;
}

static bool obstacle_left() {
  return wb_distance_sensor_get_value(sensor_l) > 80;
}

static void forward() {
  wb_motor_set_velocity(motor_r, WHEEL_SPEED);
  wb_motor_set_velocity(motor_l, WHEEL_SPEED);
}

static void forward_dist() {
  double current_pos, target_angle;
  bool completed_forward, is_L2R;
  
  current_pos  = wb_position_sensor_get_value(pos_sensor_l);
  target_angle = PI;
  
  completed_forward = current_pos >= target_angle + starting_pos;
  is_L2R            = f_state == L2R;
  f_state           = completed_forward ? (is_L2R ? R : L) : f_state;
  state             = completed_forward ? (is_L2R ? RIGHT : LEFT) : state;
  
  wb_motor_set_velocity(motor_r, WHEEL_SPEED);
  wb_motor_set_velocity(motor_l, WHEEL_SPEED);
}

static void right() {
  double target_angle, current_pos;
  bool completed_turn;
  
  current_pos  = wb_position_sensor_get_value(pos_sensor_l);
  target_angle = PI/2 * KR;
  
  completed_turn = current_pos >= target_angle + starting_pos;
  state          = completed_turn ? FORWARD : RIGHT;
  
  wb_motor_set_velocity(motor_r, -WHEEL_SPEED/2);
  wb_motor_set_velocity(motor_l, WHEEL_SPEED/2);
}

static void left() {
  double target_angle, current_pos;
  bool completed_turn;
  
  current_pos    = wb_position_sensor_get_value(pos_sensor_l);
  target_angle   = PI/2 * KL;
  
  completed_turn = current_pos <= -target_angle + starting_pos;
  state          = completed_turn ? FORWARD : LEFT;
  
  wb_motor_set_velocity(motor_r, WHEEL_SPEED/2);
  wb_motor_set_velocity(motor_l, -WHEEL_SPEED/2);
}

static void uturn() {
  double target_angle, current_pos;
  bool completed_turn;
  
  current_pos  = wb_position_sensor_get_value(pos_sensor_l);
  target_angle = PI * KR;
  
  completed_turn = current_pos >= target_angle + starting_pos;
  state          = completed_turn ? UTURN : FORWARD;
  
  wb_motor_set_velocity(motor_r, -WHEEL_SPEED/2);
  wb_motor_set_velocity(motor_l, WHEEL_SPEED/2);
}

static void run() {
  bool of, or, ol, fr, fl;
  of = obstacle_front();
  or = obstacle_right();
  ol = obstacle_left();
  fr = !or;
  fl = !ol;
  
  bool enclosed, right_corner, left_corner, next_col_right, next_col_left;

  enclosed       = of && or && ol;
  right_corner   = of && or && fl;
  left_corner    = of && ol && fr;
  next_col_right = of && fr;
  next_col_left  = of && fl;
  
  int prev_state = state;
  
  switch(state) {
    case FORWARD:
      (f_state == L || f_state == R) ? forward() : forward_dist();
  
      if (enclosed) {
        state   = UTURN;
        f_state = (f_state == L2R) ? R : (f_state == R2L) ? L : f_state;
      } 
      
      switch(f_state) {
        case L:
          state   = right_corner ? LEFT : next_col_right ? RIGHT : state;
          f_state = right_corner ? R : next_col_right ? L2R : f_state;
          break;
        case R:
          state   = left_corner ? RIGHT : next_col_left ? LEFT : state;
          f_state = left_corner ? L : next_col_left ? R2L : f_state;
          break;
        case L2R:
          state = next_col_right ? RIGHT : state;
          break;
        case R2L:
          state = next_col_left ? LEFT : state;
          break;
        default:
          state   = FORWARD;
          f_state = L;
          break;
      }
      
      break;
    case UTURN:
      uturn();
      break;
    case RIGHT:
      right();
      break;
    case LEFT:
      left();
      break;
    default:
      state = FORWARD;
      break;
  }
  
  if (prev_state != state) {
    wb_motor_set_velocity(motor_r, 0);
    wb_motor_set_velocity(motor_l, 0);
    starting_pos = wb_position_sensor_get_value(pos_sensor_l);
    printf("next state: %s\n", state_names[state]);
    printf("next f state: %s\n", f_state_names[f_state]);
    printf("\n");
  }
                 
}

int main(int argc, char **argv) {
  wb_robot_init();
  
  sensor_fr = wb_robot_get_device("ps0");
  sensor_fl = wb_robot_get_device("ps7");
  sensor_r  = wb_robot_get_device("ps2");
  sensor_l  = wb_robot_get_device("ps5");
  
  wb_distance_sensor_enable(sensor_fr, TIME_STEP);
  wb_distance_sensor_enable(sensor_fl, TIME_STEP);
  wb_distance_sensor_enable(sensor_r, TIME_STEP);
  wb_distance_sensor_enable(sensor_l, TIME_STEP);
  
  motor_r = wb_robot_get_device("right wheel motor");
  motor_l = wb_robot_get_device("left wheel motor");
  
  wb_motor_set_position(motor_r, INFINITY);
  wb_motor_set_position(motor_l, INFINITY);
  wb_motor_set_velocity(motor_r, 0);
  wb_motor_set_velocity(motor_l, 0);
  
  pos_sensor_l = wb_robot_get_device("left wheel sensor");
  wb_position_sensor_enable(pos_sensor_l, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    run();
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
