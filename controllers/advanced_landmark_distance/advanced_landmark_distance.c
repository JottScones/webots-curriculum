#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "ann.h"  // definition of the used ANN

// general defines
#define TRUE 1
#define FALSE 0
#define LEFT 0
#define RIGHT 1
#define TIME_STEP 64
#define SPEED_UNIT 0.00628

// states (for moving the robot)
#define STATIONARY 0
#define REVERSE 1
#define FORWARD 2

// modes (mode of the ANN algorithm)
#define NO_MODE 0
#define LEARN 1
#define TEST 2

// patterns (a landmark)
#define NUMBER_OF_PATTERNS 5
#define NO_PATTERN 0
#define P1 1
#define P2 2
#define P3 3
#define P4 4
// samples (an aera of the camera image which correspond to a landmark)
#define SAMPLE_MAX_NUMBER 5  // how many samples the e-puck can see at the same time
// devices
#define NB_LEDS 8
#define NB_DIST_SENS 8
#define PS_RIGHT_10 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_10 7

#define PI 3.1415926
#define WHEEL_INCREMENTS 1000
#define WHEEL_DIAM 0.04
#define WHEEL_CIRCUM PI*WHEEL_DIAM


static int state = STATIONARY;           // current state of the robot
static int mode = NO_MODE;        // current mode of the ANN algorithm
static int current_pattern = P1;  // current pattern for learning
static float samples[SAMPLE_MAX_NUMBER][2];
static int sample_counter = 0;
static WbDeviceTag cam;
static WbDeviceTag display;
static WbDeviceTag left_motor, right_motor;
static WbDeviceTag left_position_sensor;
static int speed[2] = {0, 0};
static int old_key = -1;        // last pressed key stroke
static float error = 1.0;
static double distance = 0.05;
static double prevPos, prevDist;

/*
 * This function displays the usage information
 */
static void usage() {
  printf("Commands (from keyboard):\n");
  printf("\n");

  printf("\n");
  printf("  L          : Enable (or disable) the learning mode\n");
  printf("  T          : Enable (or disable) the testing mode\n");
  printf("\n");
  printf("  O          : Load the weights stored in the weights.w file\n");
  printf("  S          : Save the weights stored in the weights.w file\n");
  printf("\n");
  printf("  R          : Reverse Robot.\n");
}

/*
 * This function rounds a float
 * to an int
 */
static int my_round(float x) {
  if (x >= 0)
    return (int)(x + 0.5);
  return (int)(x - 0.5);
}

/*
 * This function manage the last pressed
 * keyboard stroke.
 */
static void keyboard_manager() {
  int key = wb_keyboard_get_key();
  if (key != old_key) {  // for not call this function every TIME_STEP
    old_key = key;
    switch (key) {
      case 'R':
        if (state == REVERSE) {
          state = STATIONARY;
          printf("Reverse Stopped.\n");
        } else {
          prevPos = wb_position_sensor_get_value(left_position_sensor);
          prevDist = distance;
          printf("Reversing.\n");
          state = REVERSE;
        }
        break;
      case 'F':
        if (state == FORWARD) {
          state = STATIONARY;
          printf("Forward Stopped.\n");
        } else {
          prevPos = wb_position_sensor_get_value(left_position_sensor);
          prevDist = distance;
          printf("Going Forward.\n");
          state = FORWARD;
        }
        break;
      case 'L':
        if (mode == LEARN) {
          mode = NO_MODE;
          printf("Mode of the algorithm: No mode\n");
        } else {
          mode = LEARN;
          printf("Mode of the algorithm: Learn\n");
        }
        break;
      case 'T':
        if (mode == TEST) {
          mode = NO_MODE;
          printf("Mode of the algorithm: No mode\n");
        } else {
          mode = TEST;
          printf("Mode of the algorithm: Test\n");
        }
        break;
      case WB_KEYBOARD_UP:
        current_pattern++;
        current_pattern %= NUMBER_OF_PATTERNS;
        break;
      case WB_KEYBOARD_DOWN:
        current_pattern--;
        if (current_pattern < 0)
          current_pattern = NUMBER_OF_PATTERNS - 1;
        break;
      case 'S':
        SaveNetworkWeights(&network, "weights.w");
        printf("Weights of the ANN saved in weights.w\n");
        break;
      case 'O':
        LoadNetworkWeights(&network, "weights.w");
        printf("weights.w loaded in the weights of the ANN\n");
        break;
      case 'P':
        printf("Network is:\n");
        PrintNetwork(&network);
        break;
      default:
        break;
    }
    if (key == WB_KEYBOARD_UP || key == WB_KEYBOARD_DOWN || (key >= '1' && key <= '4')) {
      if (current_pattern == NO_PATTERN)
        printf("No Pattern selected for learning\n");
      else
        printf("Pattern %d is selected for learning\n", current_pattern);
    }
  }
}

/*
  Reverses robot back exactly 10mm
*/

static void reverse() {
  double currPos = wb_position_sensor_get_value(left_position_sensor);
  double currAngle = fabs(prevPos - currPos);
  distance = prevDist + (currAngle / ((double)(2.0*PI))) * ((double)WHEEL_CIRCUM);
  // printf("delta: %f\n", delta);
  // printf("Current distance: %f\n", distance);
  
  speed[LEFT] = -(100);
  speed[RIGHT] = -(100);
}

static void forward() {
  double currPos = wb_position_sensor_get_value(left_position_sensor);
  double currAngle = fabs(prevPos - currPos);
  distance = prevDist - (currAngle / ((double)(2.0*PI))) * ((double)WHEEL_CIRCUM);
  // printf("delta: %f\n", delta);
  // printf("Current distance: %f\n", distance);
  
  speed[LEFT] = (100);
  speed[RIGHT] = (100);
}

/*
 * This function perform the appropriate
 * action according to the state of the
 * robot. So, it just set the robot
 * motor speeds...
 */
static void robot_state_manager() {
  speed[LEFT] = 0;
  speed[RIGHT] = 0;
  switch (state) {
    case REVERSE:
      reverse();
      break;
    case FORWARD:
      forward();
      break;
    case STATIONARY:
    default:
      break;
  }
}

/*
 * This function search blue blobs
 * on the camera image. For each blue blob
 * found, it sample it by using a nearest
 * neighbor algorithm. The resulted sample
 * has a size of SAMPLE_WIDTH*SAMPLE_HEIGHT.
 * All the samples of the image are stored
 * in the "samples" array.
 * Finaly, this function displays the sample
 * array on the top left of the camera image.
 */
static void detect_blobs(void) {
  int i, j, k, x, y;
  const unsigned char *image = wb_camera_get_image(cam);
  int width = wb_camera_get_width(cam);
  int height = wb_camera_get_height(cam);

  // copy camera image to the display
  WbImageRef iref = wb_display_image_new(display, width, height, image, WB_IMAGE_BGRA);
  wb_display_image_paste(display, iref, 0, 0, false);

  // if the algorithm saw a blue pixel since the last sample
  int blue_pixel_detected = FALSE;

  // properties of a sample
  int sample_right = 0;
  int sample_left = width;
  int sample_up = 0;
  int sample_down = height;
  sample_counter = 0;

  // parse the image vertically
  for (i = 0; i < width; i++) {
    int blue_pixel_on_the_column = FALSE;
    for (j = 0; j < height; j++) {
      int gr = wb_camera_image_get_gray(image, width, i, j);
      if (gr>180) {  // blue pixel detected
        blue_pixel_detected = TRUE;
        blue_pixel_on_the_column = TRUE;
        // Narrow down the borders of the blob
        if (sample_right < i)
          sample_right = i;
        if (sample_left > i)
          sample_left = i;
        if (sample_up < j)
          sample_up = j;
        if (sample_down > j)
          sample_down = j;
      }
    }
    // When we can no longer detect blue pixels in column we have reached the end of the blob
    if ((i == width - 1 || blue_pixel_on_the_column == FALSE) && blue_pixel_detected == TRUE) {
      int sample_width = sample_right - sample_left + 1;
      int sample_height = sample_up - sample_down;
      if (sample_width > 1 && sample_height > 1 && sample_counter < SAMPLE_MAX_NUMBER) {
        samples[sample_counter][0] = ((float)(width - sample_width))/((float)width);
        samples[sample_counter][1] = ((float)(height - sample_height))/((float)height);
        // here a sample was seen
        // It should be sampled to SAMPLE_WIDTH*SAMPLE_HEIGHT and
        // be stored in the "samples" array and be displayed
        for (x = 0; x < SAMPLE_WIDTH; x++) {
          for (y = 0; y < SAMPLE_HEIGHT; y++) {
            int tmp_i = sample_left + my_round((float)x * sample_width / (float)SAMPLE_WIDTH);
            int tmp_j = sample_down + my_round((float)y * sample_height / (float)SAMPLE_HEIGHT);
            int value = wb_camera_image_get_blue(image, width, tmp_i, tmp_j);
            wb_display_set_color(display, (value << 16) | (value << 8) | value);
            wb_display_draw_pixel(display, x + sample_counter * (SAMPLE_WIDTH + 1), y);
          }
        }
        sample_counter++;
      }
      
      // Reset values for next blob to be detected (this assumes blobs are never on the same vertical)
      blue_pixel_detected = FALSE;
      sample_left = width;
      sample_right = 0;
      sample_up = 0;
      sample_down = height;
    }
  }

  // display the majenta bounds
  wb_display_set_color(display, 0xFF00FF);
  for (x = 0; x < sample_counter * (SAMPLE_WIDTH + 1); x++)
    wb_display_draw_pixel(display, x, SAMPLE_HEIGHT);
  for (k = 0; k < sample_counter; k++) {
    for (y = 0; y < SAMPLE_HEIGHT; y++)
      wb_display_draw_pixel(display, k + (1 + k) * SAMPLE_WIDTH, y);
  }
}

static int run(void) {
  // Manage the keyboard, move the robot,
  // and get the landmarks on the camera
  // (stored in the "samples" array)
  keyboard_manager();
  robot_state_manager();
  detect_blobs();
  // print_samples();

  /* Learning mode (only if the robot sees one (and only one) sample)
   *
   * 1. prepare the output (supervised learning) according to the selected pattern.
   *    It corresponds to the result one want to achieve
   *
   * 2. Put the sample values as input of the network
   *
   * 3. Run the backprop algorithm by specifing the desired output
   *
   * 4. Print the error
   */
  if (mode == LEARN && sample_counter == 1 && current_pattern >= P1 && current_pattern <= P4) {
    float output[1] = {0.0f};
    output[0] = distance;
    
    printf("input [%f][%f]\n", samples[0][0],samples[0][1]);

    InputToNetwork(&network, &samples[0][0]);
    ActivateNetwork(&network);
    error = TrainNetwork(&network, &output[0]);
    printf("Learned distance: %f. Resulted error: %f\n", distance, error);
  }

  /* Testing mode (only if the robot sees at least one sample)
   *
   * For each sample:
   *
   *   1. Put the sample values as input of the network
   *
   *   2. Read the output (ouput of the last layer of the ANN) and print
   *      the results (the maximum peak is concidered as the answer)
   */
  if (mode == TEST && sample_counter > 0) {
    int k;
    for (k = 0; k < sample_counter; k++) {
      InputToNetwork(&network, &samples[k][0]);
      ActivateNetwork(&network);
      // PrintNetworkOutput(&network);

      // get result
      layer_t output_layer = network.layers[network.size - 1];
      double guess = output_layer.y[0];

      // print result
      printf("Guess: %f, Real: %f\n", guess, distance);
    }
  }

  wb_motor_set_velocity(left_motor, SPEED_UNIT * speed[LEFT]);
  wb_motor_set_velocity(right_motor, SPEED_UNIT * speed[RIGHT]);

  return TIME_STEP;
}

/*
 * Main function
 */
int main() {
  wb_robot_init(); /* initialize the webots controller library */

  usage();

  RandomizeNetwork(&network);  // initiate the ANN with random numbers

  srand(time(0));  // initiate the random number generator

  // get the camera device
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP);

  // get the display device
  display = wb_robot_get_device("display");

  // enable the keyboard
  wb_keyboard_enable(TIME_STEP);

  // get the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1)
    run();

  wb_robot_cleanup();

  return 0;
}
