/*
 * File:          robot_controller.c
 * Date:
 * Description:
 * Author: Helmuth August Risch Filho
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <webots/robot.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define VELOCIDADE_MAXIMA 6.28
#define NUMBER_OF_DISTANCE_SENSORS 8
#define SENSOR_VALUE_DETECTION_THRESHOLD 140
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.588111888

static WbDeviceTag distance_sensors[NUMBER_OF_DISTANCE_SENSORS];
static const char *distance_sensors_names[NUMBER_OF_DISTANCE_SENSORS] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
static WbDeviceTag left_motor, right_motor;

static int TIME_STEP;

static void iniciar_robo() {
  wb_robot_init();
 
  TIME_STEP = (int)wb_robot_get_basic_time_step();
  
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 1.0);
  wb_motor_set_velocity(right_motor, 1.0);
  
  for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
      distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
      wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
  }

}

void motor_stop() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

void motor_move_forward() {
  wb_motor_set_velocity(left_motor, VELOCIDADE_MAXIMA);
  wb_motor_set_velocity(right_motor, VELOCIDADE_MAXIMA);
}

void motor_rotate_right() {
  wb_motor_set_velocity(left_motor, VELOCIDADE_MAXIMA);
  wb_motor_set_velocity(right_motor, -VELOCIDADE_MAXIMA);
}

void motor_rotate_left() {
  wb_motor_set_velocity(left_motor, -VELOCIDADE_MAXIMA);
  wb_motor_set_velocity(right_motor, VELOCIDADE_MAXIMA);
}

bool * get_sensors_condition()
{
  static bool sensors_condition[NUMBER_OF_DISTANCE_SENSORS] = {false};
	
  for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS ; i++) {
      if (wb_distance_sensor_get_value(distance_sensors[i]) > SENSOR_VALUE_DETECTION_THRESHOLD) {
	sensors_condition[i] = true;
      } else {
	sensors_condition[i] = false;
      }
  }
	
  return sensors_condition;
}
 
int main(int argc, char **argv) {
  iniciar_robo();
  
  while (wb_robot_step(TIME_STEP) != -1) {
    bool *is_sensors_active = get_sensors_condition();
		
    if(is_sensors_active[0] || is_sensors_active[1] || is_sensors_active[2]){
        motor_rotate_left();
    }
    else if(is_sensors_active[7] || is_sensors_active[6] || is_sensors_active[5]){
        motor_rotate_right();
    }else{
        motor_move_forward();
    }
  }

  wb_robot_cleanup();

  return 0;
}