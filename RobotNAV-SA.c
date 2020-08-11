
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}


static void display_helper_message() {
  printf("Control commands:\n");
  printf(" Arrows:       Move the robot\n");
  printf(" Page Up/Down: Rotate the robot\n");
  printf(" +/-:          (Un)grip\n");
  printf(" Shift + arrows:   Handle the arm\n");
  printf(" Space: Reset\n");
}






int main(int argc, char **argv) {
  wb_robot_init();
  
  int i;
  WbDeviceTag ds[10];
  char ds_names[10][10] = {"ds_1" ,"ds_2","ds_5","ds_6","ds_7" ,"ds_8","ds_9","ds_10","ds_11","ds_12"};
  double ds_values[10];
  bool avoid_obstacle_counter = 0;

  for (i = 0; i < 10; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  
  base_init();
  arm_init();
  gripper_init();
  arm_reset();
  base_reset();
  passive_wait(3.0);


  
  base_goto_init(TIME_STEP);
  passive_wait(2.0);
  base_goto_set_target(4.96, -3.19, 0);
  passive_wait(2.0); 


 
  while (!base_goto_reached()) {
    for (i = 0; i < 10; i++){
     ds_values[i] = wb_distance_sensor_get_value(ds[i]);
  }

   if ( ds_values[0] < 950.0 || ds_values[1] < 950.0 ){
    printf("Avoiding Obstacle\n");
        if (avoid_obstacle_counter > 0) {
      avoid_obstacle_counter--;
      
      
    } else { 
      for (i = 0; i < 2; i++)
        ds_values[i] = wb_distance_sensor_get_value(ds[i]);
      if ( ds_values[0] < 950.0 ||ds_values[1] < 950.0 )
        avoid_obstacle_counter = 100;
         base_turn_right();
        
    }
    }




   else if ( ds_values[2] < 950.0 ||ds_values[3] < 950.0 ||ds_values[4] < 950.0 ||ds_values[5] < 950.0 ||ds_values[6] < 950.0 ||ds_values[7] < 950.0 ||ds_values[8] < 950.0 ||ds_values[9] < 950.0  ){
    printf("Avoiding Obstacle\n");
        if (avoid_obstacle_counter > 0) {
      avoid_obstacle_counter--;
      
      
    } else { 
      for (i = 2; i < 10; i++)
        ds_values[i] = wb_distance_sensor_get_value(ds[i]);
      if (ds_values[2] < 950.0 ||ds_values[3] < 950.0 || ds_values[4] < 950.0 ||ds_values[5] < 950.0 ||ds_values[6] < 950.0 ||ds_values[7] < 950.0 ||ds_values[8] < 950.0 ||ds_values[9] < 950.0 )
        avoid_obstacle_counter = 100;
          base_forwards();
        
    }
    }

    else 
    base_goto_run();
    step();
    printf("Going to Point A\n");
    }
  
  
  
  base_reset();
  printf("Point A Reached\n");
  passive_wait(2.0);
  
  if(base_goto_reached()){
  base_goto_set_target(4.96, -3.19, 0);
};
 
   while (!base_goto_reached()) {
    base_goto_run();
    step();
    printf("Going to Point B\n");
  }
  
  base_reset();
  printf("Point B Reached\n");
  passive_wait(2.0);
  
  if(base_goto_reached()){
  base_goto_set_target(4.96, -3.19, 0);
};
 
   while (!base_goto_reached()) {
    base_goto_run();
    step();
    printf("Going to Point C\n");
  }
  
  base_reset();
  printf("Point C Reached\n");
  passive_wait(2.0);
  


  display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);

  while (true) {
    step();

    int c = wb_keyboard_get_key();
    if ((c >= 0) && c != pc) {
      switch (c) {
        case WB_KEYBOARD_UP:
          printf("Go forwards\n");
          base_forwards();
          break;
        case WB_KEYBOARD_DOWN:
          printf("Go backwards\n");
          base_backwards();
          break;
        case WB_KEYBOARD_LEFT:
          printf("Strafe left\n");
          base_strafe_left();
          break;
        case WB_KEYBOARD_RIGHT:
          printf("Strafe right\n");
          base_strafe_right();
          break;
        case WB_KEYBOARD_PAGEUP:
          printf("Turn left\n");
          base_turn_left();
          break;
        case WB_KEYBOARD_PAGEDOWN:
          printf("Turn right\n");
          base_turn_right();
          break;
        case WB_KEYBOARD_END:
        case ' ':
          printf("Reset\n");
          base_reset();
          arm_reset();
          break;
        case '+':
        case 388:
        case 65585:
          printf("Grip\n");
          gripper_grip();
          break;
        case '-':
        case 390:
          printf("Ungrip\n");
          gripper_release();
          break;
        case 332:
        case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
          printf("Increase arm height\n");
          arm_increase_height();
          break;
        case 326:
        case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          printf("Decrease arm height\n");
          arm_decrease_height();
          break;
        case 330:
        case WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT:
          printf("Increase arm orientation\n");
          arm_increase_orientation();
          break;
        case 328:
        case WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT:
          printf("Decrease arm orientation\n");
          arm_decrease_orientation();
          break;
        default:
          fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
    }
    pc = c;
  }

  wb_robot_cleanup();

  return 0;
}