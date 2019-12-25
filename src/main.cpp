/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"

#include "robot.h"


void setup()
{
    Robot::get()->init();
}

void loop()
{   
    Robot::get()->check_command();
    Robot::get()->do_kinmatics();
    Robot::get()->calc_odom();
    Robot::get()->get_imu_data();
    Robot::get()->check_joystick();
}