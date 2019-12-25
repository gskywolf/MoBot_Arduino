#ifndef PIBOT_COMMON_MOTOR_CONTROLLER_H_
#define PIBOT_COMMON_MOTOR_CONTROLLER_H_

#include "motor_controller.h"

class CommonMotorController:public MotorController{
  public:
    CommonMotorController(int pwm_pin, int dir_pinA, int dir_pinB, unsigned short _pwm_max);
    ~CommonMotorController(){}
    void init();
    void control(short pwm);
  private:
    int pwm_pin, dir_pinA, dir_pinB;
    unsigned short pwm_max;
    short last_pwm;
};

#endif