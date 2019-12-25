#ifndef PIBOT_AF_MOTOR_CONTROLLER_H_
#define PIBOT_AF_MOTOR_CONTROLLER_H_

#include "motor_controller.h"
#include "AFMotor.h"

class AFSMotorController:public MotorController{
  public:
    AFSMotorController(int num, unsigned short _pwm_max);
    ~AFSMotorController(){}
    void init();
    void control(short pwm);
  private:
    AF_DCMotor motor;
    unsigned short pwm_max;
    short last_pwm;
};

#endif