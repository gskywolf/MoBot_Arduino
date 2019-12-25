#ifndef PIBOT_MOTOR_CONTROLLER_H_
#define PIBOT_MOTOR_CONTROLLER_H_

class MotorController{
  public:
    virtual void init()=0;
    virtual void control(short pwm)=0;
};

#endif