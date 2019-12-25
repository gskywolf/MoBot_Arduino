#include "afs_motor_controller.h"
#include "board.h"
#include "data_holder.h"

AFSMotorController::AFSMotorController(int num, unsigned short _pwm_max): motor(num), pwm_max(_pwm_max){
}

void AFSMotorController::init(){
    last_pwm = 0;
    motor.run(RELEASE);
}

void AFSMotorController::control(short pwm){
    if (last_pwm == pwm)
        return;
    last_pwm = pwm;

    if (pwm > 0){
        motor.run(FORWARD);

        if (pwm <= short(pwm_max))
            motor.setSpeed(pwm);  
        else
            motor.setSpeed(pwm_max);  
    }else if(pwm<0){
        motor.run(BACKWARD);

        if (pwm >= -short(pwm_max))
            motor.setSpeed(-pwm);
        else
            motor.setSpeed(pwm_max); 
    }
    else{
        motor.run(RELEASE);
    }
}