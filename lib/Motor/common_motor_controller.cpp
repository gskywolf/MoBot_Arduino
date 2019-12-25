#include "common_motor_controller.h"
#include "board.h"
#include "data_holder.h"

CommonMotorController::CommonMotorController(int _pwm_pin, int _dir_pinA, int _dir_pinB, unsigned short _pwm_max): 
                pwm_pin(_pwm_pin), dir_pinA(_dir_pinA), dir_pinB(_dir_pinB), pwm_max(_pwm_max){
}

void CommonMotorController::init(){
    Board::get()->pin_init(dir_pinA, PIN_MODE_OUTPUT);
    Board::get()->pin_init(dir_pinB, PIN_MODE_OUTPUT);
    Board::get()->pwm_init(PWM_FREQUENCE);
    last_pwm = 0;

    Board::get()->pin_write(dir_pinA, 1);
    Board::get()->pin_write(dir_pinB, 1);
    Board::get()->pwm_output(pwm_pin, 0);
}

void CommonMotorController::control(short pwm){
    if (last_pwm == pwm)
        return;
    last_pwm = pwm;

    if (pwm > 0){
        Board::get()->pin_write(dir_pinA, 1);
        Board::get()->pin_write(dir_pinB, 0);

        if (pwm <= short(pwm_max))
            Board::get()->pwm_output(pwm_pin, pwm);
        else
            Board::get()->pwm_output(pwm_pin, pwm_max);
    }else if(pwm<0){
        Board::get()->pin_write(dir_pinA, 0);
        Board::get()->pin_write(dir_pinB, 1);

        if (pwm >= -short(pwm_max))
            Board::get()->pwm_output(pwm_pin, -pwm);
        else
            Board::get()->pwm_output(pwm_pin, pwm_max);
    }
    else{
        Board::get()->pin_write(dir_pinA, 1);
        Board::get()->pin_write(dir_pinB, 1);
        Board::get()->pwm_output(pwm_pin, 0);
    }
}