#include "robot.h"

#include "board.h"
#include "usart_transport.h"
#include "simple_dataframe_slave.h"
#include "data_holder.h"

#if MOTOR_CONTROLLER == COMMON_CONTROLLER
#include "common_motor_controller.h"
#define MAX_PWM_VALUE 1023
#elif MOTOR_CONTROLLER == AF_SHIELD_CONTROLLER
#include "afs_motor_controller.h"
#define MAX_PWM_VALUE 255
#endif

#include "encoder_implement.h"
#include "pid.h"

#if IMU_ENABLE
#include "GY85.h"
#endif

#if JOYSTICK_ENABLE
#include "joystick.h"
#endif

#include <stdio.h>

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))


void Robot::init(){
    Data_holder::get()->load_parameter();

#if DEBUG_ENABLE
    Board::get()->usart_debug_init();
#endif


#if DEBUG_ENABLE
    printf("RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d\r\n", 
        Data_holder::get()->parameter.wheel_diameter, Data_holder::get()->parameter.wheel_track,  Data_holder::get()->parameter.encoder_resolution, 
        Data_holder::get()->parameter.do_pid_interval, Data_holder::get()->parameter.kp, Data_holder::get()->parameter.ki, Data_holder::get()->parameter.kd, Data_holder::get()->parameter.ko, 
        Data_holder::get()->parameter.cmd_last_time, Data_holder::get()->parameter.max_v_liner_x, Data_holder::get()->parameter.max_v_liner_y, Data_holder::get()->parameter.max_v_angular_z);
#endif

#if IMU_ENABLE
    init_imu();
#endif

#if DEBUG_ENABLE
    printf("init_motor\r\n");
#endif
    init_motor();

#if DEBUG_ENABLE
    printf("init_trans\r\n");
#endif
    init_trans();

    init_joystick();

#if DEBUG_ENABLE
    printf("pibot startup\r\n");
#endif
}

void Robot::init_imu(){
#if IMU_ENABLE
    static IMU gy85;
    imu = &gy85;
    imu->init();
#endif
}

void Robot::init_joystick(){
#if JOYSTICK_ENABLE
    static Joystick joy;
    joystick = &joy;
    joystick->init();
#endif
}

void Robot::init_motor(){

#if MOTOR_COUNT>0
    #if MOTOR_CONTROLLER == COMMON_CONTROLLER
    static CommonMotorController motor1(MOTOR_1_PWM_PIN, MOTOR_1_DIR_A_PIN, MOTOR_1_DIR_B_PIN, MAX_PWM_VALUE);
    #elif MOTOR_CONTROLLER == AF_SHIELD_CONTROLLER
    static AFSMotorController motor1(MOTOR_1_PORT_NUM, MAX_PWM_VALUE);
    #endif

    static EncoderImp encoder1(MOTOR_1_ENCODER_A_PIN, MOTOR_1_ENCODER_B_PIN);
    static PID pid1(&input[0], &feedback[0], float(Data_holder::get()->parameter.kp)/Data_holder::get()->parameter.ko, 
                                            float(Data_holder::get()->parameter.ki)/Data_holder::get()->parameter.ko, 
                                            float(Data_holder::get()->parameter.kd)/Data_holder::get()->parameter.ko , MAX_PWM_VALUE);
#endif

#if MOTOR_COUNT>1
    #if MOTOR_CONTROLLER == COMMON_CONTROLLER
    static CommonMotorController motor2(MOTOR_2_PWM_PIN, MOTOR_2_DIR_A_PIN, MOTOR_2_DIR_B_PIN, MAX_PWM_VALUE);
    #elif MOTOR_CONTROLLER == AF_SHIELD_CONTROLLER
    static AFSMotorController motor2(MOTOR_2_PORT_NUM, MAX_PWM_VALUE);
    #endif
    static EncoderImp encoder2(MOTOR_2_ENCODER_A_PIN, MOTOR_2_ENCODER_B_PIN);
    static PID pid2(&input[1], &feedback[1], float(Data_holder::get()->parameter.kp)/Data_holder::get()->parameter.ko, 
                                            float(Data_holder::get()->parameter.ki)/Data_holder::get()->parameter.ko, 
                                            float(Data_holder::get()->parameter.kd)/Data_holder::get()->parameter.ko , MAX_PWM_VALUE);
#endif

#if MOTOR_COUNT>2
    #if MOTOR_CONTROLLER == COMMON_CONTROLLER
    static CommonMotorController motor3(MOTOR_3_PWM_PIN, MOTOR_3_DIR_A_PIN, MOTOR_3_DIR_B_PIN, MAX_PWM_VALUE);
    #elif MOTOR_CONTROLLER == AF_SHIELD_CONTROLLER
    static AFSMotorController motor3(MOTOR_3_PORT_NUM, MAX_PWM_VALUE);
    #endif

    static EncoderImp encoder3(MOTOR_3_ENCODER_A_PIN, MOTOR_3_ENCODER_B_PIN);
    static PID pid3(&input[2], &feedback[2], float(Data_holder::get()->parameter.kp)/Data_holder::get()->parameter.ko, 
                                            float(Data_holder::get()->parameter.ki)/Data_holder::get()->parameter.ko, 
                                            float(Data_holder::get()->parameter.kd)/Data_holder::get()->parameter.ko , MAX_PWM_VALUE);
#endif

#if MOTOR_COUNT>0
    motor[0] = &motor1;
    encoder[0] = &encoder1;
    pid[0] = &pid1;
#endif

#if MOTOR_COUNT>1
    motor[1] = &motor2;
    encoder[1] = &encoder2;
    pid[1] = &pid2;
#endif

#if MOTOR_COUNT>2
    motor[2] = &motor3;
    encoder[2] = &encoder3;
    pid[2] = &pid3;
#endif

#if ROBOT_MODEL == ROBOT_MODEL_DIFF
    static Differential diff(Data_holder::get()->parameter.wheel_diameter*0.0005, Data_holder::get()->parameter.wheel_track*0.0005);
    model = &diff;
#endif

#if ROBOT_MODEL == ROBOT_OMNI_3
    static Omni3 omni3(Data_holder::get()->parameter.wheel_diameter*0.0005, Data_holder::get()->parameter.wheel_track*0.0005);
    model = &omni3;
#endif

    for (int i=0;i<MOTOR_COUNT;i++){
        encoder[i]->init();
        motor[i]->init();
    }

    do_kinmatics_flag = false;

    memset(&odom, 0 , sizeof(odom));
    memset(&input, 0 , sizeof(input));
    memset(&feedback, 0 , sizeof(feedback));

    last_velocity_command_time = 0;
}

void Robot::init_trans(){
    static USART_transport _trans(MASTER_USART, 115200);
    static Simple_dataframe    _frame(&_trans);
    trans = &_trans;
    frame = &_frame;

    trans->init();
    frame->init();

    frame->register_notify(ID_SET_ROBOT_PARAMTER, this);
    frame->register_notify(ID_CLEAR_ODOM, this);
    frame->register_notify(ID_SET_VELOCITY, this);
}

void Robot::check_command(){
    unsigned char ch=0;
    if (trans->read(ch)){
        //printf("%02x ", ch);
        if (frame->data_recv(ch)){
            //printf("\r\n");
            frame->data_parse();
        }
    }

}
void Robot::update(const MESSAGE_ID id, void* data){
    switch (id){
    case ID_SET_ROBOT_PARAMTER:

#if DEBUG_ENABLE
    printf("RobotParameters: %d %d %d %d %d %d %d %d %d %d %d %d\r\n", 
        Data_holder::get()->parameter.wheel_diameter, Data_holder::get()->parameter.wheel_track,  Data_holder::get()->parameter.encoder_resolution, 
        Data_holder::get()->parameter.do_pid_interval, Data_holder::get()->parameter.kp, Data_holder::get()->parameter.ki, Data_holder::get()->parameter.kd, Data_holder::get()->parameter.ko, 
        Data_holder::get()->parameter.cmd_last_time, Data_holder::get()->parameter.max_v_liner_x, Data_holder::get()->parameter.max_v_liner_y, Data_holder::get()->parameter.max_v_angular_z);
#endif
        for (int i=0;i<MOTOR_COUNT;i++){
            pid[i]->update(float(Data_holder::get()->parameter.kp)/Data_holder::get()->parameter.ko, 
                                            float(Data_holder::get()->parameter.ki)/Data_holder::get()->parameter.ko, 
                                            float(Data_holder::get()->parameter.kd)/Data_holder::get()->parameter.ko , MAX_PWM_VALUE);
        }

        model->set(Data_holder::get()->parameter.wheel_diameter*0.0005, Data_holder::get()->parameter.wheel_track*0.0005);

        Data_holder::get()->save_parameter();
        break;
    case ID_CLEAR_ODOM:
        clear_odom();
        break;
    case ID_SET_VELOCITY:
        update_velocity();
        break;
    default:
        break;
    }
}

void Robot::clear_odom(){
    for(int i=0;i<MOTOR_COUNT;i++){
        encoder[i]->clear();
    }
    
	memset(&odom, 0, sizeof(odom));
	memset(&Data_holder::get()->odom, 0, sizeof(Data_holder::get()->odom));
}

#define __PI  3.1415926535897932384626433832795
void Robot::update_velocity(){
    short vx = min(max(Data_holder::get()->velocity.v_liner_x, -(short(Data_holder::get()->parameter.max_v_liner_x))), short(Data_holder::get()->parameter.max_v_liner_x));
    short vy = min(max(Data_holder::get()->velocity.v_liner_y, -(short(Data_holder::get()->parameter.max_v_liner_y))), short(Data_holder::get()->parameter.max_v_liner_y));
    short vz = min(max(Data_holder::get()->velocity.v_angular_z, -(short(Data_holder::get()->parameter.max_v_angular_z))), short(Data_holder::get()->parameter.max_v_angular_z));

    float vel[3]={vx/100.0, vy/100.0, vz/100.0};
    float motor_speed[MOTOR_COUNT]={0};
    model->motion_solver(vel, motor_speed);


    for(int i=0;i<MOTOR_COUNT;i++){
        input[i] = motor_speed[i]*short(Data_holder::get()->parameter.encoder_resolution)/(2*__PI)*short(Data_holder::get()->parameter.do_pid_interval)*0.001;
    }


#if DEBUG_ENABLE
    //printf("vx=%d %d motor_speed=%ld %ld\r\n", vx, vz, long(motor_speed[0]*1000), long(motor_speed[1]*1000));
#endif

    last_velocity_command_time = Board::get()->get_tick_count();
    do_kinmatics_flag = true;
}

void Robot::do_kinmatics(){
    if (!do_kinmatics_flag){
        for(int i=0;i<MOTOR_COUNT;i++){
            pid[i]->clear();
            encoder[i]->get_increment_count_for_dopid();
        }
        return;
    }
    
    static unsigned long last_millis=0;
    if (Board::get()->get_tick_count()-last_millis>=Data_holder::get()->parameter.do_pid_interval){
        last_millis = Board::get()->get_tick_count();
        
        for(int i=0;i<MOTOR_COUNT;i++){
            feedback[i] = encoder[i]->get_increment_count_for_dopid();
        }
#if PID_DEBUG_OUTPUT
    #if MOTOR_COUNT==2
        printf("input=%ld %ld feedback=%ld %ld\r\n", long(input[0]*1000), long(input[1]*1000), 
                                                        long(feedback[0]), long(feedback[1]));
    #endif
    #if MOTOR_COUNT==3
        printf("input=%ld %ld %ld feedback=%ld %ld %ld\r\n", long(input[0]*1000), long(input[1]*1000), long(input[2]*1000), 
                                                        long(feedback[0]), long(feedback[1]), long(feedback[2]));
    #endif
    #if MOTOR_COUNT==4
        printf("input=%ld %ld %ld %ld feedback=%ld %ld %ld %ld\r\n", long(input[0]*1000), long(input[1]*1000), long(input[2]*1000), long(input[3]*1000), 
                                                        long(feedback[0]), long(feedback[1]), long(feedback[2]), long(feedback[3]));
    #endif
#endif
        bool stoped=true;
        for(int i=0;i<MOTOR_COUNT;i++){
            if (input[i] != 0 || feedback[i] != 0){
                stoped = false;
                break;
            }
        }

        short output[MOTOR_COUNT]={0};
        if (stoped){
            for(int i=0;i<MOTOR_COUNT;i++){
                output[i] = 0;
            }
            do_kinmatics_flag = false;
        }else{
            for(int i=0;i<MOTOR_COUNT;i++){
                output[i] = pid[i]->compute(Data_holder::get()->parameter.do_pid_interval*0.001);
            }
        }

        for(int i=0;i<MOTOR_COUNT;i++){
            Data_holder::get()->pid_data.input[i] = int(input[i]);
            Data_holder::get()->pid_data.output[i] =  int(feedback[i]);
        }

#if PID_DEBUG_OUTPUT
    #if MOTOR_COUNT==2
        printf("output=%ld %ld\r\n\r\n", output[0], output[1]);
    #endif
    #if MOTOR_COUNT==3
        printf("output=%ld %ld %ld\r\n\r\n", output[0], output[1], output[2]);
    #endif
    #if MOTOR_COUNT==4
        printf("output=%ld %ld %ld %ld\r\n\r\n", output[0], output[1], output[2], output[3]);
    #endif
#endif
        for(int i=0;i<MOTOR_COUNT;i++){
            motor[i]->control(output[i]);
        }

        if (Board::get()->get_tick_count()-last_velocity_command_time>Data_holder::get()->parameter.cmd_last_time){
            for(int i=0;i<MOTOR_COUNT;i++){
                input[i] = 0;
            }
        }
    }
}

#define CALC_ODOM_INTERVAL 100

void Robot::calc_odom(){

    static unsigned long last_millis=0;

    if (Board::get()->get_tick_count()-last_millis>=CALC_ODOM_INTERVAL){
        last_millis = Board::get()->get_tick_count();
#if ODOM_DEBUG_OUTPUT        
        long total_count[MOTOR_COUNT]={0};
        for(int i=0;i<MOTOR_COUNT;i++){
            total_count[i] = encoder[i]->get_total_count();
        }

    #if MOTOR_COUNT==2
        printf("total_count=[%ld %ld]", total_count[0], total_count[1]);
    #endif
    #if MOTOR_COUNT==3
        printf("total_count=[%ld %ld %ld]", total_count[0], total_count[1], total_count[2]);
    #endif
    #if MOTOR_COUNT==4
        printf("total_count=[%ld %ld %ld %ld]", total_count[0], total_count[1], total_count[2], total_count[3]);
    #endif
#endif
        float dis[MOTOR_COUNT] = {0};
        for(int i=0;i<MOTOR_COUNT;i++){
            dis[i] = encoder[i]->get_increment_count_for_odom()*__PI*Data_holder::get()->parameter.wheel_diameter*0.001/Data_holder::get()->parameter.encoder_resolution;
#if ODOM_DEBUG_OUTPUT
            printf(" %ld ", long(dis[i]*1000000));
#endif
        }
        
        model->get_odom(&odom, dis, CALC_ODOM_INTERVAL);

#if ODOM_DEBUG_OUTPUT
        printf(" x=%ld y=%ld yaw=%ld", long(odom.x*1000), long(odom.y*1000), long(odom.z*1000));
        printf("\r\n");
#endif
        
        Data_holder::get()->odom.v_liner_x = odom.vel_x*100;
        Data_holder::get()->odom.v_liner_y = odom.vel_y*100;
        Data_holder::get()->odom.v_angular_z = odom.vel_z*100;
        Data_holder::get()->odom.x = odom.x*100;
        Data_holder::get()->odom.y = odom.y*100;
        Data_holder::get()->odom.yaw = odom.z*100;
    }
}

#define CALC_IMU_INTERVAL 100
void Robot::get_imu_data(){
#if IMU_ENABLE
    static unsigned long last_millis=0;

    if (Board::get()->get_tick_count()-last_millis>=CALC_IMU_INTERVAL){
        last_millis = Board::get()->get_tick_count();
        imu->get_data(Data_holder::get()->imu_data);
    }
#endif
}

#define CHECK_JOYSTICK_INTERVAL 100
void Robot::check_joystick(){
#if JOYSTICK_ENABLE
    static unsigned long last_millis=0;
    short liner_x=0, liner_y=0, angular_z=0;
    if (Board::get()->get_tick_count()-last_millis>=CHECK_JOYSTICK_INTERVAL){
        last_millis = Board::get()->get_tick_count();
        if (joystick->check(liner_x, liner_y, angular_z))
        {
            Data_holder::get()->velocity.v_liner_x = liner_x;
            Data_holder::get()->velocity.v_liner_y = liner_y;
            Data_holder::get()->velocity.v_angular_z = angular_z;
            update_velocity();
        }
    }
#endif
}

