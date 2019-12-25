#ifndef PIBOT_MODLE_H_
#define PIBOT_MODLE_H_


struct Odom{
    float x;
    float y;
    float z;
    float vel_x;
    float vel_y;
    float vel_z;
};

struct Model{
    Model(){}
    Model(float _wheel_radius, float _body_radius): wheel_radius(_wheel_radius), body_radius(_body_radius){}

    void set(float _wheel_radius, float _body_radius){
        wheel_radius = _wheel_radius;
        body_radius = _body_radius;
    }

    ~Model(){}

    //robot speed ------------> motor speed
    virtual void motion_solver(float* robot_speed, float* motor_speed) = 0;

    //motor speed-------------> robot speed
    //interval  ms
    virtual void get_odom(struct Odom* odom, float* motor_dis, unsigned long interval) = 0;

protected:
    float wheel_radius;
    float body_radius;
};

#endif