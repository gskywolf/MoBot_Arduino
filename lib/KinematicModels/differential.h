#ifndef PIBOT_DIFFERENTIAL_H_
#define PIBOT_DIFFERENTIAL_H_

#include "model.h"
#include "math.h"

#define MOTOR_COUNT 2

class Differential :	public Model{
public:
	Differential() {}
	Differential(float _wheel_radius, float _body_radius) : Model(_wheel_radius, _body_radius) {}
	
	void motion_solver(float* robot_speed, float* motor_speed){
		//	robot_speed[0] x	robot_speed[1] y	robot_speed[2] z
		motor_speed[0] = (-robot_speed[0] + robot_speed[2] * body_radius)/ wheel_radius;
		motor_speed[1] = (robot_speed[0] + robot_speed[2] * body_radius) / wheel_radius;
    }
		
	void get_odom(struct Odom* odom, float* motor_dis, unsigned long interval)
	{
		float dxy_ave = (-motor_dis[0] + motor_dis[1]) / 2.0;
		float dth = (motor_dis[0] + motor_dis[1]) / (2* body_radius);
		float vxy = 1000 * dxy_ave / interval;
		float vth = 1000 * dth / interval;

		odom->vel_x = vxy;
		odom->vel_y = 0;
		odom->vel_z = vth;
		float dx = 0, dy = 0;
		if (motor_dis[0] != motor_dis[1])
		{
			dx = cos(dth) * dxy_ave;
			dy = -sin(dth) * dxy_ave;
			odom->x += (cos(odom->z) * dx - sin(odom->z) * dy);
			odom->y += (sin(odom->z) * dx + cos(odom->z) * dy);;
		}

		if (motor_dis[0] + motor_dis[1] != 0)
			odom->z += dth;
	}
};

#endif
