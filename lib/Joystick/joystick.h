#ifndef PIBOT_JOYSTICK_H_
#define PIBOT_JOYSTICK_H_

#include <PS2X_lib.h>  //for v1.6

class Joystick{
    public:
        Joystick();
        bool init();
        
        void test();
        bool check(short& liner_x, short liner_y, short& angular_z);
    private:
        bool holonomic_check(short& liner_x, short liner_y, short& angular_z);
        bool nonholonomic_check(short& liner_x, short liner_y, short& angular_z);
    private:
        int error;
	    PS2X ps2x; // create PS2 Controller Class
};

#endif