#include "joystick.h"

#include "board.h"
#include <stdio.h>

#define pressures   false
#define rumble      false

Joystick::Joystick(){
}

bool Joystick::init(){

	unsigned long start = Board::get()->get_tick_count();
	while(Board::get()->get_tick_count()-start>500){
		;
	}
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
	if (error == 0)
	{
	#if JOYSTICK_DEBUG_ENABLE
		printf("Teleop start\r\n");
	#endif
        return true;
	}

#if JOYSTICK_DEBUG_ENABLE
	if (error == 1)
		printf("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips\r\n");
	else if (error == 2)
		printf("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips\r\n");
	else if (error == 3)
		printf("Controller refusing to enter Pressures mode, may not support it. \r\n");
	else
		printf("Teleop err\r\n");
#endif
    
    return false;
}

#define JOYSTICK_TEST
void Joystick::test(){
#ifdef JOYSTICK_TEST
	byte vibrate = 0;
	if (ps2x.read_gamepad(false, vibrate)) //read controller and set large motor to spin at 'vibrate' speed
	{
		//printf("read_gamepad successed\r\n");
		//delay(100);
	}
	else
	{
		printf("read_gamepad failed\r\n");
		return;
	}
	
    if (ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
		printf("Start is being held\r\n");
	if (ps2x.Button(PSB_SELECT))
		printf("Select is being held\r\n");

	if (ps2x.Button(PSB_PAD_UP))               //will be TRUE if button was JUST pressed
	{
      	printf("Up held this hard: ");
      	printf("%d\r\n", ps2x.Analog(PSAB_PAD_UP));
	}
    if(ps2x.Button(PSB_PAD_RIGHT)){
		printf("Right held this hard: ");
		printf("%d\r\n", ps2x.Analog(PSAB_PAD_RIGHT));
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
      	printf("LEFT held this hard: ");
		printf("%d\r\n", ps2x.Analog(PSB_PAD_LEFT));
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
      	printf("DOWN held this hard: ");
		printf("%d\r\n", ps2x.Analog(PSAB_PAD_DOWN));
    }

	if (ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
		printf("Circle just pressed\r\n");
    if(ps2x.ButtonPressed(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
      	printf("X just changed\r\n");
    if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
      	printf("Square just released\r\n");   

	if (ps2x.Button(PSB_L1))
	{
		printf("L1 pressed\r\n");
	}

	if (ps2x.Button(PSB_L2))
	{
		printf("L2 pressed\r\n");
	}

	if (ps2x.Button(PSB_TRIANGLE)) {      //will be TRUE as long as button is pressed
		printf("TRIANGLE\r\n");
	}
	if (ps2x.Button(PSB_SQUARE)) {
		printf("SQUARE\r\n");
	}
	if (ps2x.Button(PSB_CIRCLE)) {
		printf("CIRCL\r\n");
	}
	if (ps2x.Button(PSB_CROSS)) {
		printf("CROSS\r\n");
	}

	vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
	if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
		if (ps2x.Button(PSB_L1))
			printf("L1 pressed\r\n");
		if (ps2x.Button(PSB_R1))
			printf("R1 pressed\r\n");
		if (ps2x.Button(PSB_L2))
			printf("L2 pressed\r\n");
		if (ps2x.Button(PSB_R2))
			printf("R2 pressed\r\n");
		if (ps2x.Button(PSB_TRIANGLE))
			printf("Triangle pressed\r\n");
	}
	
	if (ps2x.Button(PSB_L1)) { //print stick values if either is TRUE
		printf("Stick Values:");
		printf("%d", ps2x.Analog(PSS_LY));
		printf(",");
		printf("%d", ps2x.Analog(PSS_LX));
		printf(",");
		printf("%d", ps2x.Analog(PSS_RY));
		printf(",");
		printf("%d\r\n", ps2x.Analog(PSS_RX));
	}
#endif
}


bool Joystick::check(short& liner_x, short liner_y, short& angular_z){
	#if JOYSTICK_FOR_HOLONOMIC
		return holonomic_check(liner_x, liner_y, angular_z);
	#else
		return nonholonomic_check(liner_x, liner_y, angular_z);
	#endif
}

bool Joystick::holonomic_check(short& liner_x, short liner_y, short& angular_z){

	bool rtn = false;
	byte vibrate = 0;
	if (ps2x.read_gamepad(false, vibrate)) //read controller and set large motor to spin at 'vibrate' speed
	{
		//printf("read_gamepad successed\r\n");
		//delay(100);
	}
	else
	{
	#if JOYSTICK_DEBUG_ENABLE
		printf("read_gamepad failed\r\n");
	#endif
		return;
	}
	
	//up down left right  for liner x, y
	if (ps2x.Button(PSB_PAD_UP))
	{
	#if JOYSTICK_DEBUG_ENABLE
      	printf("UP");
	#endif
		liner_x = JOYSTICK_MAX_LINER_X;
		rtn = true;
	}
    if(ps2x.Button(PSB_PAD_DOWN)){
	#if JOYSTICK_DEBUG_ENABLE
      	printf("DOWN");
	#endif
		liner_x = -JOYSTICK_MAX_LINER_X;
		rtn = true;
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
	#if JOYSTICK_DEBUG_ENABLE
		printf("RIGHT");
	#endif
		liner_y = JOYSTICK_MAX_LINER_Y;
		rtn = true;
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
	#if JOYSTICK_DEBUG_ENABLE
      	printf("LEFT");
	#endif
		liner_y = -JOYSTICK_MAX_LINER_Y;
		rtn = true;
    }
	
	//triangle square circle cross  for angular z
	if (ps2x.Button(PSB_SQUARE))
	{
	#if JOYSTICK_DEBUG_ENABLE
      	printf("SQUARE");
	#endif
		angular_z = JOYSTICK_MAX_ANGULAR_Z;
		rtn = true;
	}
    if(ps2x.Button(PSB_CIRCLE)){
	#if JOYSTICK_DEBUG_ENABLE
      	printf("CIRCLE");
	#endif
		angular_z = -JOYSTICK_MAX_ANGULAR_Z;
		rtn = true;
    }

	if (ps2x.Button(PSB_L1)) { //print stick values if either is TRUE
	#if JOYSTICK_DEBUG_ENABLE
		printf("Stick Values:");
		printf("%d", ps2x.Analog(PSS_LY));
		printf(",");
		printf("%d", ps2x.Analog(PSS_LX));
		printf(",");
		printf("%d", ps2x.Analog(PSS_RY));
		printf(",");
		printf("%d\r\n", ps2x.Analog(PSS_RX));
	#endif
		liner_x = ((255.0/2)-ps2x.Analog(PSS_LY))/(255.0/2)*JOYSTICK_MAX_LINER_X;
		liner_y = ((255.0/2)-ps2x.Analog(PSS_LX))/(255.0/2)*JOYSTICK_MAX_LINER_Y;
		angular_z = ((255.0/2)-ps2x.Analog(PSS_RX))/(255.0/2)*JOYSTICK_MAX_ANGULAR_Z;
		rtn = true;
	}

	return rtn;
}

bool Joystick::nonholonomic_check(short& liner_x, short liner_y, short& angular_z){
	
	liner_y = 0;
	
	bool rtn = false;
	byte vibrate = 0;
	if (ps2x.read_gamepad(false, vibrate)) //read controller and set large motor to spin at 'vibrate' speed
	{
		//printf("read_gamepad successed\r\n");
		//delay(100);
	}
	else
	{
	#if JOYSTICK_DEBUG_ENABLE
		printf("read_gamepad failed\r\n");
	#endif
		return;
	}
	
	if (ps2x.Button(PSB_PAD_UP))               //will be TRUE if button was JUST pressed
	{
	#if JOYSTICK_DEBUG_ENABLE
      	printf("Up held this hard: ");
	#endif
		liner_x = JOYSTICK_MAX_LINER_X;
		rtn = true;
	}
    if(ps2x.Button(PSB_PAD_LEFT)){
	#if JOYSTICK_DEBUG_ENABLE
      	printf("LEFT held this hard: ");
	#endif
		angular_z = JOYSTICK_MAX_ANGULAR_Z;
		rtn = true;
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
	#if JOYSTICK_DEBUG_ENABLE
		printf("Right held this hard: ");
	#endif
		angular_z = -JOYSTICK_MAX_ANGULAR_Z;
		rtn = true;
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
	#if JOYSTICK_DEBUG_ENABLE
      	printf("DOWN held this hard: ");
	#endif
		liner_x = -JOYSTICK_MAX_LINER_X;
		rtn = true;
    }

	if (ps2x.Button(PSB_L1)) { //print stick values if either is TRUE
	#if JOYSTICK_DEBUG_ENABLE
		printf("Stick Values:");
		printf("%d", ps2x.Analog(PSS_LY));
		printf(",");
		printf("%d", ps2x.Analog(PSS_LX));
		printf(",");
		printf("%d", ps2x.Analog(PSS_RY));
		printf(",");
		printf("%d\r\n", ps2x.Analog(PSS_RX));
	#endif
		liner_x = ((255.0/2)-ps2x.Analog(PSS_LY))/(255.0/2)*JOYSTICK_MAX_LINER_X;
		angular_z = ((255.0/2)-ps2x.Analog(PSS_LX))/(255.0/2)*JOYSTICK_MAX_ANGULAR_Z;
		rtn = true;
	}

	return rtn;
}