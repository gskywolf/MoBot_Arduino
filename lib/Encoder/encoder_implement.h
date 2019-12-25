#ifndef PIBOT_ENCODER_IMP_H_
#define PIBOT_ENCODER_IMP_H_

#include "encoder.h"
#include "arduino_encoder.h"

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS

class EncoderImp:public Encoder{
    public:
	    EncoderImp(unsigned char pinA, unsigned char pinB);
        void init();
        void clear();
        long get_total_count();
        long get_increment_count_for_dopid();  
        long get_increment_count_for_odom();  
    private:
        ArduinoEncoder encoder;
        long pid_pos, odom_pos;
};

#endif