#include "encoder_implement.h"


EncoderImp::EncoderImp(unsigned char pinA, unsigned char pinB):encoder(pinA, pinB){
    clear();
}

void EncoderImp::init(){
    clear();
}

void EncoderImp::clear(){
    encoder.write(0);
    pid_pos = odom_pos = 0;
}

long EncoderImp::get_total_count(){
    return encoder.read();
}

long EncoderImp::get_increment_count_for_dopid(){
    long l = encoder.read()-pid_pos;
    pid_pos = encoder.read();
    return l;
}

long EncoderImp::get_increment_count_for_odom(){
    long l = encoder.read()-odom_pos;
    odom_pos = encoder.read();
    return l;
}