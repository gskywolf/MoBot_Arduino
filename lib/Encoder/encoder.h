#ifndef PIBOT_ENCODER_H_
#define PIBOT_ENCODER_H_

class Encoder{
public:
    virtual void init()=0;
    virtual void clear()=0;
    virtual long get_total_count()=0;

    virtual long get_increment_count_for_dopid()=0;  
    virtual long get_increment_count_for_odom()=0;  
};

#endif