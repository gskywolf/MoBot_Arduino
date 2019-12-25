#ifndef PIBOT_TRANSPORT_H_
#define PIBOT_TRANSPORT_H_

class Transport{
    public:
        virtual bool init()=0;
        virtual bool read(unsigned char& ch)=0;
        virtual void write(unsigned char* data, unsigned char len) = 0;

};
#endif