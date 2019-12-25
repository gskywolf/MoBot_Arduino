#ifndef PIBOT_USART_TRANSPORT_H_
#define PIBOT_USART_TRANSPORT_H_

#include "transport.h"

class USART_transport:public Transport{
  public:
    USART_transport(unsigned char num, unsigned long buad);
    bool init();
    bool read(unsigned char& ch);
    void write(unsigned char* data, unsigned char len);
  private:
    unsigned char usart_num;
    unsigned long usart_buad;
};
#endif