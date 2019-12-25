#ifndef PIBOT_BOARD_MEGA2560_H_
#define PIBOT_BOARD_MEGA2560_H_

#include "board.h"
#include "variable_queue.h"

class Board_Mega2560 : public Board{
  public:
    Board_Mega2560();
    ~Board_Mega2560();

    void enable_irq();
    void disable_irq();
    void usart_debug_init();
    void usart_write(unsigned char num, unsigned char ch);
    void usart_init(unsigned char num, unsigned long buad);
    Queue* usart_getDataQueue(unsigned char num);

    void usart_write(unsigned char num, unsigned char* data, unsigned char len);

    void set_config(unsigned char* data, unsigned short len);
    void get_config(unsigned char* data, unsigned short len);

    void pin_init(unsigned char pin, unsigned char mode);
    void pin_write(unsigned char pin, unsigned char level);
    unsigned char pin_read(unsigned char pin);

    void pwm_init(unsigned char khz);
    void pwm_output(unsigned char pin, unsigned short pwm);

    unsigned long get_tick_count();
  private:
  public:
    static Board_Mega2560 board;

    VQueue<256> usart1_queue;
};

#endif