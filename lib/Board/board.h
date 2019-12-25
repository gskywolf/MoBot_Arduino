#ifndef PIBOT_BOARD_H_
#define PIBOT_BOARD_H_

class Queue;

enum USART_NUMBER
{
    USART_0 = 0,
    USART_1,
    USART_2,
    USART_3,
    USART_4,
    USART_5,
    USART_6,
};

#define PIN_MODE_INPUT 0x0
#define PIN_MODE_OUTPUT 0x1
#define PIN_MODE_INPUT_PULLUP 0x2

class Board{
  public:
    virtual void enable_irq()=0;
    virtual void disable_irq()=0;
    virtual void usart_debug_init()=0;
    virtual void usart_init(unsigned char num, unsigned long buad)=0;
    virtual Queue* usart_getDataQueue(unsigned char num)=0;

    virtual void usart_write(unsigned char num, unsigned char ch)=0;
    virtual void usart_write(unsigned char num, unsigned char* data, unsigned char len)=0;

    virtual void set_config(unsigned char* data, unsigned short len)=0;
    virtual void get_config(unsigned char* data, unsigned short len)=0;

    virtual void pin_init(unsigned char pin, unsigned char mode)=0;
    virtual void pin_write(unsigned char pin, unsigned char level)=0;
    virtual unsigned char pin_read(unsigned char pin)=0;

    virtual void pwm_init(unsigned char khz)=0;
    virtual void pwm_output(unsigned char pin, unsigned short pwm)=0;

    virtual unsigned long get_tick_count()=0;

    static Board* get();
};

#endif