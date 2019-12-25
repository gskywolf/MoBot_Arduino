#include "usart_transport.h"

#include "board.h"
#include <stdio.h>

#include "queue.h"

USART_transport::USART_transport(unsigned char num, unsigned long buad):usart_num(num), usart_buad(buad){

}

bool USART_transport::init()
{
    //printf("port=%d %ld\r\n", usart_num, usart_buad);
    Board::get()->usart_init(usart_num, usart_buad);
    return true;
}

bool USART_transport::read(unsigned char& ch){
    return Board::get()->usart_getDataQueue(usart_num)->get(ch);
}

void USART_transport::write(unsigned char* data, unsigned char len){
    Board::get()->usart_write(usart_num, data, len);
}