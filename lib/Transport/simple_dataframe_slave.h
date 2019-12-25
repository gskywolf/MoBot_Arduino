#ifndef PIBOT_SIMPLE_DATAFRAME_SLAVE_H_
#define PIBOT_SIMPLE_DATAFRAME_SLAVE_H_

#include "simple_dataframe.h"

class Transport;
class Simple_dataframe : public Dataframe{
    public:
        Simple_dataframe(Transport* trans=0);
        
        void register_notify(const MESSAGE_ID id, Notify* _nf){
            if (id >= ID_MESSGAE_MAX)
                return;
            nf[id] = _nf;
        }

        bool init();
        bool data_recv(unsigned char c);
        bool data_parse();
        bool interact(const MESSAGE_ID id){return true;};
    private:
        bool send_message(const MESSAGE_ID id);
        bool send_message(const MESSAGE_ID id, unsigned char* data, unsigned char len);
        bool send_message(Message* msg);
    private:
        Notify* nf[ID_MESSGAE_MAX]; 

        Message active_rx_msg;

        RECEIVE_STATE recv_state;
		Transport* trans; 
};
#endif