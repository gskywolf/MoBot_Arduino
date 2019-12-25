#include "simple_dataframe_slave.h"
#include "data_holder.h"
#include <stdio.h>
#include "transport.h"

Simple_dataframe::Simple_dataframe(Transport* _trans): trans(_trans){
    recv_state = STATE_RECV_FIX;
}

bool Simple_dataframe::init(){
    return true;
}

bool Simple_dataframe::data_recv(unsigned char c){
    //printf("%02x", c);
    switch (recv_state){
    case STATE_RECV_FIX:
        if (c == FIX_HEAD){
            memset(&active_rx_msg,0, sizeof(active_rx_msg));
            active_rx_msg.head.flag = c;
            active_rx_msg.check += c;

            recv_state = STATE_RECV_ID;
        }
        break;
    case STATE_RECV_ID:
        if (c < ID_MESSGAE_MAX){
            active_rx_msg.head.msg_id = c;
            active_rx_msg.check += c;
            recv_state = STATE_RECV_LEN;
        }
        else
            recv_state = STATE_RECV_FIX;
        break;
    case STATE_RECV_LEN:
        active_rx_msg.head.length =c;
        active_rx_msg.check += c;
        if (active_rx_msg.head.length==0)
            recv_state = STATE_RECV_CHECK;
        else
            recv_state = STATE_RECV_DATA;
        break;
    case STATE_RECV_DATA:
        active_rx_msg.data[active_rx_msg.recv_count++] = c;
        active_rx_msg.check += c;
        if (active_rx_msg.recv_count >=active_rx_msg.head.length)
            recv_state  = STATE_RECV_CHECK;
        break;
    case STATE_RECV_CHECK:
        recv_state = STATE_RECV_FIX;
        if (active_rx_msg.check == c){
            return true;
        }
        break;
    default:
        recv_state = STATE_RECV_FIX;
    }

    return false;
}

bool Simple_dataframe::data_parse(){
    MESSAGE_ID id = (MESSAGE_ID)active_rx_msg.head.msg_id;

    //printf("\r\ndata_parse:id=%d\r\n", id);

    Data_holder* dh = Data_holder::get();
    switch (id){
    case ID_GET_VERSION:
        send_message(id, (unsigned char*)&dh->firmware_info, sizeof(dh->firmware_info));
        break;
    case ID_SET_ROBOT_PARAMTER:
        memcpy(&dh->parameter, active_rx_msg.data, sizeof(dh->parameter));
        send_message(id);
        break;
    case ID_GET_ROBOT_PARAMTER:
        send_message(id, (unsigned char*)&dh->parameter, sizeof(dh->parameter));
        break;
    case ID_CLEAR_ODOM:
        send_message(id);
        break;
    case ID_SET_VELOCITY:
        memcpy(&dh->velocity, active_rx_msg.data, sizeof(dh->velocity));
        send_message(id);
        break;
    case ID_GET_ODOM:
        send_message(id, (unsigned char*)&dh->odom, sizeof(dh->odom));
        break;
    case ID_GET_PID_DATA:
        send_message(id, (unsigned char*)&dh->pid_data, sizeof(dh->pid_data));
        break;
    case ID_GET_IMU_DATA:
        send_message(id, (unsigned char*)&dh->imu_data, sizeof(dh->imu_data));
        break;
    default:
        break;
    }

    if (id < ID_MESSGAE_MAX && nf[id] != 0)
        nf[id]->update(id, &dh);

    return true;
}

bool Simple_dataframe::send_message(const MESSAGE_ID id){
    Message msg(id);

    send_message(&msg);

    return true;
}

bool Simple_dataframe::send_message(const MESSAGE_ID id, unsigned char* data, unsigned char len){
    Message msg(id, data, len);

    send_message(&msg);

    return true;
}

bool Simple_dataframe::send_message(Message* msg){
    if (trans == 0)
        return true;
    
    trans->write((unsigned char*)msg, sizeof(msg->head)+msg->head.length+1);

    return true;
}