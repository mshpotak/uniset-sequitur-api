#include <string>
#include <cstring>
#include <iostream>

#ifndef SEQREQCLASSV2_HPP_
#define SEQREQCLASSV2_HPP_

#define CLIENT_PING 0
#define CLIENT_SET_PARAMETER 3
#define CLIENT_GET_SENSORS 51
#define CLIENT_GET_RANGE 50
#define CLIENT_SCAN 56
#define CLIENT_SEND_DATA 57
#define CLIENT_GET_ANCHOR_INFO 10
#define CLIENT_SET_ANCHOR_INFO 11
#define CLIENT_GET_TAG_POSITION 14
#define CLIENT_POSITION_FORWARD 59

class Seq_Request{
    private:
        int code;
        std::string parameters;
        int hashcode;
        const char* send_msg(int req_hashcode, int req_code, std::string req_parameters);
        std::string recv_msg(const char* msg);
    public:
        Seq_Request(int user_code);
        void send_request();
        void recv_response();
        void rec_response();
        int check_hashcode();

};

#endif
