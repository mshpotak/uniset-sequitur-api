#include <string>
#include <cstring>
#include <iostream>

#ifndef SEQREQCLASS_HPP_
#define SEQREQCLASS_HPP_

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
    public:
        Seq_Request(int user_code);
        Seq_Request(int user_code, std::string user_parameters);
        int get_code();
        const char* get_parameters();
        int get_size();
        void set(std::string user_parameters, int user_code);
        //parameter number from SEQUITUR manual
        void set_parameter(int parameter_number, std::string user_parameter);
        void set_parameters(std::string user_parameters);
};

#endif
