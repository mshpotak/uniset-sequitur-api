#include <string>
#include <cstring>
#include <iostream>

#ifndef SEQREQCLASS_HPP_
#define SEQREQCLASS_HPP_

class Seq_Request{
    private:
        int code;
        std::string parameters;
    public:
        Seq_Request(int user_code);
        int get_code();
        const char* get_parameters();
        int get_size();
        void set(std::string user_parameters, int user_code);
        //parameter number from SEQUITUR manual
        void set_parameter(int parameter_number, std::string user_parameter);
};

#endif
