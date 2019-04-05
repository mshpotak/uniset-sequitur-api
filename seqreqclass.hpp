#include <string>
#include <cstring>
#include <iostream>

#ifndef SEQREQCLASS_HPP_
#define SEQREQCLASS_HPP_

class Seq_Request{
    private:
        int code;
        string parameters;
    public:
        int get_code();
        const char* get_parameters();
        int get_size();
        void set(string user_parameters, int user_code);
        //parameter number from SEQUITUR manual
        void set_parameter(int parameter_number, string user_parameter);
};

#endif
