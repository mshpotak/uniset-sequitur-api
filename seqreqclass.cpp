#include "seqreqclass.hpp"

class Seq_Request{
    request(int user_code){
        code = user_code
        switch(code){
            case 0:     set("0", 0);
                        break;
            case 3:     set("dataforwardenabled 1 0 0", 3);
                        break;
            case 14:    set("0", 14);
                        break;
            default:    std::cout << "Invalid request code" << std::endl;
        }
    }
    int get_code(){
        return code;
    }
    const char* get_parameters(){
        return parameters.c_str();
    }
    int get_size(){
        return parameters.size();
    }
    void set(string user_parameters, int user_code){
        parameters = user_parameters;
        code = user_code;
    }
    //parameter number from SEQUITUR manual
    void set_parameter(int parameter_number, string user_parameter){
        size_t pos1, pos2 = 0, found = 0;
        for(int i = 1; i < parameter_number); i++, pos1++){
            found = parameters.find(" ", found+1);
            pos1 = pos2;
            pos2 = found;
        }
        parameters.replace(pos1, pos2-pos1, user_parameter);
    }
};
