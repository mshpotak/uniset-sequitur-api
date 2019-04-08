#include "seqreqclass.hpp"


Seq_Request::Seq_Request(int user_code){
    code = user_code;
    switch(code){
        case CLIENT_PING:
            set("0", 0);
            break;
        case CLIENT_SET_PARAMETER:
            set("dataforwardenabled 1 1 0", 3);
            break;
        case CLIENT_GET_TAG_POSITION:
            set("0", 14);
            break;
        default:
            std::cout << "Invalid request code" << std::endl;
    }
}

int Seq_Request::get_code(){
    return code;
}

const char* Seq_Request::get_parameters(){
    return parameters.c_str();
}

int Seq_Request::get_size(){
    return parameters.size();
}

void Seq_Request::set(std::string user_parameters, int user_code){
    parameters = user_parameters;
    code = user_code;
}

//single parameter change
//parameter number from SEQUITUR manual
void Seq_Request::set_parameter(int parameter_number, std::string user_parameter){
    size_t pos1, pos2 = 0, found = 0;
    for(int i = 1; i < parameter_number; i++, pos1++){
        found = parameters.find(" ", found+1);
        pos1 = pos2;
        pos2 = found;
    }
    parameters.replace(pos1, pos2-pos1, user_parameter);
}

//set all parameters
void Seq_Request::set_parameters(std::string user_parameters){
    parameters.clear();
    parameters = user_parameters;
}
