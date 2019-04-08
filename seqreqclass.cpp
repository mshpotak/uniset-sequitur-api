#include "seqreqclass.hpp"


Seq_Request::Seq_Request(int user_code){
    switch(user_code){
        case 0:
            set("0", CLIENT_PING);
            break;
        case 3:
            set("positionforwardenabled 1 1 0", CLIENT_SET_PARAMETER);
            break;
        case 14:
            set("0", CLIENT_GET_TAG_POSITION);
            break;
        default:
            std::cout << "Invalid request code" << std::endl;
    }
}

Seq_Request::Seq_Request(int user_code, string user_parameters){
    set(user_parameters, user_code);
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
