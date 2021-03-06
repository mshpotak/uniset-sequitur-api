#include "sequitur-api.hpp"

//Sequitur class definition

Sequitur::Sequitur(){
    hashcode = -1;
    req_code = -1;
    set_network();
}

Sequitur::~Sequitur(){
    close(sock_fd);
};

void Sequitur::set_network(){
    std::string ip;
    std::cout << "Enter the IP address:\n";
    std::cin >> ip;
    result = connect_to( ip.c_str(), PORT_SEQ );
    if( result != 0 ) exit(-1);
}

void Sequitur::compose_msg(){
    hashcode = rand() % 9000 + 1000;
    std::stringstream ss;
    ss << '{' << hashcode << ' ' << req_code << ' ' << req_parameters << '}' << '\0';
    msg.clear();
    msg = ss.str();
}

void Sequitur::decompose_msg(){
    msg.erase( msg.begin() + msg.find('{') );
    msg.erase( msg.begin() + msg.find('}') );
}

void Sequitur::req_once(){
    compose_msg();
    if( send_msg( msg.c_str() ) != 0 ){
        req_once();
        return;
    }
    if( await() != 0 ){
        req_once();
        return;
    }
    if( recv_msg() != 0 ){
        req_once();
        return;
    }
    decompose_msg();
}

void Sequitur::set_req( int user_code, std::string user_parameters ){
    req_code = user_code;
    req_parameters = user_parameters;
}

void Sequitur::print_msg(){
    printf( "%s\n", msg.c_str() );
}

//Forward class definition

Tag::ForwardData::ForwardData( Sequitur *owner ){
    seq = owner;
}

Tag::ForwardData::~ForwardData(){
    seq->set_req( CLIENT_SET_PARAMETER, "positionforwardenabled 0 1 0" );
    seq->req_once();
}

void Tag::ForwardData::fwd_state( bool fwd ){
    if( fwd == true ){
        seq->set_req( CLIENT_SET_PARAMETER, "positionforwardenabled 1 1 0" );
        seq->req_once();
    } else {
        seq->set_req( CLIENT_SET_PARAMETER, "positionforwardenabled 0 1 0" );
        seq->req_once();
    }
}

void Tag::ForwardData::recv_upd( bool print ){
    seq->await(0);
    seq->recv_msg();
    if( print == true ){
        seq->print_msg();
    }
    decompose_msg();
}

void Tag::ForwardData::decompose_msg(){
    seq->decompose_msg();
    std::stringstream ss;
    ss << seq->msg;
    ss >> seq->result >> seq->result >> seq->result >> pose.timestamp >>
    pose.position.x >> pose.position.y >> pose.position.z >>
    pose.posconf.x >> pose.posconf.y >> pose.posconf.z >>
    seq->result >>
    pose.accel.x >> pose.accel.y >> pose.accel.z >>
    pose.gyro.x >> pose.gyro.y >> pose.gyro.z >>
    pose.mag.x >> pose.mag.y >> pose.mag.z;
}

//Set boundaries class definition

Tag::SetAnchorLocation::SetAnchorLocation( Sequitur *owner ){
    seq = owner;
    parameters[0] = "1 1 10205F1310000DE1 2.439 -2.231 0.695";
    parameters[1] = "2 1 10205F1310001423 2.631 2.544 2.145";
    parameters[2] = "3 1 10205F1310001422 -1.856 -2.247 2.095";
    parameters[3] = "4 1 10205F1310001425 -1.548 2.541 0.705";
}

void Tag::SetAnchorLocation::decompose_msg(){
    std::stringstream ss;
    ss << seq->msg;
    ss >> seq->result >> seq->result;
}

void Tag::SetAnchorLocation::set_default_loc(){
    for( int i = 0; i < 4; i++ ){
        seq->set_req( CLIENT_SET_ANCHOR_INFO, parameters[i] );
        seq->req_once();
        decompose_msg();
        if( seq->result != 0 ){
            i--;
        } else {
            std::cout << "Board #" << i+1 << " is set.\n";
        }
    }
}

//Scan class definition

Sequitur::Scan::Scan( Sequitur *owner ){
    seq = owner;
}

void Sequitur::Scan::decompose_msg(){
    std::stringstream ss;
    ss << seq->msg;
    ss >> seq->result >> seq->result >> range;
}

double Sequitur::Scan::get_range( std::string node_id ){
    seq->set_req( CLIENT_GET_RANGE, node_id );
    seq->req_once();
    decompose_msg();
    if( seq->result == 255 ) return -1;
    return range;
}
