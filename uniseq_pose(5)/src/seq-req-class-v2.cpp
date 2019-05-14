#include "seq-req-class-v2.hpp"

//Network class definition
Network::Network(){
    std::string ip;
    std::cout << "Enter the IP address:\n";
    std::cin >> ip;
    connect_to( ip.c_str(), PORT_SEQ );
    if( result != 0 ) exit(-1);
}

Network::~Network(){
    close(sock_fd);
}

int Network::connect_to( const char ip[], const char port[], int family, int socktype, int flags ){
    struct addrinfo hints;
    struct addrinfo *servinfo;

    memset( &hints, 0, sizeof( hints ) );

    hints.ai_family = family;
    hints.ai_socktype = socktype;
    hints.ai_flags = flags;

    result = getaddrinfo( ip, port, &hints, &servinfo );
    if( result != 0 ){
        fprintf( stderr, "getaddrinfo: %s\n", gai_strerror( result ) );
        return -1;
    }

    sock_fd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol);
    if( sock_fd == -1 ){
        perror( "socket error" );
        return -1;
    }

    result = connect(sock_fd, servinfo->ai_addr, servinfo->ai_addrlen);
    if( result == -1 ){
        perror( "connect error" );
        return -1;
    }
    freeaddrinfo( servinfo );
    return 0;
}

int Network::await( int timeout_ms ){
    struct pollfd sfd;
    sfd.fd = sock_fd;
    sfd.events = POLLIN;

    result = poll( &sfd, 1, timeout_ms );
    if( result == -1 ){
        perror( "poll error" );
        return -1;
    }
    if( result == 0 ){
        return 1;
    }
    if( result > 0 ){
        if( sfd.revents & POLLERR ){
            perror( "poll error" );
            return -1;
        }
        if( sfd.revents & POLLIN ){
            return 0;
        }
    }
    return -1;
}

int Network::send_msg( const char* msg ){
	result = send( sock_fd, msg, strlen( msg ), 0 );
	if( result == -1 ){
		perror("send error");
		return -1;
	}
    return 0;
}

int Network::recv_msg(){
    memset( buffer, '\0', BUFF_SIZE );
    result = recv( sock_fd, buffer, BUFF_SIZE, 0 );
    if( result == -1 ){
        perror("receive error:");
        return -1;
    }
    return 0;
}

//Sequitur class definition

Sequitur::Sequitur(): tag(this), anchor(this){}

Sequitur::~Sequitur(){};

void Sequitur::compose_msg(){
    hashcode = rand() % 9000 + 1000;
    std::stringstream ss;
    ss << '{' << hashcode << ' ' << req_code << ' ' << req_parameters << '}' << '\0';
    msg.clear();
    msg = ss.str();
}

void Sequitur::decompose_msg(){
    msg.clear();
    msg = buffer;
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

//Forward class definition

Sequitur::ForwardData::ForwardData( Sequitur *owner ){
    seq = owner;
    seq->set_req( CLIENT_SET_PARAMETER, "positionforwardenabled 1 1 0" );
    seq->req_once();
    seq->set_req( CLIENT_POSITION_FORWARD );
}

Sequitur::ForwardData::~ForwardData(){
    seq->set_req( CLIENT_SET_PARAMETER, "positionforwardenabled 0 1 0" );
    seq->req_once();
}

void Sequitur::ForwardData::recv_upd(){
    seq->await(0);
    seq->recv_msg();
    seq->decompose_msg();
}

void Sequitur::ForwardData::decompose_msg(){
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

Sequitur::SetAnchorLocation::SetAnchorLocation( Sequitur *owner ){
    seq = owner;
    parameters[0] = "1 1 10205F1310000DE1 2.439 -2.231 0.695";
    parameters[1] = "2 1 10205F1310001423 2.631 2.544 2.145";
    parameters[2] = "3 1 10205F1310001422 -1.856 -2.247 2.095";
    parameters[3] = "4 1 10205F1310001425 -1.548 2.541 0.705";
}

void Sequitur::SetAnchorLocation::decompose_msg(){
    seq->decompose_msg();
    std::stringstream ss;
    ss << seq->msg;
    ss >> seq->result >> seq->result;
}

void Sequitur::SetAnchorLocation::set_loc(){
    for( int i = 0; i < 4; i++ ){
        seq->set_req( CLIENT_SET_ANCHOR_INFO, parameters[i] );
        seq->req_once();
        if( seq->result != 0 ){
            i--;
        } else {
            std::cout << "Board #" << i+1 << " is set.\n";
        }
    }
}
