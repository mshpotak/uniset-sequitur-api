#include "seq-req-class-v2.hpp"

//Network class definition
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

void Sequitur::send_req(){
    compose_msg();
    send_msg( msg.c_str() );
};
void Sequitur::recv_resp(){
    await();
    recv_msg();
    decompose_msg();
};

void Sequitur::req_once(){
    send_req();
    recv_resp();
}

void Sequitur::set_req(int user_code, std::string user_parameters ){
    req_code = user_code;
    req_parameters = user_parameters;
}

//Forward class definition

Forward::Forward(){
    set_req(3, "positionforwardenabled 1 1 0");
    req_once();
}

Forward::~Forward(){
    set_req(3, "positionforwardenabled 0 1 0");
    req_once();
}

void Forward::recv_upd(){
    int skip;
    recv_resp();
    std::stringstream ss;
    ss << msg;
    ss >> skip >> skip >> skip >> pose.timestamp >>
    pose.position.x >> pose.position.y >> pose.position.z >>
    pose.posconf.x >> pose.posconf.y >> pose.posconf.z >>
    skip >>
    pose.accel.x >> pose.accel.y >> pose.accel.z >>
    pose.gyro.x >> pose.gyro.y >> pose.gyro.z >>
    pose.mag.x >> pose.mag.y >> pose.mag.z;
}
