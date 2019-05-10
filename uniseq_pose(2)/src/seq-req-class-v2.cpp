#include "seq-req-class-v2.hpp"


bool xyz::operator ==( const struct xyz& xyz ){
    if( x == xyz.x &&
        y == xyz.y &&
        z == xyz.z){
            return true;
    } else  return false;
}

bool imu::operator ==( const struct imu& imu ){
    if( accel == imu.accel &&
        gyro == imu.gyro &&
        mag == imu.mag){
            return true;
    } else  return false;
}

bool pose::operator ==( const struct pose& pose ){
    if( position == pose.position ){
            return true;
    } else  return false;
}

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

Sequitur::Sequitur(){
    std::string ip;
    std::cout << "Enter the IP address:\n";
    std::cin >> ip;
    connect_to( ip.c_str(), PORT_SEQ );
}

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
    recv_resp();
}

void Forward::decompose_msg(){
    Sequitur::decompose_msg();
    std::stringstream ss;
    ss << msg;
    ss >> result >> result >> result >> pose.timestamp >>
    pose.position.x >> pose.position.y >> pose.position.z >>
    pose.posconf.x >> pose.posconf.y >> pose.posconf.z >>
    result >>
    pose.accel.x >> pose.accel.y >> pose.accel.z >>
    pose.gyro.x >> pose.gyro.y >> pose.gyro.z >>
    pose.mag.x >> pose.mag.y >> pose.mag.z;
}

GetPose::GetPose(){
    set_req(3, "positionforwardenabled 0 1 0");
    req_once();
}

void GetPose::get_upd(){
    bool pose_b = false, imu_b = false;
    do{
        std::cout << "1" << std::endl;
        if( pose_b != true ){
            set_req( CLIENT_GET_TAG_POSITION, "0" );
            req_once();
            if( pose_valid == OUTOFDATE_POSITION ){
                pose_b = true;
            }
            std::cout << "11" << std::endl;
            if( !(pose == pose_old) ){
                std::cout << "111" << std::endl;
                pose_old = pose;
                pose_b = true;
            }
        }
        std::cout << "2" << std::endl;
        if( imu_b != true ){
            set_req( CLIENT_GET_SENSORS );
            req_once();
            std::cout << "22" << std::endl;
            if( !(imu == imu_old) ){
                std::cout << "222" << std::endl;
                imu_old = imu;
                imu_b = true;
            }
        }
        std::cout << "3" << std::endl;
    } while ( !(pose_b && imu_b) );
    pose = pose_old;
    imu = imu_old;
}

void GetPose::decompose_msg(){
    Sequitur::decompose_msg();
    if( req_code == CLIENT_GET_TAG_POSITION ){
        std::stringstream ss;
        ss << msg;
        ss >> result >> pose_valid >> pose.timestamp >>
        pose.position.x >> pose.position.y >> pose.position.z >>
        pose.conf.x >> pose.conf.y >> pose.conf.z;
    }
    if( req_code == CLIENT_GET_SENSORS ){
        std::stringstream ss;
        ss << msg;
        ss >> result >> result >> imu.timestamp >>
        imu.accel.x >> imu.accel.y >> imu.accel.z >>
        imu.gyro.x >> imu.gyro.y >> imu.gyro.z >>
        imu.mag.x >> imu.mag.y >> imu.mag.z;
    }
}
