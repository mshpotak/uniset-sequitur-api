#include "unix-network.hpp"

//Network class definition
Network::Network(){
    sock_fd = -1;
    result = 0;
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
    char buffer[BUFF_SIZE];
    memset( buffer, '\0', BUFF_SIZE );
    result = recv( sock_fd, buffer, BUFF_SIZE, 0 );
    msg.clear();
    msg = buffer;
    if( result == -1 ){
        perror("receive error:");
        return -1;
    }
    return 0;
}
