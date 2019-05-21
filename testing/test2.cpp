#include "sequitur-api.hpp"
#include <iostream>
#include <stdlib.h>
#include <string>
#include <cstring>

using namespace std;

int main(int argc, const char* argv[])
{
    Tag id0;
    Tag::ForwardData fun0( &id0 );
    fun0.fwd_state( true );
    while(1){
        fun0.recv_upd( true );
    }
    return 0;
}
