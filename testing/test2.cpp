#include "sequitur-api.hpp"
#include <iostream>
#include <stdlib.h>
#include <string>
#include <cstring>

using namespace std;

<<<<<<< HEAD
//char buffer[1024];

int main(int argc, const char* argv[])
{
    char str[] = "{-1 59 2 148235353571.1 15.88 73.39 1.04 0.34 0.10 0.25 148235353571.1 0.028 0.013 0.976 5.329 -1.199 -0.604 -0.200 -0.119 -0.440 1020.78 3 }";
    sscanf(str,"{%*d %*d %*d %[^_}]", str);
    string buff = str;
    cout << buff << endl;

=======
int main(int argc, const char* argv[])
{
    Tag id0;
    Tag::ForwardData fun0( &id0 );
    fun0.fwd_state( true );
    while(1){
        fun0.recv_upd( true );
    }
>>>>>>> 9c87d3a1626395d62451eadc73e1054630850a8e
    return 0;
}
