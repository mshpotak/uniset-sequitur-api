#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <cstring>

using namespace std;

int main()
{
    int argn = 4;
    string x = "14 0 1 1";
    string y = "0";
    size_t pos1, pos2 = 0, found = 0;

    // cout << x << "\t" << x.size() << endl;
    // if(argn == 1){
    //     found = x.find(" ");
    //     pos1 = 0;
    //     pos2 = found;
    //     x.replace(pos1, pos2, y);
    // }
    cout << x << "\t" << x.size() << endl;


    for(int i = 0; i < argn; i++, pos1++){
        found = x.find(" ", found+1);
        pos1 = pos2;
        pos2 = found;
    }

    x.replace(pos1, pos2-pos1, y);
    cout << x << "\t" << x.size() << endl;

    return 0;
}
