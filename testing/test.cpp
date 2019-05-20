
#include "seq-req-class-v2.hpp"
#include <math.h>
#include <chrono>
#include <ctime>

int main( int argc, char *argv[]){
    printf("Start...\n");
    Sequitur seq;
    std::chrono::system_clock::time_point start, end;
    std::chrono::duration<double> t_cycle;
    unsigned int i = 1;
    double t_sum = 0, range_avg = 0;
    while(1){
        start = std::chrono::system_clock::now();
        seq.scan.get_range("10205F1310001425");
        end = std::chrono::system_clock::now();
        t_cycle = end-start;
        range_avg += seq.scan.range;
        t_sum += t_cycle.count();
        printf("Range: %f, Range avg: %f, \tTime: %f,\t Frequency: %f\n", seq.scan.range, range_avg/i, t_cycle.count(), 1/(t_sum/i) );
        i++;
    }
    return 0;
}
