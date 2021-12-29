#include "PID.h"
#include <ctime>
#include <chrono>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    
    start = std::chrono::system_clock::now();
}

void PID::UpdateError(double cte) {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double,std::milli> elapsed_seconds = end - start;
    
    std::cout << "Time Elapsed " << elapsed_seconds.count()/1000 << endl;

    double prev_d;
    double prev_i;
    d_error = (cte - p_error)/(elapsed_seconds.count()/1000);
    p_error = cte;
    i_error = cte*elapsed_seconds.count()/1000 + i_error;
    
    start = std::chrono::system_clock::now();
}

double PID::TotalError() {
    
    return Kp*p_error + Kd*d_error + Ki*i_error;
    
}

