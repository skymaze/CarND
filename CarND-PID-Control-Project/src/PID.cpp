#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    Kp = Kp;
    Ki = Ki;
    Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    pre_cte = 0.0;
    sum_err = 0.0;
    n = 0;
}

void PID::UpdateError(double cte) {
    p_error = cte;
    i_error += cte;
    d_error = cte - pre_cte;
    pre_cte = cte;
    sum_err += cte * cte;
    n++;
}

double PID::TotalError() {
    return Kp * p_error + Ki * i_error + Kd * d_error;
}

double PID::SquaredError() {
    return sum_err / (double)n;
}

void PID::Reset() {
    sum_err = 0.0;
    n = 0;
}

int PID::Iteration() {
  return n;
}

void PID::UpdateKp(double delta) {
    Kp += delta;
}

void PID::UpdateKi(double delta) {
    Ki += delta;
}

void PID::UpdateKd(double delta) {
    Kd += delta;
}

double PID::GetKp() {
    return Kp;
}

double PID::GetKi() {
    return Ki;
}

double PID::GetKd() {
    return Kd;
}