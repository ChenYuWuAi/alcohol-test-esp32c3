#include "pid.h"

PID::PID(float Kp, float Ki, float Kd, float Pmax, float Imax, float Dmax, float max, float sum_err_max)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Imax = Imax;
    this->Dmax = Dmax;
    this->outputMax = max;
    this->SumErrMax = sum_err_max;

    error = 0;
    error_last = 0;
    Pout = 0;
    Iout = 0;
    Dout = 0;
    output = 0;
}

float PID::get_output(float feedback_input, float target_input)
{
    target = target_input;
    error = target - feedback_input;

    if (error > SumErrMax)
        error = SumErrMax;
    else if (error < -SumErrMax)
        error = -SumErrMax;

    Pout = Kp * error;
    Iout += Ki * error;
    Dout = Kd * (error - error_last);

    if (Iout > Imax)
        Iout = Imax;
    else if (Iout < -Imax)
        Iout = -Imax;

    if (Dout > Dmax)
        Dout = Dmax;
    else if (Dout < -Dmax)
        Dout = -Dmax;

    output = Pout + Iout + Dout;

    if (output > outputMax)
        output = outputMax;
    else if (output < -outputMax)
        output = -outputMax;

    error_last = error;

    return output;
}

void PID::reset()
{
    error = 0;
    error_last = 0;
    Pout = 0;
    Iout = 0;
    Dout = 0;
    target = 0;
    output = 0;
}

void PID::reset(float Kp, float Ki, float Kd, float Imax, float Dmax, float max, float sum_err_max)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Imax = Imax;
    this->Dmax = Dmax;
    this->outputMax = max;
    this->SumErrMax = sum_err_max;

    reset();
}