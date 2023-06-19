#include "PID_controller.h"

PIDController::PIDController(){
    _Kp = _Ki = _Kd= 0;
    _integral_min = _output_min = -1000;
    _integral_max = _output_max = 1000;
    _acceptable_error = 0.1;
    _frequency = 1;
    reset();
}

PIDController::~PIDController(){

}

void PIDController::setConstants(float Kp, float Ki, float Kd, float acceptable_error,float frequency){

    _Kp               = Kp;
    _Ki              = Ki;
    _Kd             = Kd;
    _acceptable_error = acceptable_error;
    _frequency = frequency;
    reset();
    

}

void PIDController::setMinMaxLimits(float output_min, float output_max, float integral_min, float integral_max){

    _output_min  = output_min;
    _output_max  = output_max;
    _integral_min = integral_min;
    _integral_max = integral_max;
    reset();
}


void PIDController::setTargetValue(float target_value){

    _target_value = target_value;
}

float PIDController::updateOutput(float current_value,float rate_of_change){
    updateOutput(current_value,rate_of_change,1/_frequency);
}

float PIDController::updateOutput(float current_value,float rate_of_change,float time_difference){

    float error, p, i, d, output;    
    error = _target_value - current_value;
    
    if ( ( error >= 0 ) &&    ( error <= _acceptable_error  ) )
    {
        error = 0;
    }
    else if (( error < 0 ) &&    ( error >= _acceptable_error  ))
    {
        error = 0;
    }
    
    p = _Kp * error;

    i += _Ki * error* time_difference;

    i = limitToRange(i,_integral_min,_integral_max);
    _integrated_error += i;
    d = _Kd * rate_of_change;
    
    output = p + i + d;

    output = limitToRange(output,_output_min,_output_max);
    return output;
}

float PIDController::updateOutput(float current_value,float rate_of_change,float time_difference,float target_value){
    setTargetValue(target_value);
    return updateOutput(current_value,rate_of_change,time_difference);
}


void PIDController::reset(){
    _integrated_error = 0;
}

float PIDController::limitToRange(float value, float minimum, float maximum){
    if (value > maximum)
    {
        return maximum;
    }
    else if (value < minimum)
    {
        return minimum;
    }
    else
    {
        return value;
    }
    
    
    
}