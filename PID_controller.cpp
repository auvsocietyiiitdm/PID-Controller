#include "PID_Controller.h"
#include <limits>


PIDController::PIDController(){
    Kp_ = Ki_ = Kd_ = 0;
    p_  = i_  = d_  = 0;
    integral_min_ = output_min_ = std::numeric_limits<float>::min();
    integral_max_ = output_max_ = std::numeric_limits<float>::max();
}

PIDController::~PIDController(){

}

void PIDController::setConstants(float Kp, float Ki, float Kd, float acceptable_error){

    Kp_               = Kp;
    Ki_               = Ki;
    Kd_               = Kd;
    acceptable_error_ = acceptable_error;

}

void PIDController::setMinMaxLimits(float output_min, float output_max, float integral_min, float integral_max){

    output_min_   = output_min;
    output_max_   = output_max;
    integral_min_ = integral_min;
    integral_max_ = integral_max;
}

void PIDController::setCurrentValue(float current_value){

    current_value_ = current_value;
}

void PIDController::setTargetValue(float target_value){

    target_value_ = target_value;
}

float PIDController::updateOutput(){
    
    error_ = target_value_ - current_value_;
    
    if ( ( error_ >= 0 ) &&    ( error_ <= acceptable_error_  ) )
    {
        error_ = 0;
    }
    else if (( error_ < 0 ) &&    ( error_ >= acceptable_error_  ))
    {
        error_ = 0;
    }
    

    
    p_ = Kp_ * 

}