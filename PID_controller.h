#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


class PIDController
{
private: 
    float _Kp,_Ki,_Kd;
    float _target_value;
    float _integral_max,_integral_min;
    float _output_max,_output_min;
    float _acceptable_error;
    float _integrated_error;
    
    float limitToRange(float value, float minimum, float maximum);
public:
   
    
    PIDController();
    ~PIDController();

    void setConstants(float Kp, float Ki, float Kd,float acceptable_error);
    void setMinMaxLimits(float output_min, float output_max, float integral_min, float integral_max);
    void setTargetValue(float target_value);

    float updateOutput(float current_value,float rate_of_change,float time_difference);
    float updateOutput(float current_value,float rate_of_change,float time_difference,float target_value);
    void reset();

    
};



#endif