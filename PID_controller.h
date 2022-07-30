#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
class PIDController
{
private: 
    float Kp_,Ki_,Kd_;
    float current_value_,target_value_;
    float integral_max_,integral_min_;
    float output_max_,output_min_;
    float error_,acceptable_error_;
    float p_,i_,d_;
    float output_;
    
    float limitToRange(float value, float minimum, float maximum);
public:
   
    
    PIDController(/* args */);
    ~PIDController();

    void setConstants(float Kp, float Ki, float Kd,float acceptable_error);
    void setMinMaxLimits(float output_min, float output_max, float integral_min, float integral_max);
    void setCurrentValue(float current_value);
    void setTargetValue(float target_value);

    float updateOutput();
    float updateOutput(float current_value);
    float updateOutput(float current_value,float target_value);

    void reset();

    
};



#endif