#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


class PIDController
{
private: 
    double _Kp,_Ki,_Kd;
    double _target_value;
    double _integral_max,_integral_min;
    double _output_max,_output_min;
    double _acceptable_error;
    double _integrated_error;
    double _frequency;
    double _output;
    
    double limitToRange(double value, double minimum, double maximum);
public:
   
    
    PIDController();
    ~PIDController();

    void setConstants(double Kp, double Ki, double Kd,double acceptable_error,double frequency);
    void setMinMaxLimits(double output_min, double output_max, double integral_min, double integral_max);
    void setTargetValue(double target_value);
    void updateOutput(double current_value,double rate_of_change);
    void updateOutput(double current_value,double rate_of_change,double time_difference);
    void updateOutput(double current_value,double rate_of_change,double time_difference,double target_value);
    double getOutput();
    void reset();

    
};



#endif