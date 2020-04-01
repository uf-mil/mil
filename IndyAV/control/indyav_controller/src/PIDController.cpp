#include "indyav_controller/PIDController.hpp"

/*  --Constructor--
    Tuning Parameters:
        kp_ = Proportional Constant [Equal to Controller Gain (Kc)]
        ki_ = Integral Constant [Equal to (Kc/Ti), where Ti is the Integral Time Constant]
        kd_ = Derivative Constant [Equal to (Kc*Td), where Td is the Derivative Time Constant]
    
    Other Parameters:
        min_/max_ = For specifying controller output range
        cycle_time = For specifying how often the calculate function will be called (Hz)
*/
PID::PID(double kp_, double ki_, double kd_, double min_, double max_, double cycle_time_)
{
    kd = kp_;
    ki = ki_;
    kd = kd_;
    min = min_;
    max = max_;
    cycle_time = 1/cycle_time_; //Convert Hz to seconds
}

//Destructor
PID::~PID(){}

/*  --PID Controller Calculation--
    Parameters:
        set_point = target value, ex: target velocity
        process_variable = current value, ex: current/actual velocity
*/
double PID::calculate(double set_point, double process_variable)
{
    //Find error, add to error sum
    double error = set_point - process_variable;
    error_sum += error * cycle_time;

    //Find P Term
    double P = kp * error;
    
    //Find I Term
    double I = ki * error_sum;

    //Find D Term
    double D = kd * ((process_variable - old_process_variable) / cycle_time);
    
    //Find output using PID equation
    double output = P + I - D;

    //Restrict output within specified range
    if(output > max)
        return max;
    else if (output < min)
        return min;

    return output;
}




