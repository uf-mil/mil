#pragma once

class PID
{
public:
    
    //Constructor
	PID(double kp_, double ki_, double kd_, double min_, double max_, double cycle_time_);
    
    //Destructor
    ~PID();

    //Performs PID Calculation
	double calculate(double set_point, double process_variable);

private:
 
	double error_sum; 
    double old_process_variable;
    double cycle_time; 
    double min;
    double max;
    double kp; 
    double ki;
    double kd;
};