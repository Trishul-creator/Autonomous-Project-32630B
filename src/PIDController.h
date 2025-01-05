#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "vex.h"
#include "RobotConfig.h"
#include <vector>

class PIDController {
    public:
        // Constructor
        PIDController(double kP, double kI, double kD);

        
        // Method to calculate the PID output 
        double calculate(double setpoint, double measuredValue);

        // Calculate when pid should stop
        bool atSetPoint();


        double degreesToDistance(double degrees, vex::distanceUnits units);

        void reset();

        const std::vector<std::pair<double, double>>& getErrorLog() const;

    private:
        // PID Constants
        double kP;
        double kI;
        double kD;

        // PID Variables
        double error;
        double integral;
        double derivative;
        double previousError;
        double previousTime;

        //Limits
        double maxOutput;
        double minOutput;
        double maxIntegral;

        // PID Timer
        vex::timer pidTimer;

        //Oscillation vector

        std::vector<std::pair<double,double>> errorLog;
        
        
        
};

#endif // PIDCONTROLLER_H