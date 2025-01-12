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

        bool inIntegralRange();




        double degreesToDistance(double degrees, vex::distanceUnits units);

        void reset();

        const std::vector<std::pair<double, double>>& getErrorLog() const;

    private:
        // PID Drive Constants
        double kP;
        double kI;
        double kD;

        // PID Turn Constants
        double turnkP;
        double turnkD;
        double turnkI;

        // PID Drive  Variables
        double error;
        double integral;
        double derivative;
        double previousError;
        double previousTime;

        // PID Turn Variables
        double turnError;
        double turnIntegral;
        double turnDerivative;
        double turnPreviousError;
        double turnPreviousTime;

        //Setpoint
        double setpoint;

        //Turn setpoint
        double turnSetPoint;

        //Limits
        double maxOutput;
        double minOutput;
        double maxIntegral;

        

        // PID Drive  Timer
        vex::timer pidTimer;

        // PID Turn Timeer;
        vex::timer turnTimer;


        //Oscillation vector

        std::vector<std::pair<double,double>> errorLog;
        
        
        
};

#endif // PIDCONTROLLER_H