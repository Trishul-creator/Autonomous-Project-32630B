#ifndef PIDFUNCTIONS_H
#define PIDFUNCTIONS_H

#include "vex.h"
#include "PIDController.h"
#include "RobotConfig.h"

class PIDFunctions { 
    public:
        PIDFunctions();
        // Method to drive straight
        void driveStraight(double targetDistance, vex::distanceUnits units, vex::directionType direction);
        double convertDistanceToDegrees(double distance, vex::distanceUnits units);
        void resetSensors();    
        double convertDegreesToDistance(double distance, vex::distanceUnits units);
        PIDController getPIDController();
        void printErrorLogs();

    private:
        PIDController distancePID;
        double prevHeadingError;
};

#endif // PIDFUNCTIONS_H
