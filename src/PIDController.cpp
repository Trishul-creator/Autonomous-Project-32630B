#include "PIDController.h"

PIDController::PIDController(double kP, double kI, double kD)
    : kP(kP), kI(kI), kD(kD), error(0), integral(0), derivative(0), previousError(0), previousTime(0), maxOutput(100), minOutput(-100), maxIntegral(300){
        pidTimer.clear();
}

double PIDController::calculate(double setpoint, double measuredValue) {
    double currentTime = pidTimer.time();
    double deltaTime = currentTime - previousTime; // Change in time
    previousTime = currentTime;

    error = setpoint - measuredValue; // Distance between where we want to be and where we are !
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("%f", degreesToDistance(error, vex::distanceUnits::in));
    integral += error * deltaTime; // Accumulated error over time;
    
    if(integral > maxIntegral) {
        integral = maxIntegral;
    } 

    derivative = (error - previousError) / deltaTime; // Rate of change of error over time

    // output to send to the motor
    double output = (kP * error) + (kI * integral) + (kD * derivative);

    if(output > maxOutput) {
        output = maxOutput;
    } else if(output < minOutput) {
        output = minOutput;
    }

    // set previous values to current values
    errorLog.push_back({currentTime, error});
    previousError = error;
    previousTime = currentTime;

    return output;

}

const std::vector<std::pair<double, double>>& PIDController::getErrorLog() const {
    return errorLog;
}

void PIDController::reset() {
    error = 0;
    integral = 0;
    derivative = 0;
    previousError = 0;
    previousTime = 0;
    pidTimer.clear();
}

bool PIDController::atSetPoint() {
    if(degreesToDistance(error, vex::distanceUnits::in)  < 0.2 && degreesToDistance(error, vex::distanceUnits::in) > -0.2) {
        return true;
    } else {
        return false;
    }
}

double PIDController::degreesToDistance(double degrees, vex::distanceUnits units) {
    double wheelDiameter = 4.0; // Example wheel diameter in inches
    double wheelCircumference = wheelDiameter * M_PI;
    double gearRatio = 1.0; // Example gear ratio

    double distanceInInches = (degrees / 360.0) * wheelCircumference / gearRatio;

    if (units == vex::distanceUnits::mm) {
        distanceInInches *= 25.4; // Convert inches to mm
    }

    return distanceInInches;
}

