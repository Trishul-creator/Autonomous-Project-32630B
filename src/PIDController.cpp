#include "PIDController.h"

PIDController::PIDController(double kP, double kI, double kD)
    : kP(kP), kI(kI), kD(kD), error(0), integral(0), derivative(0), previousError(0), previousTime(0), maxOutput(80), minOutput(-80), maxIntegral(300){
        pidTimer.clear();
}

double PIDController::calculate(double setpoint, double measuredValue) {
    this->setpoint = setpoint;
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

    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("%f",(kD * derivative));

    // output to send to the motor
    double output = (kP * error) + (kI * integral) + (kD * derivative);
    Brain.Screen.setCursor(4,1);
    Brain.Screen.print("%f", output);

    if(output > maxOutput) {
        output = maxOutput;
    } else if(output < minOutput) {
        output = minOutput;
    }

    // set previous values to current values
    previousError = error;
    previousTime = currentTime;

    return output;

}

bool PIDController::inIntegralRange() {
    if(degreesToDistance(error, vex::distanceUnits::in) > 0.1* degreesToDistance(setpoint, vex::distanceUnits::in)) { 
        return true;
    } else {
        return false;
    }
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
    setpoint = 0;
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
    double wheelDiameter = 2.5; // Example wheel diameter in inches
    double wheelCircumference = wheelDiameter * M_PI;
    double gearRatio = 2.5; // Example gear ratio

    double distanceInInches = (degrees / 360.0) * wheelCircumference / gearRatio;

    if (units == vex::distanceUnits::mm) {
        distanceInInches *= 25.4; // Convert inches to mm
    }

    return distanceInInches;
}

