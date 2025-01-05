#include "PIDFunctions.h"
using namespace vex;

PIDFunctions::PIDFunctions() : distancePID(2, 0.0, 0.0) {}

// Method to drive straight
void PIDFunctions::driveStraight(double targetDistance, distanceUnits units,  vex::directionType direction) {
    // Reset the PID Controller
    resetSensors();
    distancePID.reset();
    double setPoint = convertDistanceToDegrees(targetDistance, units);
    double initialHeading = BrainInertial.heading();

    while(true) {
        double averagePosiion = (LDMotor.position(degrees) + RDMotor.position(degrees)) / 2;
        double distanceOutput = distancePID.calculate(setPoint, averagePosiion);
        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("%f", convertDegreesToDistance(setPoint, vex::distanceUnits::in));

        double headingError = initialHeading - BrainInertial.rotation();
        double headingOutput = headingError * 0.0;

        double leftMotorSpeed = distanceOutput + headingOutput;
        double rightMotorSpeed = distanceOutput - headingOutput;

        if(leftMotorSpeed > 0 && direction == forward) {
            LDMotor.spin(forward, leftMotorSpeed, percent);
        } else if(leftMotorSpeed < 0 && direction == forward) {
            LDMotor.spin(reverse, -leftMotorSpeed, percent);
        } else if(leftMotorSpeed > 0 && direction == reverse) {
            LDMotor.spin(reverse, leftMotorSpeed, percent);
        } else if(leftMotorSpeed < 0 && direction == reverse) {
            LDMotor.spin(forward, -leftMotorSpeed, percent);
        }

         if(rightMotorSpeed > 0 && direction == forward) {
            RDMotor.spin(forward, rightMotorSpeed, percent);
        } else if(rightMotorSpeed < 0 && direction == forward) {
            RDMotor.spin(reverse, -rightMotorSpeed, percent);
        } else if(rightMotorSpeed > 0 && direction == reverse) {
            RDMotor.spin(reverse, rightMotorSpeed, percent);
        } else if(rightMotorSpeed < 0 && direction == reverse) {
            RDMotor.spin(forward, -rightMotorSpeed, percent);
        }
        

        wait(10, msec);
        
        
    }
    Drivetrain.stop(vex::brakeType::brake);
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("Done !!!!");
}

void PIDFunctions::resetSensors() {
    LDMotor.resetPosition();
    RDMotor.resetPosition();
    BrainInertial.resetHeading();
    BrainInertial.resetRotation();
}
PIDController PIDFunctions::getPIDController() {
    return distancePID;
}
double PIDFunctions::convertDistanceToDegrees(double distance, vex::distanceUnits units) {
            double wheelDiameter = 2.5;
            double wheelCircumference = wheelDiameter * M_PI;
            double gearRatio = 2.5;
            
            if(units == vex::distanceUnits::mm) {
                distance /= 25.4;
            }
            double distanceInDegrees = (360 / wheelCircumference) * distance * gearRatio;
            return distanceInDegrees;

        }
double PIDFunctions::convertDegreesToDistance(double degrees, vex::distanceUnits units) {
    double wheelDiameter = 2.5; // Example wheel diameter in inches
    double wheelCircumference = wheelDiameter * M_PI;
    double gearRatio = 2.5; // Example gear ratio

    double distanceInInches = (degrees / 360.0) * wheelCircumference / gearRatio;

    if (units == vex::distanceUnits::mm) {
        distanceInInches *= 25.4; // Convert inches to mm
    }

    return distanceInInches;
}
