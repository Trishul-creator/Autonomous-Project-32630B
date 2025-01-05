#include "PIDFunctions.h"
using namespace vex;

PIDFunctions::PIDFunctions() : distancePID(0.5, 0.0, 0.0) {}

// Method to drive straight
void PIDFunctions::driveStraight(double targetDistance, distanceUnits units, double maxSpeed) {
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

        LDMotor.spin(forward, leftMotorSpeed, percent);
        RDMotor.spin(forward, rightMotorSpeed, percent);

        if(Drivetrain.velocity(pct) < 5 && distancePID.atSetPoint()) {
            break;
        } 
        wait(10, msec);
        
        
    }
}

void PIDFunctions::resetSensors() {
    LDMotor.resetPosition();
    RDMotor.resetPosition();
    BrainInertial.resetHeading();
    BrainInertial.resetRotation();
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
