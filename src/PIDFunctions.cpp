#include "PIDFunctions.h"
using namespace vex;

PIDFunctions::PIDFunctions() : distancePID(0.185, 0.0031, 2.81) {}

// Method to drive straight
void PIDFunctions::driveStraight(double targetDistance, distanceUnits units,  vex::directionType direction) {
    // Reset the PID Controller
    resetSensors();
    distancePID.reset();
    double setPoint = convertDistanceToDegrees(targetDistance, units);
    double initialHeading = BrainInertial.heading();
    prevHeadingError = 0;
    

    while(true) {
        double averagePosiion = (LDMotor.position(degrees) + RDMotor.position(degrees)) / 2;
        double distanceOutput = distancePID.calculate(setPoint, averagePosiion);
        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("%f", convertDegreesToDistance(setPoint, vex::distanceUnits::in));

        double headingError = initialHeading - BrainInertial.rotation();
        Brain.Screen.setCursor(5,1);
        Brain.Screen.print("%f", headingError);
        double headingDerivative = headingError - prevHeadingError;
        double headingIntegral;
        headingIntegral += headingError;
        if(headingIntegral > 3) {
            headingIntegral = 3;
        }
        double headingOutput = (headingError * 1.671) + (headingDerivative * 14) + (headingIntegral * 0.00000000001);


        double leftMotorSpeed = distanceOutput - headingOutput;
        if(distancePID.inIntegralRange()) {
            leftMotorSpeed = distanceOutput - headingOutput + 3.48;
        }
        double rightMotorSpeed = distanceOutput + headingOutput;

        prevHeadingError = headingError;
        


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

        if(Drivetrain.velocity(pct) < 0.00001 && Drivetrain.velocity(pct) > -0.00001 && distancePID.atSetPoint() && headingError < 1.5 && headingError > -1.5) {
            Drivetrain.stop(vex::brakeType::brake);
            break;
        }
        

        wait(10, msec);
        
        
    }
    
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("Done !!!!");
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
            double distanceInDegrees = distance / (wheelCircumference * 360 * gearRatio);
            return distanceInDegrees;

        }
double PIDFunctions::convertDegreesToDistance(double degrees, vex::distanceUnits units) {
    double wheelDiameter = 2.5; // Example wheel diameter in inches
    double wheelCircumference = wheelDiameter * M_PI;
    double gearRatio = 2.5; // Example gear ratio

    double distanceInInches = (degrees / 360) * wheelCircumference * gearRatio;

    if (units == vex::distanceUnits::mm) {
        distanceInInches *= 25.4; // Convert inches to mm
    }

    return distanceInInches;
}
