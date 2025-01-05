#include "RobotConfig.h"

vex::brain Brain;
vex::inertial BrainInertial = vex::inertial();
vex::motor LDMotor = vex::motor(vex::PORT9, 2.5, false);
vex::motor RDMotor = vex::motor(vex::PORT3, 2.5, true);

vex::smartdrive Drivetrain = vex::smartdrive(LDMotor, RDMotor, BrainInertial, 200);;
vex::motor BackRoller = vex::motor(vex::PORT1, true);
vex::motor IntakeRollerA = vex::motor(vex::PORT6, true);
vex::motor IntakeRollerB = vex::motor(vex::PORT4, false);
vex::motor_group IntakeRollers = vex::motor_group(IntakeRollerA, IntakeRollerB);
vex::motor Launcher = vex::motor(vex::PORT7, true);


