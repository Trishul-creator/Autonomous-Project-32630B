#pragma region Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fstream>
#include <string.h>
#include "RobotConfig.h"
#include "vex.h"
#include "PIDFunctions.h"


using namespace vex;

// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed); 
}

bool vexcode_initial_drivetrain_calibration_completed = false;
void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  vexcode_initial_drivetrain_calibration_completed = true;
  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}

void vexcodeInit() {

  // Calibrate the Drivetrain
  calibrateDrivetrain();

  // Initializing random seed.
  initializeRandomSeed(); 

  
}
#pragma endregion
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       gurumurthyvenkataraman                                    */
/*    Created:      1/4/2025, 2:50:18 PM                                      */
/*    Description:  IQ2 project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

int main() {
    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();

    PIDFunctions pid = PIDFunctions();

    pid.driveStraight(12, distanceUnits::in, forward);

    
  
    

    


    



   



    

}