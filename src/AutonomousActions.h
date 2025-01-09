#ifndef AUTONOMOUSACTIONS_H
#define AUTONOMOUSACTIONS_H

#include "vex.h"
#include "RobotConfig.h"
#include "PIDFunctions.h"


class AutonomousActions {
    public :
        AutonomousActions();
        void setUp();
        void rightGoalInitialShoot();
        void leftGoalInitialShoot();
        void shootLoop(int numberOfTimes);
        void pointCalcuator(bool switchOrNot);
    private:
        PIDFunctions pid;

};

#endif // AUTONOMOUSACTIONS_H




