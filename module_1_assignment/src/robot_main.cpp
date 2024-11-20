#include "robot_class.h"
#include<iostream>



int main() {
    mobile::Robot mobileRobot("MobileRobot", 10);
    aerial::Robot aerialRobot("AerialRobot", 5);
    arm::Robot armRobot("ArmRobot", 30);

    mobileRobot.moveForward();
    mobileRobot.moveBackward();
    mobileRobot.stop();

    aerialRobot.moveForward();
    aerialRobot.moveBackward();
    aerialRobot.stop();

    armRobot.moveForward();
    armRobot.moveBackward();
    armRobot.stop();

    return 0;
}