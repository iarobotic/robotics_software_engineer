#include "robot_class.h"

class RobotSensors : public mobile::Robot {
    public:
        RobotSensors(string nameRobot, double speed) : mobile::Robot(nameRobot, speed) {}
        void displayTemperature(){
            cout << "Temperature: 20°C" << endl;
        };
        void displayDistance(){
            cout << "Distance: 1000 cm" << endl;
        };
};

int main() {
    RobotSensors mobileRobot("MobileRobot", 10);
    mobileRobot.moveForward();
    mobileRobot.moveBackward();
    mobileRobot.stop();
    mobileRobot.displayTemperature();
    mobileRobot.displayDistance();
    return 0;
}
