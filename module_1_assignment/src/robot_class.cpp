#include "robot_class.h"

namespace mobile{
    Robot::Robot(string nameRobot, double speed){
        name = nameRobot;
        this->speed = speed;
    }
    void Robot::moveForward(){
        cout << "Robot mobile " << name << " is moving forward" << endl;
    }

    void Robot::moveBackward(){
        cout << "Robot mobile " << name << " is moving backward" << endl;
    }

    void Robot::stop(){
        cout << "Robot mobile " << name << " is stopped" << endl;
    }
};

namespace aerial{
    Robot::Robot(string nameRobot, double speed){
        name = nameRobot;
        this->speed = speed;
    }

    void Robot::moveForward(){
        cout << "Robot aerial " << name << " is moving forward" << endl;
    }

    void Robot::moveBackward(){
        cout << "Robot aerial " << name << " is moving backward" << endl;
    }

    void Robot::stop(){
        cout << "Robot aerial " << name << " is stopped" << endl;
    }
};

namespace arm{
    Robot::Robot(string nameRobot, double speed){
        name = nameRobot;
        speed = speed;
    }

    void Robot::moveForward(){
        cout << "Robot arm " << name << " is moving forward" << endl;
    }

    void Robot::moveBackward(){
        cout << "Robot arm " << name << " is moving backward" << endl;
    }

    void Robot::stop(){
        cout << "Robot arm " << name << " is stopped" << endl;
    }
};