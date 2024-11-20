#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
using namespace std;

namespace mobile{
    class Robot {
    private:
        string name;
        double speed;

    public:
        Robot(string nameRobot, double speed);
        const double weight = 10;
        const double size = 10;
        const double number_of_sensors = 5;
        void moveForward();
        void moveBackward();
        void stop();
    };
};
namespace aerial{
    class Robot {
    private:
        string name;
        double speed;

    public:
        Robot(string nameRobot, double speed);
        const double weight = 5;
        const double size = 1;
        const double number_of_sensors = 5;
        void moveForward();
        void moveBackward();
        void stop();
    };
};
namespace arm{
    class Robot {
    private:
        string name;
        double speed;

    public:
        Robot(string nameRobot, double speed);
        const double weight = 30;
        const double size = 20;
        const double number_of_sensors = 2;
        void moveForward();
        void moveBackward();
        void stop();
    };
};


#endif 