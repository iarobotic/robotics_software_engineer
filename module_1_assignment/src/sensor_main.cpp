#include "sensor_lib.h"
#include <iostream>


int main() {
    
    TemperatureSensor<int> sensorInt;
    TemperatureSensor<float> sensorFloat;
    TemperatureSensor<std::string> sensorString;
   
    cout << "Temperatura (int): " << sensorInt.getTemperature() << endl;
    cout << "Temperatura (float): " << sensorFloat.getTemperature() << endl;
    cout << "Temperatura (string): " << sensorString.getTemperature() << endl;



    DistanceSensor<int> sensorIntD;
    DistanceSensor<float> sensorFloatD;
    DistanceSensor<std::string> sensorStringD;

    cout << "Distance (int): " << sensorIntD.getDistance() << endl;
    cout << "Distance (float): " << sensorFloatD.getDistance() << endl;
    cout << "Distance (string): " << sensorStringD.getDistance() << endl;


    return 0;
}