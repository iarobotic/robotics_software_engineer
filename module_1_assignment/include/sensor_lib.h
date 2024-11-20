#ifndef TEMPERATURE_SENSOR_HPP
#define TEMPERATURE_SENSOR_HPP

#include <string>
#include <sstream>
#include <iomanip>

using namespace std;

template <typename T>
class TemperatureSensor {
private:
    const float rawTemperature = 23.45f;

    // Helper method to convert float to string with precision
    std::string floatToString(float value, int precision = 2) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }

public:
    // Método para obtener la temperatura en el tipo especificado
    T getTemperature() {
        if constexpr (std::is_same_v<T, int>) {
            return static_cast<int>(rawTemperature);
        }
        else if constexpr (std::is_same_v<T, float>) {
            return rawTemperature;
        }
        else if constexpr (std::is_same_v<T, std::string>) {
            return floatToString(rawTemperature) + "°C";
        }
    }
};

template <typename T>
class DistanceSensor {
private:
    const float rawDistance = 125.10f;

    std::string floatToString(float value, int precision = 2) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }

public:
    T getDistance() {
        if constexpr (std::is_same_v<T, int>) {
            return static_cast<int>(rawDistance);
        }
        else if constexpr (std::is_same_v<T, float>) {
            return rawDistance;
        }
        else if constexpr (std::is_same_v<T, std::string>) {
            return floatToString(rawDistance) + "Mts.";
        }
    }
};


#endif // TEMPERATURE_SENSOR_HPP