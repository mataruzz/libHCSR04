#ifndef HCSR04_SENSOR_HPP_
#define HCSR04_SENSOR_HPP_

#include <iostream>
#include <wiringPi.h>
#include <chrono>
#include <unistd.h>

class HCSR04
{
public:
    HCSR04(int trigger, int echo);

    double distance();
    double speed();

private:
    int trigger_;
    int echo_;
    double distanceMeters;
    std::chrono::high_resolution_clock::time_point startTimeUsec;
    std::chrono::high_resolution_clock::time_point endTimeUsec;
    std::chrono::microseconds travelTimeUsec;

    double sonicSpeed_m_s_{343.0};
    int trigger_on_us_{10};
};

#endif // HCSR04_SENSOR_HPP