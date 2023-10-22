#include "../include/libHCSR04/HCSR04_sensor.hpp"


HCSR04::HCSR04(int trigger, int echo) : trigger_(trigger), echo_(echo)
{
    pinMode(trigger_, OUTPUT);
    pinMode(echo_, INPUT);
    digitalWrite(trigger, LOW);
    usleep(500);
}

double HCSR04::distance()
{
    digitalWrite(trigger_, HIGH);
    usleep(trigger_on_us_);
    digitalWrite(trigger_, LOW);


    startTimeUsec = std::chrono::high_resolution_clock::now();
    endTimeUsec = std::chrono::high_resolution_clock::now();

    
    while(digitalRead(echo_) == LOW)
    {
        startTimeUsec = std::chrono::high_resolution_clock::now();
    }

    while(digitalRead(echo_) == HIGH)
    {
        endTimeUsec = std::chrono::high_resolution_clock::now();
    }

    travelTimeUsec = std::chrono::duration_cast<std::chrono::microseconds>(endTimeUsec - startTimeUsec);
    distanceMeters = (double)((travelTimeUsec.count()/1'000'000.0)*sonicSpeed_m_s_)/2;

    return distanceMeters;

}

double HCSR04::speed()
{
    auto distance1 = this->distance();
    usleep(1'000'000); // 1 sec sleep as time interval
    auto distance2 = this->distance();

    // divedere (1) must correspond with the time interval between the computed distance
    return (distance2-distance1)/1;
}
