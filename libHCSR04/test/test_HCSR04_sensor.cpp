#include <iostream>
#include "../include/libHCSR04/HCSR04_sensor.hpp"


using namespace std;

int trigger = 14;
int echo = 4;

int main()
{
    if (wiringPiSetupGpio() == -1)
        return -1;

    HCSR04 ultrasonic(trigger, echo);
    // ultrasonic.init(trigger, echo);

    while(1){
        cout << "-------------------" << endl;
        cout << "Distance is " << ultrasonic.distance()*100 << " cm" << endl;
        
        // Of course, due to low distance precision, speed is gonna be even less precise
        cout << "Speed is: " << ultrasonic.speed()*100 << " cm/s" << endl;
        usleep(500'000);
    }
}