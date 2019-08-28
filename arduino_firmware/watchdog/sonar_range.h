#ifndef SONAR_RANGE_H_
#define SONAR_RANGE_H_

/*
********************************************************************************
* Filename      : Serial Handler
* Author        : Susung Park
* Description   : Serial data handler for communication with Arduino
* Version       : Initial release; 07 Jul 2019
********************************************************************************

Sound wave travels approximately 340 meters per second, which becomes 0.34
millimeters per microseconds. Therefore, multiplying by 17 and dividing by 100
would yield the distance to target in units of millimeters (since the measured
time corresponds to that of round trip).
*/

class SonarRange
{
    public:
    SonarRange(int trigger, int echo, int timeout = 20000);

    public:
    void Measure();
    int GetMeasurement();

    private:
    const int trigger_pin_;
    const int echo_pin_;
    int measurement_;
    int timeout_;
};

SonarRange::SonarRange(int trigger, int echo, int timeout = 20000)
  : trigger_pin_(trigger), echo_pin_(echo), measurement_(0), timeout_(timeout)
{
    pinMode(trigger_pin_, OUTPUT);
    pinMode(echo_pin_, INPUT);
    digitalWrite(trigger_pin_, LOW);
}

// Measure distance and return it in units of millimiters.
void SonarRange::Measure()
{
    digitalWrite(trigger_pin_, LOW);
    delayMicroseconds(5);
    digitalWrite(trigger_pin_, HIGH);
    delayMicroseconds(2);
    digitalWrite(trigger_pin_, LOW);
    measurement_ = pulseIn(echo_pin_, HIGH, timeout_) * 17 / 100;
}

int SonarRange::GetMeasurement()
{
    return measurement_;
}

#endif
