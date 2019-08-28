/*
********************************************************************************
* Filename      : Watchdog Firmware
* Author        : Susung Park
* Description   : Watchdog firmware that monitors 7 sonar range sensors.
* Version       : On development...
********************************************************************************
*/

#include "serial_handler.h"
#include "sonar_range.h"

// Serial buffers.
char inboundSerialBuffer[10];
char outboundSerialBuffer[16];
SerialHandler serial(inboundSerialBuffer, outboundSerialBuffer);

// Initialize SonarRange objects.
SonarRange sonar_01(30, 31);
SonarRange sonar_02(32, 33);
SonarRange sonar_03(34, 35);
SonarRange sonar_04(36, 37);
SonarRange sonar_05(38, 39);
SonarRange sonar_06(40, 41);
SonarRange sonar_07(42, 43);

void bindBytes(char *start_pos, int val)
{
    char *pos = start_pos;
    *pos = char(val & 0x00FF);
    ++pos;
    *pos = char((val & 0xFF00) >> 1);
}

void measureCallback()
{
    sonar_01.Measure();
    sonar_02.Measure();
    sonar_03.Measure();
    sonar_04.Measure();
    sonar_05.Measure();
    sonar_06.Measure();
    sonar_07.Measure();

    Serial.print(sonar_01.GetMeasurement()); Serial.print(" / ");
    Serial.print(sonar_02.GetMeasurement()); Serial.print(" / ");
    Serial.print(sonar_03.GetMeasurement()); Serial.print(" / ");
    Serial.print(sonar_04.GetMeasurement()); Serial.print(" / ");
    Serial.print(sonar_05.GetMeasurement()); Serial.print(" / ");
    Serial.print(sonar_06.GetMeasurement()); Serial.print(" / ");
    Serial.print(sonar_07.GetMeasurement()); Serial.println();

//    bindBytes(outboundSerialBuffer +  0, sonar_01.GetMeasurement());
//    bindBytes(outboundSerialBuffer +  2, sonar_02.GetMeasurement());
//    bindBytes(outboundSerialBuffer +  4, sonar_03.GetMeasurement());
//    bindBytes(outboundSerialBuffer +  6, sonar_04.GetMeasurement());
//    bindBytes(outboundSerialBuffer +  8, sonar_05.GetMeasurement());
//    bindBytes(outboundSerialBuffer + 10, sonar_06.GetMeasurement());
//    bindBytes(outboundSerialBuffer + 12, sonar_07.GetMeasurement());
//    outboundSerialBuffer[14] = '\0';
//    outboundSerialBuffer[15] = '\0';
//
//    Serial.println(outboundSerialBuffer);
}

void setup()
{
    Serial.begin(57600);
}

void loop()
{
    serial.Receive();
    if (serial.IsDataReady())
    {
        if (inboundSerialBuffer[0] == 'a')
            for (int i = 0; i < 100; i++)
                measureCallback();
        serial.ConsumeData();
    }
}
