#include "serial_handler.h"
#include <TimerOne.h>

byte bitMask;
char inboundSerialBuffer[5];
char outboundSerialBuffer[3];
SerialHandler serial(inboundSerialBuffer, outboundSerialBuffer);

const long indicatorBlinkInterval = 400;

const int leftIndicatorPin = 11;
const int rightIndicatorPin = 12;
const int frontLightPin = 13;
const int rearLightPin = 10;

volatile bool leftIndicatorState = false;
volatile bool rightIndicatorState = false;

volatile bool timerInterruptAttached = false;

// Timer-attachable funtion for left indicator blinking.
void leftIndicatorBlink()
{
    leftIndicatorState = !leftIndicatorState;
    digitalWrite(leftIndicatorPin, leftIndicatorState);
}

// Timer-attachable funtion for right indicator blinking.
void rightIndicatorBlink()
{
    rightIndicatorState = !rightIndicatorState;
    digitalWrite(rightIndicatorPin, rightIndicatorState);
}

// Timer-attachable funtion for alert indicator blinking.
void alertBlink()
{
    leftIndicatorState = !leftIndicatorState;
    rightIndicatorState = leftIndicatorState;
    digitalWrite(leftIndicatorPin, leftIndicatorState);
    digitalWrite(rightIndicatorPin, rightIndicatorState);
}

// Verify code; only one of left, right, and alert byte should be 1.
bool verifyCode(byte code)
{
    byte mask = 0b00001000;
    int highCnt = 0;
    
    for (int i = 0; i < 3; i++)
    {
        if (code & mask)
            ++highCnt;
        mask <<= 1;
    }
    
    if (highCnt == 0 || highCnt == 1)
        return true;
    else
        return false;
}

void setup()
{
    // Initialize both serial buffers.
    memset(inboundSerialBuffer, '\0', 2);
    memset(outboundSerialBuffer, '\0', 3);
    
    // Set pin modes.
    pinMode(leftIndicatorPin, OUTPUT);
    pinMode(rightIndicatorPin, OUTPUT);
    pinMode(frontLightPin, OUTPUT);
    pinMode(rearLightPin, OUTPUT);

    // Start serial communication.
    Serial.begin(9600);

    // Initialize timer interrupt.
    Timer1.initialize(indicatorBlinkInterval * 1000);

    // Indicate the computer that the Arduino is ready.
    outboundSerialBuffer[0] = 0x01;
    outboundSerialBuffer[1] = '\n';
    outboundSerialBuffer[2] = '\0';
//    delay(1000);
    Serial.print(outboundSerialBuffer);
}

void loop()
{
    serial.Receive();
    if (serial.IsDataReady())
    {
        Serial.print(inboundSerialBuffer);
        if (verifyCode(inboundSerialBuffer[0]) == false)
        {
            outboundSerialBuffer[0] = 'a';
            outboundSerialBuffer[1] = '\n';
            outboundSerialBuffer[2] = '\0';
            Serial.println("Invalid");
        }
        else
        {
            if (timerInterruptAttached)
            {
                Timer1.detachInterrupt();
                timerInterruptAttached = false;
                leftIndicatorState = rightIndicatorState = false;
                digitalWrite(leftIndicatorPin, leftIndicatorState);
                digitalWrite(rightIndicatorPin, rightIndicatorState);
            }
            
            if (inboundSerialBuffer[0] & 0b00001000)        // Case: Alert indicator
            {
                Timer1.attachInterrupt(alertBlink);
                timerInterruptAttached = true;
            }
            else if (inboundSerialBuffer[0] & 0b00010000)   // Case: Right indicator
            {
                Timer1.attachInterrupt(rightIndicatorBlink);
                timerInterruptAttached = true;
            }
            else if (inboundSerialBuffer[0] & 0b00100000)   // Case: Left indicator
            {
                Timer1.attachInterrupt(leftIndicatorBlink);
                timerInterruptAttached = true;
            }

            if (inboundSerialBuffer[0] & 0b01000000)        // Rear lights switch
                digitalWrite(rearLightPin, HIGH);
            else
                digitalWrite(rearLightPin, LOW);
    
            if (inboundSerialBuffer[0] & 0b10000000)        // Front lights switch
                digitalWrite(frontLightPin, HIGH);
            else
                digitalWrite(frontLightPin, LOW);

            outboundSerialBuffer[0] = 0x06;
            outboundSerialBuffer[1] = '\n';
            outboundSerialBuffer[2] = '\0';
            Serial.print(outboundSerialBuffer);
        }

        serial.SetDataState(false);
    }
}
