/*
********************************************************************************
* Filename      : Kinematic Controller
* Author        : Susung Park
* Description   : Kinematics controller for autonomous vehicle.
* Version       : On development...
********************************************************************************
*/

#include "pid_controller.h"
#include "serial_handler.h"
#include "motor.h"

// Serial handler buffers.
char inboundSerialBuffer[9];
char outboundSerialBuffer[13];
SerialHandler serial(inboundSerialBuffer, outboundSerialBuffer);

// Encoder values
EncoderMotor motor1(30, 8, 2, 22);
EncoderMotor motor2(32, 9, 3, 24);
EncoderMotor motor3(34, 10, 4, 26);
EncoderMotor motor4(36, 11, 5, 28);

// Values for controls.
long motor1VelTarget, motor2VelTarget, motor3VelTarget, motor4VelTarget;

// Initiate PID controllers.
PIDController<long, long> motor1VelocityPID(2, 0.001, 0.001);
PIDController<long, long> motor2VelocityPID(2, 0.001, 0.001);
PIDController<long, long> motor3VelocityPID(2, 0.001, 0.001);
PIDController<long, long> motor4VelocityPID(2, 0.001, 0.001);
int motor1VelocityPIDOutput, motor2VelocityPIDOutput, motor3VelocityPIDOutput, motor4VelocityPIDOutput;

void encoder1Callback()
{
    if (digitalRead(motor1.GetEncoderDirPin()) == LOW)
        motor1.Increment();
    else
        motor1.Decrement();

    velocityPID.SetSysoutVal(motor1.GetEncoderSpeed());
}

void encoder2Callback()
{
    if (digitalRead(motor2.GetEncoderDirPin()) == LOW)
        motor2.Increment();
    else
        motor2.Decrement();

    velocityPID.SetSysoutVal(motor2.GetEncoderSpeed());
}

void encoder3Callback()
{
    if (digitalRead(motor3.GetEncoderDirPin()) == LOW)
        motor3.Increment();
    else
        motor3.Decrement();

    velocityPID.SetSysoutVal(motor3.GetEncoderSpeed());
}

void encoder4Callback()
{
    if (digitalRead(motor4.GetEncoderDirPin()) == LOW)
        motor4.Increment();
    else
        motor4.Decrement();
        
    velocityPID.SetSysoutVal(motor4.GetEncoderSpeed());
}

void decodeInboundSerial(int &v1, int &v2, int &v3, int &v4)
{
    int *ptr = (int *)inboundSerialBuffer;
    v1 = *(ptr + 0);
    v2 = *(ptr + 1);
    v3 = *(ptr + 2);
    v4 = *(ptr + 3);
}

void encodeOutboundSerial(long time_diff, int diff1, int diff2, int diff3, int diff4)
{
    int *int_ptr = (int *)(outboundSerialBuffer + 4);
    long *long_ptr = (long *)(outboundSerialBuffer);
    *long_ptr = time_diff;
    *(int_ptr + 0) = diff1;
    *(int_ptr + 1) = diff2;
    *(int_ptr + 2) = diff3;
    *(int_ptr + 3) = diff4;
}

void setup()
{
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(2), encoder1Callback, RISING);
    attachInterrupt(digitalPinToInterrupt(3), encoder2Callback, RISING);
    attachInterrupt(digitalPinToInterrupt(4), encoder3Callback, RISING);
    attachInterrupt(digitalPinToInterrupt(5), encoder4Callback, RISING);
}

void loop()
{
    serial.Receive();
    if (serial.IsDataReady())
    {
        // Decode inbound serial value.
        decodeInboundSerial(motor1VelTarget, motor2VelTarget, motor3VelTarget, motor4VelTarget);

        // Set target value.
        motor1VelocityPID.SetTargetVal(motor1VelTarget);
        motor1VelocityPID.SetTargetVal(motor1VelTarget);
        motor1VelocityPID.SetTargetVal(motor1VelTarget);
        motor1VelocityPID.SetTargetVal(motor1VelTarget);

        // Encode outbound serial buffer and transmit.
        
        
        // Flush serial handler.
        serial.ConsumeData();
    }

    velocityPIDOutput = velocityPID.ExecClippedPID(positionPIDOutput, -50, 50);
    motor.SetSpeed(velocityPIDOutput);
}
