#ifndef MOTOR_H_
#define MOTOR_H_

class EncoderMotor
{
    public:
    EncoderMotor(int motor_dir, int motor_pwm, int encoder_clk, int encoder_dir);

    public:
    void SetSpeed(int val);

    public:
    void Increment();
    void Decrement();

    public:
    int GetEncoderClkPin();
    int GetEncoderDirPin();
    long GetEncoderVal();
    double GetEncoderSpeed();

    private:
    int motor_dir_pin_;
    int motor_pwm_pin_;
    int motor_dir_val_;
    int motor_pwm_val_;

    private:
    int encoder_clk_pin_;
    int encoder_dir_pin_;
    long timestamp_curr_;
    long timestamp_prev_;
    long encoder_val_;
    long encoder_speed_;
};

EncoderMotor::EncoderMotor(int dir_pin, int pwm_pin, int encoder_clk, int encoder_dir)
  : motor_dir_pin_(dir_pin), motor_pwm_pin_(pwm_pin),
    encoder_clk_pin_(encoder_clk), encoder_dir_pin_(encoder_dir),
    timestamp_curr_(0), timestamp_prev_(0)
{
    pinMode(motor_dir_pin_, OUTPUT);
    pinMode(motor_pwm_pin_, OUTPUT);
    pinMode(encoder_clk_pin_, INPUT);
    pinMode(encoder_dir_pin_, INPUT);
}

void EncoderMotor::SetSpeed(int val)
{
    if (val > 0)
    {
        motor_dir_val_ = HIGH;
        motor_pwm_val_ = val;
    }
    else
    {
        motor_dir_val_ = LOW;
        motor_pwm_val_ = -val;
    }

    digitalWrite(motor_dir_pin_, motor_dir_val_);
    analogWrite(motor_pwm_pin_, motor_pwm_val_);
}

void EncoderMotor::Increment()
{
    ++encoder_val_;
    timestamp_curr_ = micros();
    encoder_speed_ = MICROS / (timestamp_curr_ - timestamp_prev_);
    timestamp_prev_ = timestamp_curr_;
}

void EncoderMotor::Decrement()
{
    --encoder_val_;
    timestamp_curr_ = micros();
    encoder_speed_ = -MICROS / (timestamp_curr_ - timestamp_prev_);
    timestamp_prev_ = timestamp_curr_;
}

int EncoderMotor::GetEncoderClkPin()
{
    return encoder_clk_pin_;
}

int EncoderMotor::GetEncoderDirPin()
{
    return encoder_dir_pin_;
}

long EncoderMotor::GetEncoderVal()
{
    return encoder_val_;
}

long EncoderMotor::GetEncoderDiff()
{
    return encoder_val_ - 
}

double EncoderMotor::GetEncoderSpeed()
{
    return encoder_speed_;
}

#endif
