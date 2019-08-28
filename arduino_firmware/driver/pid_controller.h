/*
********************************************************************************
* Filename      : PID Controller
* Author        : Susung Park
* Description   : Generic PID controller
* Version       : On development...
********************************************************************************
*/

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#define MILLIS 1000
#define MICROS 1000000

template <typename T_src, typename T_out>
class PIDController
{
public:
    PIDController(T_out, T_out, T_out);

public:
    void SetTargetVal(T_src val);
    void SetSysoutVal(T_src val);

public:
    T_out ExecFreePID(T_src&);
    T_out ExecClippedPID(T_src&, T_out, T_out);

private: // PID gains
    T_out kp_;
    T_out ki_;
    T_out kd_;

private: // PID computations
    T_out err_curr_;
    T_out err_prev_;
    T_out err_gross_;
    T_out err_integral_;
    T_out err_derivative_;

private: // State variables
    T_src target_val_;
    T_src sysout_val_;
    long timestamp_curr_;        // Current timestamp
    long timestamp_prev_;        // Previous timestamp
    long timestamp_diff_;        // Time step between current and previous timestamp
};

template <typename T_src, typename T_out>
PIDController<T_src, T_out>::PIDController(T_out kp, T_out ki, T_out kd)
  : kp_(kp), ki_(ki), kd_(kd),
    err_curr_(0), err_prev_(0), err_integral_(0), err_derivative_(0),
    target_val_(0), sysout_val_(0),
    timestamp_curr_(0), timestamp_prev_(0), timestamp_diff_(0)
{
    
}

template <typename T_src, typename T_out>
void PIDController<T_src, T_out>::SetTargetVal(T_src val)
{
    target_val_ = val;
}

template <typename T_src, typename T_out>
void PIDController<T_src, T_out>::SetSysoutVal(T_src val)
{
    sysout_val_ = val;
}

template <typename T_src, typename T_out>
T_out PIDController<T_src, T_out>::ExecFreePID(T_src &tracking_val)
{
    // Calculate time difference.
    timestamp_curr_ = micros();
    timestamp_diff_ = timestamp_curr_ - timestamp_prev_;

    // Calculate the error.
    err_curr_ = target_val_ - sysout_val_;

    // Calculate each PID variable.
    err_gross_ += err_curr_;
    err_integral_ = err_gross_ / MICROS;
    err_derivative_ = (err_curr_ - err_prev_) * MICROS;

    T_out pidVal = T_out(err_curr_ * kp_) + T_out(err_integral_ * ki_) + T_out(err_derivative_ * kd_);
    err_prev_ = err_curr_;
    return pidVal;
}

template <typename T_src, typename T_out>
T_out PIDController<T_src, T_out>::ExecClippedPID(T_src &tracking_val, T_out min_bound, T_out max_bound)
{
    T_out pidVal = ExecFreePID(tracking_val);
    pidVal = (pidVal > max_bound ? max_bound : (pidVal < min_bound ? min_bound : pidVal));
    return pidVal;
}

#endif
