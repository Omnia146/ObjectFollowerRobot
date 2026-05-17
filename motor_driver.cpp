/**
 * @file    motor_driver.cpp
 * @brief   L298N Motor Driver Implementation
 * @board   NUCLEO-F401RE (STM32F401RE)
 *
 * PWM frequency is set to 1 kHz (1 ms period) which is suitable
 * for most small DC motors used in robot car kits.
 */

#include "motor_driver.h"

MotorDriver::MotorDriver(PinName ena, PinName in1, PinName in2,
                         PinName enb, PinName in3, PinName in4)
    : _ena(ena), _in1(in1), _in2(in2),
      _enb(enb), _in3(in3), _in4(in4)
{
    _ena.period_ms(1);
    _enb.period_ms(1);
    stop();
}

void MotorDriver::forward(float speed)
{
    speed = clamp(speed);
    set_single_motor(_ena, _in1, _in2,  speed);
    set_single_motor(_enb, _in3, _in4,  speed);
}

void MotorDriver::backward(float speed)
{
    speed = clamp(speed);
    set_single_motor(_ena, _in1, _in2, -speed);
    set_single_motor(_enb, _in3, _in4, -speed);
}

void MotorDriver::stop()
{
    _ena.write(0.0f);
    _enb.write(0.0f);
    _in1 = 0;
    _in2 = 0;
    _in3 = 0;
    _in4 = 0;
}

void MotorDriver::set_motors(float left_speed, float right_speed)
{
    set_single_motor(_ena, _in1, _in2, left_speed);
    set_single_motor(_enb, _in3, _in4, right_speed);
}

float MotorDriver::clamp(float value)
{
    if (value >  1.0f) return  1.0f;
    if (value < -1.0f) return -1.0f;
    return value;
}

void MotorDriver::set_single_motor(PwmOut &pwm, DigitalOut &fwd,
                                   DigitalOut &rev, float speed)
{
    speed = clamp(speed);
    if (speed > 0.0f) {
        fwd = 1;  rev = 0;
        pwm.write(speed);
    } else if (speed < 0.0f) {
        fwd = 0;  rev = 1;
        pwm.write(-speed);
    } else {
        fwd = 0;  rev = 0;
        pwm.write(0.0f);
    }
}
