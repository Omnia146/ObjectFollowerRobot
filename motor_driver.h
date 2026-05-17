/**
 * @file    motor_driver.h
 * @brief   L298N Dual H-Bridge Motor Driver for Mbed OS
 * @board   NUCLEO-F401RE (STM32F401RE)
 *
 * This module controls two DC motors via an L298N motor driver board.
 * Each motor is controlled by two direction pins (IN1/IN2 or IN3/IN4)
 * and one PWM-capable enable pin for speed control.
 *
 * Wiring (default pins):
 *   Left Motor :  ENA -> D5 (PB_4),  IN1 -> D2 (PA_10), IN2 -> D10 (PB_6)
 *   Right Motor:  ENB -> D6 (PB_10), IN3 -> D4 (PB_5),  IN4 -> D9 (PC_7)
 *
 * Speed is set as a float 0.0 – 1.0 (0 % – 100 % duty cycle).
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "mbed.h"

/**
 * @class MotorDriver
 * @brief Controls a differential-drive robot (two DC motors via L298N).
 */
class MotorDriver {
public:
    /**
     * @brief Construct the motor driver with specified pins.
     *
     * @param ena   PWM pin for left motor speed
     * @param in1   Direction pin 1 for left motor
     * @param in2   Direction pin 2 for left motor
     * @param enb   PWM pin for right motor speed
     * @param in3   Direction pin 1 for right motor
     * @param in4   Direction pin 2 for right motor
     */
    MotorDriver(PinName ena, PinName in1, PinName in2,
                PinName enb, PinName in3, PinName in4);

    /**
     * @brief Move both motors forward at the given speed.
     * @param speed  Duty cycle 0.0 – 1.0
     */
    void forward(float speed);

    /**
     * @brief Move both motors backward at the given speed.
     * @param speed  Duty cycle 0.0 – 1.0
     */
    void backward(float speed);

    /**
     * @brief Stop both motors immediately (brake).
     */
    void stop();

    /**
     * @brief Set individual motor speeds and directions.
     *
     * Positive speed = forward, negative = backward. Range: -1.0 to 1.0
     *
     * @param left_speed   Left motor speed
     * @param right_speed  Right motor speed
     */
    void set_motors(float left_speed, float right_speed);

private:
    PwmOut     _ena;    ///< Left motor PWM enable
    DigitalOut _in1;    ///< Left motor direction pin 1
    DigitalOut _in2;    ///< Left motor direction pin 2
    PwmOut     _enb;    ///< Right motor PWM enable
    DigitalOut _in3;    ///< Right motor direction pin 1
    DigitalOut _in4;    ///< Right motor direction pin 2

    /**
     * @brief  Clamp a value between -1.0 and 1.0
     */
    float clamp(float value);

    /**
     * @brief  Set a single motor's speed and direction.
     * @param  pwm   PWM output for speed
     * @param  fwd   Forward direction pin
     * @param  rev   Reverse direction pin
     * @param  speed Signed speed (-1.0 to 1.0)
     */
    void set_single_motor(PwmOut &pwm, DigitalOut &fwd,
                          DigitalOut &rev, float speed);
};

#endif // MOTOR_DRIVER_H
