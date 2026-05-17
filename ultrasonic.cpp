/**
 * @file    ultrasonic.cpp
 * @brief   HC-SR04 Ultrasonic Sensor Driver Implementation
 * @board   NUCLEO-F401RE (STM32F401RE)
 *
 * Measurement sequence:
 *   1. Pull TRIG HIGH for 10 µs → sensor emits 8-cycle ultrasonic burst.
 *   2. Sensor sets ECHO HIGH for the round-trip travel time.
 *   3. Distance = (pulse_width_us × 0.0343) / 2  [cm].
 */

#include "ultrasonic.h"

/* ---------- Constructor ---------- */
Ultrasonic::Ultrasonic(PinName trigger_pin, PinName echo_pin)
    : _trigger(trigger_pin),
      _echo(echo_pin),
      _last_distance_cm(0.0f)
{
    _trigger = 0;   // Ensure trigger starts LOW
}

/* ---------- Public: blocking distance read ---------- */
float Ultrasonic::read_distance_cm()
{
    /* --- 1. Send 10 µs trigger pulse --- */
    _trigger = 0;
    wait_us(2);          // Ensure clean LOW before pulse
    _trigger = 1;
    wait_us(10);         // HC-SR04 requires ≥ 10 µs trigger
    _trigger = 0;

    /* --- 2. Wait for ECHO pin to go HIGH (start of pulse) --- */
    _timer.reset();
    _timer.start();

    while (_echo.read() == 0) {
        if (_timer.elapsed_time().count() > TIMEOUT_US) {
            _timer.stop();
            _last_distance_cm = 0.0f;   // No echo – obstacle out of range
            return _last_distance_cm;
        }
    }

    /* --- 3. Measure ECHO HIGH duration --- */
    _timer.reset();

    while (_echo.read() == 1) {
        if (_timer.elapsed_time().count() > TIMEOUT_US) {
            _timer.stop();
            _last_distance_cm = 0.0f;   // Pulse too long – out of range
            return _last_distance_cm;
        }
    }

    _timer.stop();

    /* --- 4. Calculate distance --- */
    long pulse_us = _timer.elapsed_time().count();
    _last_distance_cm = (pulse_us * SPEED_OF_SOUND_CM_US) / 2.0f;

    return _last_distance_cm;
}

/* ---------- Public: cached getter ---------- */
float Ultrasonic::get_last_distance_cm() const
{
    return _last_distance_cm;
}
