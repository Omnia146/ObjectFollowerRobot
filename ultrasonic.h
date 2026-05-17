/**
 * @file    ultrasonic.h
 * @brief   HC-SR04 Ultrasonic Sensor Driver for Mbed OS
 * @board   NUCLEO-F401RE (STM32F401RE)
 *
 * This module provides a clean interface to the HC-SR04 ultrasonic
 * distance sensor. It uses a 10 µs trigger pulse and measures the
 * echo pulse width with a Timer to calculate distance in centimetres.
 *
 * Wiring:
 *   Trigger  -> D7  (PA_8)
 *   Echo     -> D8  (PA_9)
 *   VCC      -> 5 V
 *   GND      -> GND
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "mbed.h"

/**
 * @class Ultrasonic
 * @brief Driver for the HC-SR04 ultrasonic distance sensor.
 *
 * Usage:
 *   Ultrasonic sensor(D7, D8);          // trigger, echo
 *   float d = sensor.read_distance_cm(); // blocking call
 */
class Ultrasonic {
public:
    /**
     * @brief Construct a new Ultrasonic sensor object.
     * @param trigger_pin  DigitalOut pin connected to TRIG
     * @param echo_pin     DigitalIn pin connected to ECHO (must be 5 V tolerant)
     */
    Ultrasonic(PinName trigger_pin, PinName echo_pin);

    /**
     * @brief  Perform a single distance measurement (blocking).
     * @return Distance in centimetres (0.0 if no echo received / timeout).
     *
     * The call blocks for up to ~30 ms (timeout for max range).
     */
    float read_distance_cm();

    /**
     * @brief  Get the last measured distance without triggering a new reading.
     * @return Last distance in centimetres.
     */
    float get_last_distance_cm() const;

private:
    DigitalOut  _trigger;   ///< Trigger output pin
    DigitalIn   _echo;      ///< Echo input pin
    Timer       _timer;     ///< High-resolution timer for pulse measurement
    float       _last_distance_cm;  ///< Cache of last measured distance

    static constexpr float SPEED_OF_SOUND_CM_US = 0.0343f; ///< cm per µs
    static constexpr int   TIMEOUT_US = 30000;             ///< 30 ms timeout (~5 m)
};

#endif // ULTRASONIC_H
