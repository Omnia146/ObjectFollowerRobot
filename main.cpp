/**
 * ============================================================================
 *  Object-Following Robot Car
 * ============================================================================
 *  Board  : NUCLEO-F401RE (STM32F401RE, ARM Cortex-M4 @ 84 MHz)
 *  IDE    : Mbed Studio
 *  OS     : Mbed OS 6.x
 *
 *  Description
 *  -----------
 *  The robot uses an HC-SR04 ultrasonic sensor mounted on the front to
 *  measure the distance to an object ahead.  A simple state machine
 *  controls the two DC motors (via an L298N H-bridge) so that the car:
 *
 *    1. Follows an object moving in a straight line.
 *    2. Maintains a safe following distance (~25 cm).
 *    3. Slows down proportionally as it approaches the object.
 *    4. Stops completely when the object stops or is too close.
 *    5. Stops when no object is detected (out of range).
 *
 *  Pin Mapping
 *  -----------
 *    HC-SR04 Trigger  -> D7  (PA_8)
 *    HC-SR04 Echo     -> D8  (PA_9)
 *    Motor L  ENA     -> D5  (PB_4)   [PWM]
 *    Motor L  IN1     -> D2  (PA_10)
 *    Motor L  IN2     -> D10 (PB_6)
 *    Motor R  ENB     -> D6  (PB_10)  [PWM]
 *    Motor R  IN3     -> D4  (PB_5)
 *    Motor R  IN4     -> D9  (PC_7)
 *    On-board LED     -> LED1 (PA_5)  [status indicator]
 *
 *  State Machine
 *  -------------
 *    STATE_STOP       : Object too close (< 15 cm) or not detected → motors OFF
 *    STATE_CRAWL      : Object at 15 – 25 cm → move at 30 % speed
 *    STATE_FOLLOW     : Object at 25 – 50 cm → move at 60 % speed
 *    STATE_FAST       : Object at 50 – 100 cm → move at 90 % speed
 *    STATE_SEARCH     : Object > 100 cm / lost → motors OFF, LED blink
 *
 *  Startup Code
 *  ------------
 *  Mbed OS performs the following before main() is entered:
 *    - SystemInit() configures the clock tree (HSI/PLL → 84 MHz SYSCLK).
 *    - The C/C++ runtime initialises .data and .bss sections.
 *    - Static constructors run (our global objects are created).
 *    - NVIC priorities are configured by the RTOS kernel.
 *    - The RTOS scheduler starts; main() runs as the default thread.
 *
 *  ISRs Used
 *  ---------
 *    - SysTick_Handler : Mbed RTOS tick (1 ms) for thread scheduling.
 *    - TIMx_IRQHandler : Used internally by PwmOut for motor PWM generation.
 *    - No user-registered ISRs; sensor reads are polled in the main loop.
 * ============================================================================
 */

#include "mbed.h"
#include "ultrasonic.h"
#include "motor_driver.h"

/* ======================== Pin Definitions ======================== */
#define TRIG_PIN    D7      // HC-SR04 Trigger
#define ECHO_PIN    D8      // HC-SR04 Echo

#define ENA_PIN     D5      // Left motor PWM
#define IN1_PIN     D2      // Left motor direction 1
#define IN2_PIN     D10     // Left motor direction 2
#define ENB_PIN     D6      // Right motor PWM
#define IN3_PIN     D4      // Right motor direction 1
#define IN4_PIN     D9      // Right motor direction 2

/* =================== Distance Thresholds (cm) =================== */
static const float DIST_TOO_CLOSE  =  15.0f;   // Emergency stop zone
static const float DIST_CLOSE      =  25.0f;   // Crawl zone
static const float DIST_MEDIUM     =  50.0f;   // Normal follow zone
static const float DIST_FAR        = 100.0f;    // Fast approach zone
// Beyond DIST_FAR → object lost / search mode

/* ======================= Speed Settings ========================= */
static const float SPEED_CRAWL     = 0.30f;    // 30 % duty cycle
static const float SPEED_FOLLOW    = 0.60f;    // 60 % duty cycle
static const float SPEED_FAST      = 0.90f;    // 90 % duty cycle

/* ======================== Robot States ========================== */
enum RobotState {
    STATE_STOP,     // Object too close or initial state
    STATE_CRAWL,    // Approaching slowly
    STATE_FOLLOW,   // Normal following speed
    STATE_FAST,     // Object moving away quickly
    STATE_SEARCH    // Object lost / out of range
};

/* ================= Global Hardware Objects ====================== */
Ultrasonic  sensor(TRIG_PIN, ECHO_PIN);
MotorDriver motors(ENA_PIN, IN1_PIN, IN2_PIN,
                   ENB_PIN, IN3_PIN, IN4_PIN);

DigitalOut  led(LED1);              // On-board LED for status
/* NOTE: In Mbed OS 6, Serial is removed. Use printf() which routes
 * to the default STDIO (USBTX/USBRX) configured in mbed_app.json. */

/* ============== Helper: state name for serial output ============ */
const char* state_name(RobotState s)
{
    switch (s) {
        case STATE_STOP:   return "STOP";
        case STATE_CRAWL:  return "CRAWL";
        case STATE_FOLLOW: return "FOLLOW";
        case STATE_FAST:   return "FAST";
        case STATE_SEARCH: return "SEARCH";
        default:           return "UNKNOWN";
    }
}

/* ================================================================
 *                          MAIN
 * ================================================================ */
int main()
{
    printf("\r\n========================================\r\n");
    printf("  Object-Following Robot Car\r\n");
    printf("  Board : NUCLEO-F401RE\r\n");
    printf("  Sensor: HC-SR04 Ultrasonic\r\n");
    printf("========================================\r\n\r\n");

    /* --- Initial safety stop --- */
    motors.stop();
    led = 0;

    RobotState current_state = STATE_STOP;
    RobotState previous_state = STATE_SEARCH;   // Force first print

    int lost_counter = 0;       // Counts consecutive "no object" readings
    const int LOST_THRESHOLD = 5;  // Readings before declaring "lost"

    /* -------- Main control loop (runs every ~100 ms) -------- */
    while (true) {

        /* 1. Read distance from ultrasonic sensor */
        float distance = sensor.read_distance_cm();

        /* 2. Determine new state based on distance */
        if (distance <= 0.0f || distance > DIST_FAR) {
            /*
             * No echo received or object beyond max follow range.
             * Increment lost counter; only enter SEARCH after
             * several consecutive lost readings (noise filter).
             */
            lost_counter++;
            if (lost_counter >= LOST_THRESHOLD) {
                current_state = STATE_SEARCH;
            }
        } else {
            /* Valid reading – reset lost counter */
            lost_counter = 0;

            if (distance < DIST_TOO_CLOSE) {
                current_state = STATE_STOP;         // Too close → brake
            } else if (distance < DIST_CLOSE) {
                current_state = STATE_CRAWL;        // Getting close → slow
            } else if (distance < DIST_MEDIUM) {
                current_state = STATE_FOLLOW;       // Ideal range → follow
            } else {
                current_state = STATE_FAST;         // Far → speed up
            }
        }

        /* 3. Act on the current state */
        switch (current_state) {

            case STATE_STOP:
                motors.stop();
                led = 1;   // Solid LED = stopped
                break;

            case STATE_CRAWL:
                motors.forward(SPEED_CRAWL);
                led = 1;
                break;

            case STATE_FOLLOW:
                motors.forward(SPEED_FOLLOW);
                led = 1;
                break;

            case STATE_FAST:
                motors.forward(SPEED_FAST);
                led = 1;
                break;

            case STATE_SEARCH:
                motors.stop();
                led = !led;   // Blink LED = searching
                break;
        }

        /* 4. Print status on state change or periodically */
        if (current_state != previous_state) {
            printf("[STATE] %s -> %s | Distance: %.1f cm\r\n",
                   state_name(previous_state),
                   state_name(current_state),
                   distance);
            previous_state = current_state;
        }

        /* 5. Debug output every cycle */
        printf("Distance: %6.1f cm | State: %-7s\r\n",
               distance, state_name(current_state));

        /* 6. Loop delay – sensor needs ~60 ms between readings */
        ThisThread::sleep_for(100ms);
    }
}
