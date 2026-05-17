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
static const float DIST_TOO_CLOSE  =  20.0f;   // Emergency stop zone
static const float DIST_CLOSE      =  25.0f;   // Crawl zone
static const float DIST_MEDIUM     =  50.0f;   // Normal follow zone
static const float DIST_FAR        = 100.0f;    // Fast approach zone
// Beyond DIST_FAR → object lost / search mode

/* ======================= Speed Settings ========================= */
static const float SPEED_CRAWL     = 0.25f;   // 25% for careful approach
static const float SPEED_FOLLOW    = 0.45f;   // 45% standard cruise speed
static const float SPEED_FAST      = 0.70f;   // 70% to catch up quickly
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
 * MAIN
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
        ThisThread::sleep_for(60ms);
    }
}

