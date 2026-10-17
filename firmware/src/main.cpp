/**
 * ESP32-S3 Robot Arm — Serial Control Firmware
 *
 * Receives joint angles from Python deploy script via USB Serial.
 * Protocol: "<base>,<shoulder>,<elbow>,<gripper>\n"
 * Example:  "90,75,110,90\n"
 *
 * Responds with "OK\n" after moving servos.
 *
 * Joints:
 *   Servo 0 — Base     (GPIO 4)   0–180°
 *   Servo 1 — Shoulder (GPIO 5)  30–150°
 *   Servo 2 — Elbow    (GPIO 6)  30–150°
 *   Servo 3 — Gripper  (GPIO 7)  60–120°
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ── Pin assignments ───────────────────────────────────────────
static const int PIN_BASE     = 4;
static const int PIN_SHOULDER = 5;
static const int PIN_ELBOW    = 6;
static const int PIN_GRIPPER  = 7;

// ── Joint limits (degrees) ────────────────────────────────────
static const int BASE_MIN     =   0,  BASE_MAX     = 180;
static const int SHOULDER_MIN =  30,  SHOULDER_MAX = 150;
static const int ELBOW_MIN    =  30,  ELBOW_MAX    = 150;
static const int GRIPPER_MIN  =  60,  GRIPPER_MAX  = 120;

// ── Home position ─────────────────────────────────────────────
static const int HOME_BASE     = 90;
static const int HOME_SHOULDER = 90;
static const int HOME_ELBOW    = 90;
static const int HOME_GRIPPER  = 90;

// ── Servo pulse range (µs) ────────────────────────────────────
// Same as the original test firmware; ESP32Servo's default would be 544–2400 µs,
// which shifts every angle by ~2°.
static const int PULSE_MIN_US = 500;
static const int PULSE_MAX_US = 2400;

// ── Serial ────────────────────────────────────────────────────
static const int BAUD_RATE    = 115200;
static const int MAX_MSG_LEN  = 32;

// ── Staggered motion ──────────────────────────────────────────
// Starting all servos at once draws the largest current peak, which a weak 5 V supply
// cannot deliver (servos hum, positions drift). Joints are started one after another
// instead, and joints whose target did not change are not touched at all.
static const int STAGGER_MS = 120;   // pause between starting two servos
static const int SETTLE_MS  = 300;   // wait after the last start before detaching

// ── Joints ────────────────────────────────────────────────────
static const int NUM_JOINTS = 4;
static const int UNKNOWN    = -1;    // position before the first move after boot

Servo servo_base;
Servo servo_shoulder;
Servo servo_elbow;
Servo servo_gripper;

struct Joint {
    Servo& servo;
    int    pin;
    int    min_deg;
    int    max_deg;
    int    current;   // last commanded angle (SG90 has no position feedback)
};

// Order = order of the protocol fields = order in which joints are started.
Joint joints[NUM_JOINTS] = {
    {servo_base,     PIN_BASE,     BASE_MIN,     BASE_MAX,     UNKNOWN},
    {servo_shoulder, PIN_SHOULDER, SHOULDER_MIN, SHOULDER_MAX, UNKNOWN},
    {servo_elbow,    PIN_ELBOW,    ELBOW_MIN,    ELBOW_MAX,    UNKNOWN},
    {servo_gripper,  PIN_GRIPPER,  GRIPPER_MIN,  GRIPPER_MAX,  UNKNOWN},
};


// ── Helpers ───────────────────────────────────────────────────

int clamp(int value, int min_val, int max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void detach_all() {
    for (Joint& j : joints) j.servo.detach();   // reduce jitter when idle
}

/**
 * Move to the target angles (degrees, clamped to the joint limits), starting the joints
 * one after another. Blocks until the servos had time to arrive, then detaches them.
 */
void move_staggered(const int target[NUM_JOINTS]) {
    bool started = false;
    for (Joint& j : joints) {
        const int angle = clamp(*target++, j.min_deg, j.max_deg);
        if (angle == j.current) continue;

        if (started) delay(STAGGER_MS);
        j.servo.attach(j.pin, PULSE_MIN_US, PULSE_MAX_US);
        j.servo.write(angle);
        j.current = angle;
        started = true;
    }
    if (started) delay(SETTLE_MS);
    detach_all();
}


// ── Parse incoming message ─────────────────────────────────────

/**
 * Parse a message of the form "90,75,110,90" into four integers.
 *
 * Returns true on success, false if the format is invalid.
 */
bool parse_angles(const char* msg, int& base, int& shoulder, int& elbow, int& gripper) {
    int parsed = sscanf(msg, "%d,%d,%d,%d", &base, &shoulder, &elbow, &gripper);
    return parsed == 4;
}


// ── Setup ─────────────────────────────────────────────────────

void setup() {
    Serial.begin(BAUD_RATE);
    delay(500);

    // Move to home position on startup (all joints, since their position is unknown)
    const int home[NUM_JOINTS] = {HOME_BASE, HOME_SHOULDER, HOME_ELBOW, HOME_GRIPPER};
    move_staggered(home);

    Serial.println("READY");
}


// ── Main loop ─────────────────────────────────────────────────

void loop() {
    static char  buf[MAX_MSG_LEN];
    static int   buf_idx = 0;

    // Read bytes until newline
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            if (buf_idx > 0) {
                buf[buf_idx] = '\0';   // null-terminate

                int target[NUM_JOINTS];
                if (parse_angles(buf, target[0], target[1], target[2], target[3])) {
                    move_staggered(target);
                    Serial.println("OK");
                } else {
                    Serial.print("ERR: bad format: ");
                    Serial.println(buf);
                }

                buf_idx = 0;   // reset buffer
            }
        } else if (buf_idx < MAX_MSG_LEN - 1) {
            buf[buf_idx++] = c;
        }
    }
}
