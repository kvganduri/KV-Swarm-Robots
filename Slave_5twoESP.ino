// --- START: Rover 1 ESP32 Code (V1.7 - Minimal Serial Output) ---
// Based on V1.6, removed all Serial prints except currentYaw and PID E/C values.

#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h> // Using MPU6050_light library
#include <PID_v1.h>       // Include PID library
#include <cmath>          // For trig functions

// --- Network Config ---
const char *ssid = "ROVER_CONTROL"; // Must match Master AP SSID
const char *password = "12345678";   // Must match Master AP password

// --- Pin Definitions (FROM V4.3 - VERIFY THESE!) ---
// L298N Motor Driver Pins
#define ENA_L_PIN 26  // Left Motors Enable (PWM Speed) -> Connect to L298N ENA
#define IN1_L_PIN 27  // Left Motors Direction Input 1 -> Connect to L298N IN1
#define IN2_L_PIN 32  // Left Motors Direction Input 2 -> Connect to L298N IN2

#define ENA_R_PIN 33  // Right Motors Enable (PWM Speed) -> Connect to L298N ENB
#define IN3_R_PIN 25  // Right Motors Direction Input 3 -> Connect to L298N IN3
#define IN4_R_PIN 17  // Right Motors Direction Input 4 -> Connect to L298N IN4

// Encoder Pins (LF = Index 0, RF = Index 1) - From V4.3
#define ENC_LF 0
#define ENC_RF 1
const int NUM_ENCODERS = 2; // Only using 2 encoders
const int encoderPins[NUM_ENCODERS][2] = {
    {14, 12}, // LF Encoder Pins [A, B] - Index 0
    {13, 16}  // RF Encoder Pins [A, B] - Index 1
};

#define LEFT_SIDE 0
#define RIGHT_SIDE 1
const int NUM_SIDES = 2;
const int PWM_FREQ = 5000; const int PWM_RESOLUTION = 8; const int MAX_PWM_VALUE = 255;
const int ledcChannels[] = {0, 1}; // LEDC channels for PWM (Left=0, Right=1)

// --- Speed & Control Definitions ---
const int FWD_SPEED_BASE = 70;
const int TURN_SPEED = 100;
const float COUNTS_PER_CM = 75.0; // !!! REPLACE with your actual calibrated value !!!
const float TURN_ANGLE_THRESHOLD = 1;
const unsigned long TASK_PAUSE_DURATION_MS = 1000;

// --- PID Tuning Parameters ---
double Kp_pid = 100;
double Ki_pid = 5.0;
double Kd_pid = 1;

// --- Global Variables ---
volatile long encoderCounts[NUM_ENCODERS] = {0, 0}; // LF, RF
portMUX_TYPE encoderMutex[NUM_ENCODERS];
long startEncoderTicks[NUM_ENCODERS];
MPU6050 mpu(Wire);
float targetYaw = 0.0; float currentYaw = 0.0;

// PID Variables
double headingError, pidOutputCorrection, headingSetpoint = 0;
PID headingPID(&headingError, &pidOutputCorrection, &headingSetpoint, Kp_pid, Ki_pid, Kd_pid, DIRECT);

// State Machine Variables
enum RoverState {
    IDLE,
    TURNING_LEFT,         // Basic 'l' command turn
    TURNING_RIGHT,        // Basic 'r' command turn
    MOVING_FORWARD,       // Basic 'f' command move

    TASK1_LEG1,           // Task 1 specific states
    TASK1_PAUSE_AFTER_LEG1,
    TASK1_TURN,           // Specific state for task 1 turn logic (180 R)
    TASK1_PAUSE_AFTER_TURN,
    TASK1_LEG2,

    SHAPE_N_R1_LEG1,      // Shape 'n' (Rover 1) states (105cm Fwd)
    SHAPE_N_R1_TURN1,     // Specific state for N turn 1 logic (90 L)
    SHAPE_N_R1_LEG2,      // (30cm Fwd)
    SHAPE_N_R1_WAIT_HOME, // Wait state for 'h' after 'n'

    SHAPE_H_N_R1_TURN1,   // Return sequence after 'n' (specific turn 1: 180 R)
    SHAPE_H_N_R1_LEG1,    // (30cm Fwd)
    SHAPE_H_N_R1_TURN2,   // Specific state for N home turn 2 (90 R)
    SHAPE_H_N_R1_LEG2,    // (105cm Fwd)

    SHAPE_M_R1_LEG1,      // Shape 'm' (Rover 1) states (50cm Fwd)
    SHAPE_M_R1_TURN1,     // Specific state for M turn 1 logic (90 L)
    SHAPE_M_R1_LEG2,      // (65cm Fwd)
    SHAPE_M_R1_WAIT_HOME, // Wait state for 'h' after 'm'

    SHAPE_H_M_R1_TURN1,   // Return sequence after 'm' (specific turn 1: 180 R)
    SHAPE_H_M_R1_LEG1,    // (65cm Fwd)
    SHAPE_H_M_R1_TURN2,   // Specific state for M home turn 2 (90 R)
    SHAPE_H_M_R1_LEG2     // (50cm Fwd)
};
RoverState currentState = IDLE;
float targetTurnAngle = 0.0;
float startYawForTurn = 0.0;
long targetEncoderTicks = 0;
unsigned long pauseStartTime = 0;
char lastShapeCommand = '\0';

// --- ESP-NOW Configuration ---
uint8_t masterAddress[] = {0xA8, 0x42, 0xE3, 0x47, 0x27, 0x90}; // User's Master MAC
esp_now_peer_info_t peerInfo;

// --- Data Structures (Must match Master) ---
typedef enum { COMMAND_MOVEMENT, COMMAND_PID_TUNING } message_type_t;
typedef struct { char command_type; float value; } payload_movement_t;
typedef struct { double kp; double ki; double kd; } payload_pid_tuning_t;
typedef struct struct_message_universal { message_type_t msg_type; union { payload_movement_t movement; payload_pid_tuning_t tuning; } payload; } struct_message_universal;

// Encoder data sent to Master (REMOVED - was not requested to keep)
// typedef struct struct_message_encoder_data { long encoderLF; long encoderRF; } struct_message_encoder_data;
// struct_message_encoder_data encoderDataToSend;

// --- Function Prototypes ---
void setSideSpeed(int side, int direction, int speed);
void IRAM_ATTR handleEncoderInterrupt(void *arg);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
float calculateAngleDifference(float targetAngle, float currentAngle);
void stopMotors();
void prepareForwardMove(float distanceCm);

// --- Setup ---
void setup() {
    // *** Initialize Serial, but no other prints ***
    Serial.begin(115200);

    // Initialize MPU6050
    Wire.begin(23, 22);
    byte status = mpu.begin();
    while(status != 0) { delay(100); status = mpu.begin(); }
    delay(1000); mpu.calcOffsets();

    // Initialize PID
    headingPID.SetMode(MANUAL); headingPID.SetSampleTime(10); headingPID.SetOutputLimits(-FWD_SPEED_BASE, FWD_SPEED_BASE);
    headingPID.SetTunings(Kp_pid, Ki_pid, Kd_pid);

    // Get initial Yaw angle
    mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw;

    // Initialize mutexes
    for (int i = 0; i < NUM_ENCODERS; i++) { encoderMutex[i] = portMUX_INITIALIZER_UNLOCKED; }

    // Setup L298N Motor Pins
    pinMode(IN1_L_PIN, OUTPUT); digitalWrite(IN1_L_PIN, LOW);
    pinMode(IN2_L_PIN, OUTPUT); digitalWrite(IN2_L_PIN, LOW);
    pinMode(IN3_R_PIN, OUTPUT); digitalWrite(IN3_R_PIN, LOW);
    pinMode(IN4_R_PIN, OUTPUT); digitalWrite(IN4_R_PIN, LOW);
    ledcSetup(ledcChannels[LEFT_SIDE], PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(ledcChannels[RIGHT_SIDE], PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ENA_L_PIN, ledcChannels[LEFT_SIDE]);
    ledcAttachPin(ENA_R_PIN, ledcChannels[RIGHT_SIDE]);
    ledcWrite(ledcChannels[LEFT_SIDE], 0);
    ledcWrite(ledcChannels[RIGHT_SIDE], 0);

    // Setup Encoder Pins and Interrupts
    for (int i = 0; i < NUM_ENCODERS; i++) { pinMode(encoderPins[i][0], INPUT_PULLUP); pinMode(encoderPins[i][1], INPUT_PULLUP); attachInterruptArg(digitalPinToInterrupt(encoderPins[i][0]), handleEncoderInterrupt, (void*)i, RISING); }

    // Setup WiFi and ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }
    if (esp_now_init() != ESP_OK) { ESP.restart(); } // Minimal error handling
    esp_now_register_send_cb(OnDataSent); memcpy(peerInfo.peer_addr, masterAddress, 6); peerInfo.channel = 0; peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK){ return; } // Minimal error handling
    esp_now_register_recv_cb(OnDataRecv);
}

// --- Loop ---
void loop() {
    mpu.update();
    currentYaw = mpu.getAngleZ();
    // *** ADDED: Print current Yaw ***
    Serial.println(currentYaw);

    // --- State Machine Logic ---
    switch (currentState) {
        case IDLE: if (headingPID.GetMode() == AUTOMATIC) { headingPID.SetMode(MANUAL); } break;

        // --- Basic Turns ---
        case TURNING_LEFT: {
            setSideSpeed(LEFT_SIDE, 1, TURN_SPEED);
            setSideSpeed(RIGHT_SIDE, -1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                stopMotors(); currentState = IDLE;
            } break; }
        case TURNING_RIGHT: {
            setSideSpeed(LEFT_SIDE, -1, TURN_SPEED);
            setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                stopMotors(); currentState = IDLE;
            } break; }

        // --- Forward Movement Handler (Common for all legs) ---
        case MOVING_FORWARD:
        case TASK1_LEG1:
        case TASK1_LEG2:
        case SHAPE_N_R1_LEG1:
        case SHAPE_N_R1_LEG2:
        case SHAPE_M_R1_LEG1:
        case SHAPE_M_R1_LEG2:
        case SHAPE_H_N_R1_LEG1:
        case SHAPE_H_N_R1_LEG2:
        case SHAPE_H_M_R1_LEG1:
        case SHAPE_H_M_R1_LEG2: {
            headingError = calculateAngleDifference(targetYaw, currentYaw);
            headingPID.Compute();
            // *** ADDED: Print Error and Correction ***
            int leftSpeed = FWD_SPEED_BASE + (int)pidOutputCorrection;
            int rightSpeed = FWD_SPEED_BASE - (int)pidOutputCorrection;
            leftSpeed = constrain(leftSpeed, 0, MAX_PWM_VALUE); rightSpeed = constrain(rightSpeed, 0, MAX_PWM_VALUE);
            setSideSpeed(LEFT_SIDE, 1, leftSpeed);
            setSideSpeed(RIGHT_SIDE, 1, rightSpeed);

            // Check Distance Target
            bool targetReached = false; long current_enc[NUM_ENCODERS];
            portENTER_CRITICAL(&encoderMutex[ENC_LF]); current_enc[ENC_LF] = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]);
            portENTER_CRITICAL(&encoderMutex[ENC_RF]); current_enc[ENC_RF] = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]);
            long moved_lf = abs(current_enc[ENC_LF] - startEncoderTicks[ENC_LF]);
            long moved_rf = abs(current_enc[ENC_RF] - startEncoderTicks[ENC_RF]);
            if (moved_lf >= targetEncoderTicks || moved_rf >= targetEncoderTicks) {
                targetReached = true;
            }

            if (targetReached) {
                stopMotors();
                if (headingPID.GetMode() == AUTOMATIC) { headingPID.SetMode(MANUAL); }

                // --- State Transitions after completing a move ---
                if      (currentState == MOVING_FORWARD) { lastShapeCommand = '\0'; currentState = IDLE; }
                else if (currentState == TASK1_LEG1)     { pauseStartTime = millis(); currentState = TASK1_PAUSE_AFTER_LEG1; }
                else if (currentState == TASK1_LEG2)     { lastShapeCommand = '\0'; currentState = IDLE; }
                else if (currentState == SHAPE_N_R1_LEG1){ startYawForTurn = currentYaw; targetTurnAngle = 90.0; currentState = SHAPE_N_R1_TURN1; }
                else if (currentState == SHAPE_N_R1_LEG2){ currentState = SHAPE_N_R1_WAIT_HOME; }
                else if (currentState == SHAPE_H_N_R1_LEG1){ startYawForTurn = currentYaw; targetTurnAngle = 90.0; currentState = SHAPE_H_N_R1_TURN2; }
                else if (currentState == SHAPE_H_N_R1_LEG2){ lastShapeCommand = '\0'; currentState = IDLE; }
                else if (currentState == SHAPE_M_R1_LEG1){ startYawForTurn = currentYaw; targetTurnAngle = 90.0; currentState = SHAPE_M_R1_TURN1; }
                else if (currentState == SHAPE_M_R1_LEG2){ currentState = SHAPE_M_R1_WAIT_HOME; }
                else if (currentState == SHAPE_H_M_R1_LEG1){ startYawForTurn = currentYaw; targetTurnAngle = 90.0; currentState = SHAPE_H_M_R1_TURN2; }
                else if (currentState == SHAPE_H_M_R1_LEG2){ lastShapeCommand = '\0'; currentState = IDLE; }
                else { currentState = IDLE; } // Failsafe
            }
            Serial.printf("E:%.2f C:%.2f\n", headingError, pidOutputCorrection);
            delay(1);
            break; // End forward movement states
        }

        // --- Task 1 Specific States ---
        case TASK1_PAUSE_AFTER_LEG1: { if (millis() - pauseStartTime >= TASK_PAUSE_DURATION_MS) { mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw; targetTurnAngle = 180.0; currentState = TASK1_TURN; } break; }
        case TASK1_TURN: {
            setSideSpeed(LEFT_SIDE, -1, TURN_SPEED);
            setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - (TURN_ANGLE_THRESHOLD * 1.5)) {
                 stopMotors(); pauseStartTime = millis(); currentState = TASK1_PAUSE_AFTER_TURN;
            } break; }
        case TASK1_PAUSE_AFTER_TURN: { if (millis() - pauseStartTime >= TASK_PAUSE_DURATION_MS) { prepareForwardMove(150.0); currentState = TASK1_LEG2; } break; }

        // --- Shape N (Rover 1) Specific States ---
        case SHAPE_N_R1_TURN1: {
            setSideSpeed(LEFT_SIDE, 1, TURN_SPEED);
            setSideSpeed(RIGHT_SIDE, -1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                stopMotors(); prepareForwardMove(30.0); currentState = SHAPE_N_R1_LEG2;
            } break; }
        case SHAPE_N_R1_WAIT_HOME: { break; }

        // --- Shape M (Rover 1) Specific States ---
        case SHAPE_M_R1_TURN1: {
            setSideSpeed(LEFT_SIDE, 1, TURN_SPEED);
            setSideSpeed(RIGHT_SIDE, -1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                stopMotors(); prepareForwardMove(45.0); currentState = SHAPE_M_R1_LEG2; // Target distance was 65? Using 45 from M home turn 1
            } break; }
        case SHAPE_M_R1_WAIT_HOME: { break; }

        // --- Shape Home Return (Rover 1 - after N) States ---
        case SHAPE_H_N_R1_TURN1: {
             setSideSpeed(LEFT_SIDE, -1, TURN_SPEED);
             setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
             float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
             if (angleTurned >= targetTurnAngle - (TURN_ANGLE_THRESHOLD * 1.5)) {
                  stopMotors(); prepareForwardMove(30.0); currentState = SHAPE_H_N_R1_LEG1;
             } break; }
        case SHAPE_H_N_R1_TURN2: {
            setSideSpeed(LEFT_SIDE, -1, TURN_SPEED);
            setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                 stopMotors(); prepareForwardMove(105.0); currentState = SHAPE_H_N_R1_LEG2;
            } break; }

        // --- Shape Home Return (Rover 1 - after M) States ---
        case SHAPE_H_M_R1_TURN1: {
             setSideSpeed(LEFT_SIDE, -1, TURN_SPEED);
             setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
             float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
             if (angleTurned >= targetTurnAngle - (TURN_ANGLE_THRESHOLD * 1.5)) {
                 stopMotors(); prepareForwardMove(45.0); currentState = SHAPE_H_M_R1_LEG1; // Using 45 from M turn 1
             } break; }
        case SHAPE_H_M_R1_TURN2: {
             setSideSpeed(LEFT_SIDE, -1, TURN_SPEED);
             setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
             float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
             if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                 stopMotors(); prepareForwardMove(110.0); currentState = SHAPE_H_M_R1_LEG2; // Using 110 from M leg 1
             } break; }

    } // End switch

    // --- REMOVED: Send Encoder Data via ESP-NOW periodically ---
    // (Assuming not needed based on removing other prints)

    delay(1); // Keep small delay
}

// --- ESP-NOW Callbacks ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { /* No feedback */ }

// OnDataRecv: Handles incoming commands
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    struct_message_universal receivedMessage;
    if (len == sizeof(receivedMessage)) {
        memcpy(&receivedMessage, incomingData, sizeof(receivedMessage));
        if (receivedMessage.msg_type == COMMAND_MOVEMENT) {
            payload_movement_t movCmd = receivedMessage.payload.movement;

            // Handle stop 's' and reset 'x' anytime
            if (movCmd.command_type == 's') { stopMotors(); currentState = IDLE; lastShapeCommand = '\0'; return; }
            if (movCmd.command_type == 'x') { stopMotors(); ESP.restart(); } // Simple reset

            // Handle 'h' command only if waiting after a shape
            if (currentState == SHAPE_N_R1_WAIT_HOME || currentState == SHAPE_M_R1_WAIT_HOME) {
                if (movCmd.command_type == 'h') {
                    mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw;
                    targetTurnAngle = 180.0;
                    if (lastShapeCommand == 'n') { currentState = SHAPE_H_N_R1_TURN1; }
                    else if (lastShapeCommand == 'm') { currentState = SHAPE_H_M_R1_TURN1; }
                    else { stopMotors(); currentState = IDLE; } // Failsafe
                }
                return; // Ignore other commands while waiting
            }

            // Ignore other commands if not IDLE
            if (currentState != IDLE) { return; }

            // Process commands only when IDLE
            lastShapeCommand = '\0';
            switch(movCmd.command_type) {
                case 'l': {
                    if (movCmd.value > 0) {
                        mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw; targetTurnAngle = movCmd.value;
                        currentState = TURNING_LEFT;
                    } break; }
                case 'r': {
                    if (movCmd.value > 0) {
                        mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw; targetTurnAngle = movCmd.value;
                        currentState = TURNING_RIGHT;
                    } break; }
                case 'f': {
                    if (movCmd.value > 0) {
                        prepareForwardMove(movCmd.value);
                        currentState = MOVING_FORWARD;
                    } break; }
                case 't': {
                    if (movCmd.value == 1) { // Task 1
                        prepareForwardMove(150.0);
                        currentState = TASK1_LEG1;
                    } break; }
                case 'n': {
                    lastShapeCommand = 'n';
                    prepareForwardMove(105.0);
                    currentState = SHAPE_N_R1_LEG1;
                    break; }
                case 'm': {
                    lastShapeCommand = 'm';
                    prepareForwardMove(110.0); // Using 110 from M Home turn 2
                    currentState = SHAPE_M_R1_LEG1;
                    break; }
                case 'h': break; // Ignore 'h' when IDLE
                default: { stopMotors(); currentState = IDLE; } // Unknown command
            } // End switch
        } else if (receivedMessage.msg_type == COMMAND_PID_TUNING) {
             payload_pid_tuning_t tuningCmd = receivedMessage.payload.tuning;
             Kp_pid = tuningCmd.kp; Ki_pid = tuningCmd.ki; Kd_pid = tuningCmd.kd;
             headingPID.SetTunings(Kp_pid, Ki_pid, Kd_pid); // Apply new tunings
        }
    }
}


// --- ISR ---
void IRAM_ATTR handleEncoderInterrupt(void *arg) {
    int i = (int)arg;
    if (i < 0 || i >= NUM_ENCODERS) return;
    portENTER_CRITICAL_ISR(&encoderMutex[i]);
    bool pinB_state = digitalRead(encoderPins[i][1]);
    if (pinB_state) { encoderCounts[i]++; } else { encoderCounts[i]--; }
    portEXIT_CRITICAL_ISR(&encoderMutex[i]);
}


// --- Motor Control ---
// direction: 1=Forward, -1=Backward, 0=Brake
void setSideSpeed(int side, int direction, int speed) {
    int enPin, in1Pin, in2Pin, pwmChannel;
    if (side == LEFT_SIDE) { enPin = ENA_L_PIN; in1Pin = IN1_L_PIN; in2Pin = IN2_L_PIN; pwmChannel = ledcChannels[LEFT_SIDE]; }
    else if (side == RIGHT_SIDE) { enPin = ENA_R_PIN; in1Pin = IN3_R_PIN; in2Pin = IN4_R_PIN; pwmChannel = ledcChannels[RIGHT_SIDE]; }
    else { return; }
    int pwmValue = constrain(speed, 0, MAX_PWM_VALUE);
    if (direction == 1) { digitalWrite(in1Pin, HIGH); digitalWrite(in2Pin, LOW); ledcWrite(pwmChannel, pwmValue); }
    else if (direction == -1) { digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, HIGH); ledcWrite(pwmChannel, pwmValue); }
    else { digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, LOW); ledcWrite(pwmChannel, MAX_PWM_VALUE); } // Brake
}

// --- Helper for Angle Difference ---
float calculateAngleDifference(float targetAngle, float currentAngle) {
    float diff = targetAngle - currentAngle;
    while (diff <= -180.0f) diff += 360.0f;
    while (diff > 180.0f) diff -= 360.0f;
    return diff;
}

// --- Helper to Stop Motors ---
void stopMotors() {
    setSideSpeed(LEFT_SIDE, 0, 0); // Brake Left
    setSideSpeed(RIGHT_SIDE, 0, 0); // Brake Right
}

// --- Helper function to prepare for a forward move (cm) ---
void prepareForwardMove(float distanceCm) {
    targetEncoderTicks = (long)((distanceCm * COUNTS_PER_CM) + 0.5f);
    mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw;
    portENTER_CRITICAL(&encoderMutex[ENC_LF]); startEncoderTicks[ENC_LF] = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]);
    portENTER_CRITICAL(&encoderMutex[ENC_RF]); startEncoderTicks[ENC_RF] = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]);
    headingPID.SetMode(AUTOMATIC);
    setSideSpeed(LEFT_SIDE, 1, FWD_SPEED_BASE); // Start moving
    setSideSpeed(RIGHT_SIDE, 1, FWD_SPEED_BASE);
}

// --- END: Rover ESP32 Code ---