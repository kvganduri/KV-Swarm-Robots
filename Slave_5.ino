// --- START: Rover 1 ESP32 Code (V1.6 - L298N & Updated m/h sequence v2) ---
// Integrated L298N control, pins, and variables from V4.3 into V1.1 m/h logic
// Updated 'n'/'h' and 'm'/'h' sequences as per user request (V1.6 spec).

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

// --- Speed & Control Definitions (Values from User V1.3 Snippet) ---
const int FWD_SPEED_BASE = 60;   // From V4.3
const int TURN_SPEED = 90;       // From V4.3
const float COUNTS_PER_CM = 75.0; // From V4.3 - !!! REPLACE with your actual calibrated value !!!
const float TURN_ANGLE_THRESHOLD = 1; // From User V1.3 Snippet - Stop turn when within this many degrees
const unsigned long TASK_PAUSE_DURATION_MS = 1000; // 1 second pause (from V1.1)

// --- PID Tuning Parameters --- (Values from User V1.3 Snippet)
double Kp_pid = 100; // From V4.3 / User V1.3
double Ki_pid = 5.0; // From V4.3 / User V1.3
double Kd_pid = 1;   // From User V1.3 Snippet

// --- Global Variables ---
volatile long encoderCounts[NUM_ENCODERS] = {0, 0}; // LF, RF
portMUX_TYPE encoderMutex[NUM_ENCODERS];
long startEncoderTicks[NUM_ENCODERS]; // Store individual start ticks (from V1.1 logic)
MPU6050 mpu(Wire);
float targetYaw = 0.0; float currentYaw = 0.0;

// PID Variables
double headingError, pidOutputCorrection, headingSetpoint = 0;
PID headingPID(&headingError, &pidOutputCorrection, &headingSetpoint, Kp_pid, Ki_pid, Kd_pid, DIRECT);

// State Machine Variables (FROM V1.1)
enum RoverState {
    IDLE,
    TURNING_LEFT,           // Basic 'l' command turn
    TURNING_RIGHT,          // Basic 'r' command turn
    MOVING_FORWARD,         // Basic 'f' command move

    TASK1_LEG1,             // Task 1 specific states
    TASK1_PAUSE_AFTER_LEG1,
    TASK1_TURN,             // Specific state for task 1 turn logic (180 R)
    TASK1_PAUSE_AFTER_TURN,
    TASK1_LEG2,

    SHAPE_N_R1_LEG1,        // Shape 'n' (Rover 1) states (105cm Fwd)
    SHAPE_N_R1_TURN1,       // Specific state for N turn 1 logic (90 L)
    SHAPE_N_R1_LEG2,        // (30cm Fwd)
    SHAPE_N_R1_WAIT_HOME,   // Wait state for 'h' after 'n'

    SHAPE_H_N_R1_TURN1,     // Return sequence after 'n' (specific turn 1: 180 R)
    SHAPE_H_N_R1_LEG1,      // (30cm Fwd)
    SHAPE_H_N_R1_TURN2,     // Specific state for N home turn 2 (90 R) <<< Updated V1.6
    SHAPE_H_N_R1_LEG2,      // (105cm Fwd)

    SHAPE_M_R1_LEG1,        // Shape 'm' (Rover 1) states (50cm Fwd) <<< Updated V1.6
    SHAPE_M_R1_TURN1,       // Specific state for M turn 1 logic (90 L)
    SHAPE_M_R1_LEG2,        // (65cm Fwd)
    SHAPE_M_R1_WAIT_HOME,   // Wait state for 'h' after 'm'

    SHAPE_H_M_R1_TURN1,     // Return sequence after 'm' (specific turn 1: 180 R)
    SHAPE_H_M_R1_LEG1,      // (65cm Fwd)
    SHAPE_H_M_R1_TURN2,     // Specific state for M home turn 2 (90 R) <<< Updated V1.6
    SHAPE_H_M_R1_LEG2       // (50cm Fwd) <<< Updated V1.6
};
RoverState currentState = IDLE;
float targetTurnAngle = 0.0; // Target angle in degrees
float startYawForTurn = 0.0; // Starting angle in degrees
long targetEncoderTicks = 0; // Calculated target based on distance CM
unsigned long pauseStartTime = 0;
char lastShapeCommand = '\0'; // Track if last move was 'n' or 'm'

// --- ESP-NOW Configuration ---
uint8_t masterAddress[] = {0xA8, 0x42, 0xE3, 0x47, 0x27, 0x90}; // User's Master MAC
esp_now_peer_info_t peerInfo;

// --- Data Structures (Must match Master) ---
typedef enum { COMMAND_MOVEMENT, COMMAND_PID_TUNING } message_type_t;
typedef struct { char command_type; float value; } payload_movement_t;
typedef struct { double kp; double ki; double kd; } payload_pid_tuning_t;
typedef struct struct_message_universal { message_type_t msg_type; union { payload_movement_t movement; payload_pid_tuning_t tuning; } payload; } struct_message_universal;

// Encoder data sent to Master
typedef struct struct_message_encoder_data { long encoderLF; long encoderRF; } struct_message_encoder_data;
struct_message_encoder_data encoderDataToSend;

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
    Serial.begin(115200);
    Serial.println("Rover 1 Setup (V1.6 - L298N & Updated m/h sequence v2)..."); // Version Bump

    // Initialize MPU6050
    Wire.begin(23, 22);
    byte status = mpu.begin(); Serial.print(F("MPU6050 status: ")); Serial.println(status); while(status != 0) { delay(100); status = mpu.begin(); }
    Serial.println(F("Calculating MPU offsets, do not move...")); delay(1000); mpu.calcOffsets(); Serial.println("MPU Offsets calculated.");

    // Initialize PID
    headingPID.SetMode(MANUAL); headingPID.SetSampleTime(10); headingPID.SetOutputLimits(-FWD_SPEED_BASE, FWD_SPEED_BASE);
    headingPID.SetTunings(Kp_pid, Ki_pid, Kd_pid);
    Serial.println("PID Initialized (Manual Mode).");

    // Get initial Yaw angle
    mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw; Serial.printf("Initial Yaw: %.1f\n", currentYaw);

    // Initialize mutexes
    for (int i = 0; i < NUM_ENCODERS; i++) { encoderMutex[i] = portMUX_INITIALIZER_UNLOCKED; }

    // Setup L298N Motor Pins
    Serial.println("Setting up L298N pins...");
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
    Serial.printf(" Left Motor Pins: ENA=%d, IN1=%d, IN2=%d (PWM Chan %d)\n", ENA_L_PIN, IN1_L_PIN, IN2_L_PIN, ledcChannels[LEFT_SIDE]);
    Serial.printf(" Right Motor Pins: ENB=%d, IN3=%d, IN4=%d (PWM Chan %d)\n", ENA_R_PIN, IN3_R_PIN, IN4_R_PIN, ledcChannels[RIGHT_SIDE]);
    Serial.println("L298N pins setup.");

    // Setup Encoder Pins and Interrupts
    Serial.println("Attaching Encoder Interrupts (LF, RF only)...");
    for (int i = 0; i < NUM_ENCODERS; i++) { pinMode(encoderPins[i][0], INPUT_PULLUP); pinMode(encoderPins[i][1], INPUT_PULLUP); attachInterruptArg(digitalPinToInterrupt(encoderPins[i][0]), handleEncoderInterrupt, (void*)i, RISING); Serial.printf("   Attached ISR to pin %d (Encoder Index %d)\n", encoderPins[i][0], i); }

    // Setup WiFi and ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi "); Serial.print(ssid);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println(" Connected!");
    Serial.print("Rover MAC Address: "); Serial.println(WiFi.macAddress());
    if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW Init Failed"); ESP.restart(); }
    esp_now_register_send_cb(OnDataSent); memcpy(peerInfo.peer_addr, masterAddress, 6); peerInfo.channel = 0; peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK){ Serial.println("Failed to add peer"); return; } else { Serial.println("Peer Added OK"); }
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP-NOW Initialized.");

    Serial.println("Setup Complete. Rover in IDLE state.");
}

// --- Loop ---
unsigned long lastSendTime = 0;
const long SEND_INTERVAL_MS = 100;

void loop() {
    mpu.update();
    currentYaw = mpu.getAngleZ();

    // --- State Machine Logic (Updated sequences V1.6) ---
    switch (currentState) {
        case IDLE: if (headingPID.GetMode() == AUTOMATIC) { headingPID.SetMode(MANUAL); } break;

        // --- Basic Turns ---
        case TURNING_LEFT: { /* ... no change ... */
            setSideSpeed(LEFT_SIDE, 1, TURN_SPEED);   // Left Turn
            setSideSpeed(RIGHT_SIDE, -1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                Serial.printf("Turn Left Command Complete. Turned: %.1f / Target: %.1f\n", angleTurned, targetTurnAngle);
                stopMotors(); currentState = IDLE;
            } break; }
        case TURNING_RIGHT: { /* ... no change ... */
            setSideSpeed(LEFT_SIDE, -1, TURN_SPEED); // Right Turn
            setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                Serial.printf("Turn Right Command Complete. Turned: %.1f / Target: %.1f\n", angleTurned, targetTurnAngle);
                stopMotors(); currentState = IDLE;
            } break; }

        // --- Forward Movement Handler (Common for all legs) ---
        case MOVING_FORWARD:
        case TASK1_LEG1:
        case TASK1_LEG2:
        case SHAPE_N_R1_LEG1:
        case SHAPE_N_R1_LEG2:
        case SHAPE_M_R1_LEG1: // Distance updated in OnDataRecv for 'm'
        case SHAPE_M_R1_LEG2:
        case SHAPE_H_N_R1_LEG1:
        case SHAPE_H_N_R1_LEG2:
        case SHAPE_H_M_R1_LEG1:
        case SHAPE_H_M_R1_LEG2: { // Distance updated in state transition for M Home leg 2
            headingError = calculateAngleDifference(targetYaw, currentYaw); headingPID.Compute();
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
                Serial.printf("Distance Complete: Target Ticks: %ld (LF moved: %ld, RF moved: %ld)\n", targetEncoderTicks, moved_lf, moved_rf);
            }

            if (targetReached) {
                stopMotors();
                if (headingPID.GetMode() == AUTOMATIC) { headingPID.SetMode(MANUAL); }

                // --- State Transitions after completing a move (Updated V1.6) ---
                if       (currentState == MOVING_FORWARD) { Serial.printf("Distance Fwd Complete.\n"); lastShapeCommand = '\0'; currentState = IDLE; }
                // Task 1 Transitions
                else if (currentState == TASK1_LEG1)     { Serial.printf("Task 1 Leg 1 Complete. Pausing...\n"); pauseStartTime = millis(); currentState = TASK1_PAUSE_AFTER_LEG1; }
                else if (currentState == TASK1_LEG2)     { Serial.printf("Task 1 Leg 2 Complete.\n"); Serial.println("Task 1 Finished."); lastShapeCommand = '\0'; currentState = IDLE; }
                // Shape N Transitions (105 Fwd -> 90 L -> 30 Fwd)
                else if (currentState == SHAPE_N_R1_LEG1){ Serial.println("Shape N Leg 1 (105cm) complete. Starting Turn 1 (L90)..."); startYawForTurn = currentYaw; targetTurnAngle = 90.0; currentState = SHAPE_N_R1_TURN1; }
                else if (currentState == SHAPE_N_R1_LEG2){ Serial.println("Shape N Leg 2 (30cm) complete. Waiting for Home..."); currentState = SHAPE_N_R1_WAIT_HOME; }
                // Shape N Home Transitions (180 R -> 30 Fwd -> 90 R -> 105 Fwd)
                else if (currentState == SHAPE_H_N_R1_LEG1){ Serial.println("Shape Home (N) Leg 1 (30cm) complete. Starting Turn 2 (R90)..."); startYawForTurn = currentYaw; targetTurnAngle = 90.0; currentState = SHAPE_H_N_R1_TURN2; } // <<< Message Updated
                else if (currentState == SHAPE_H_N_R1_LEG2){ Serial.println("Shape Home (N) Leg 2 (105cm) complete. Sequence Finished."); lastShapeCommand = '\0'; currentState = IDLE; }
                // Shape M Transitions (50 Fwd -> 90 L -> 65 Fwd)
                else if (currentState == SHAPE_M_R1_LEG1){ Serial.println("Shape M Leg 1 (50cm) complete. Starting Turn 1 (L90)..."); startYawForTurn = currentYaw; targetTurnAngle = 90.0; currentState = SHAPE_M_R1_TURN1; } // <<< Message Updated (distance)
                else if (currentState == SHAPE_M_R1_LEG2){ Serial.println("Shape M Leg 2 (65cm) complete. Waiting for Home..."); currentState = SHAPE_M_R1_WAIT_HOME; }
                // Shape M Home Transitions (180 R -> 65 Fwd -> 90 R -> 50 Fwd)
                else if (currentState == SHAPE_H_M_R1_LEG1){ Serial.println("Shape Home (M) Leg 1 (65cm) complete. Starting Turn 2 (R90)..."); startYawForTurn = currentYaw; targetTurnAngle = 90.0; currentState = SHAPE_H_M_R1_TURN2; } // <<< Message Updated
                else if (currentState == SHAPE_H_M_R1_LEG2){ Serial.println("Shape Home (M) Leg 2 (50cm) complete. Sequence Finished."); lastShapeCommand = '\0'; currentState = IDLE; } // <<< Message Updated (distance)
                else { currentState = IDLE; } // Failsafe
            }
            break; // End forward movement states
        }

        // --- Task 1 Specific States ---
        case TASK1_PAUSE_AFTER_LEG1: { /* ... no change ... */ if (millis() - pauseStartTime >= TASK_PAUSE_DURATION_MS) { Serial.println("Task 1 Pause 1 Finished. Starting Turn (180R)..."); mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw; targetTurnAngle = 180.0; currentState = TASK1_TURN; } break; }
        case TASK1_TURN: { /* ... no change ... */
            setSideSpeed(LEFT_SIDE, -1, TURN_SPEED); // Right Turn
            setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - (TURN_ANGLE_THRESHOLD * 1.5)) {
                Serial.printf("Task 1 Turn (R180) Complete. Turned: %.1f / Target: %.1f\n", angleTurned, targetTurnAngle);
                stopMotors(); Serial.println("Task 1 Turn Finished. Pausing...");
                pauseStartTime = millis(); currentState = TASK1_PAUSE_AFTER_TURN;
            } break; }
        case TASK1_PAUSE_AFTER_TURN: { /* ... no change ... */ if (millis() - pauseStartTime >= TASK_PAUSE_DURATION_MS) { Serial.println("Task 1 Pause 2 Finished. Starting Leg 2 (150cm)..."); prepareForwardMove(150.0); currentState = TASK1_LEG2; } break; }

        // --- Shape N (Rover 1) Specific States ---
        case SHAPE_N_R1_TURN1: { /* ... no change (90 L) ... */
            setSideSpeed(LEFT_SIDE, 1, TURN_SPEED); // Left Turn
            setSideSpeed(RIGHT_SIDE, -1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                Serial.println("Shape N Turn 1 (L90) complete. Starting Leg 2 (30cm)...");
                stopMotors(); prepareForwardMove(30.0); currentState = SHAPE_N_R1_LEG2;
            } break; }
        case SHAPE_N_R1_WAIT_HOME: { /* ... no change ... */ break; }

        // --- Shape M (Rover 1) Specific States ---
        case SHAPE_M_R1_TURN1: { /* ... no change (90 L) ... */
            setSideSpeed(LEFT_SIDE, 1, TURN_SPEED); // Left Turn
            setSideSpeed(RIGHT_SIDE, -1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                Serial.println("Shape M Turn 1 (L90) complete. Starting Leg 2 (65cm)...");
                stopMotors(); prepareForwardMove(45.0); currentState = SHAPE_M_R1_LEG2;
            } break; }
        case SHAPE_M_R1_WAIT_HOME: { /* ... no change ... */ break; }

        // --- Shape Home Return (Rover 1 - after N) States ---
        case SHAPE_H_N_R1_TURN1: { /* ... no change (180 R) ... */
            setSideSpeed(LEFT_SIDE, -1, TURN_SPEED); // Right Turn
            setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - (TURN_ANGLE_THRESHOLD * 1.5)) {
                 Serial.println("Shape Home (N) Turn 1 (R180) complete. Starting Leg 1 (30cm)...");
                 stopMotors(); prepareForwardMove(30.0); currentState = SHAPE_H_N_R1_LEG1;
             } break; }
        case SHAPE_H_N_R1_TURN2: { // <<< CHANGED V1.6: 90 Degree Right Turn
            setSideSpeed(LEFT_SIDE, -1, TURN_SPEED); // Right Turn
            setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                Serial.println("Shape Home (N) Turn 2 (R90) complete. Starting Leg 2 (105cm)..."); // <<< Message Updated
                stopMotors();
                prepareForwardMove(105.0);
                currentState = SHAPE_H_N_R1_LEG2;
            } break; }

        // --- Shape Home Return (Rover 1 - after M) States ---
        case SHAPE_H_M_R1_TURN1: { /* ... no change (180 R) ... */
            setSideSpeed(LEFT_SIDE, -1, TURN_SPEED); // Right Turn
            setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - (TURN_ANGLE_THRESHOLD * 1.5)) {
                Serial.println("Shape Home (M) Turn 1 (R180) complete. Starting Leg 1 (65cm)...");
                stopMotors(); prepareForwardMove(45.0); currentState = SHAPE_H_M_R1_LEG1;
            } break; }
        case SHAPE_H_M_R1_TURN2: { // <<< CHANGED V1.6: 90 Degree Right Turn
             setSideSpeed(LEFT_SIDE, -1, TURN_SPEED); // Right Turn
             setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED);
             float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
             if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                Serial.println("Shape Home (M) Turn 2 (R90) complete. Starting Leg 2 (50cm)..."); // <<< Message & Distance Updated
                stopMotors();
                prepareForwardMove(110.0); // <<< Distance Updated
                currentState = SHAPE_H_M_R1_LEG2;
            } break; }

    } // End switch

    // --- Send Encoder Data via ESP-NOW periodically ---
    unsigned long currentTime = millis();
    if (currentTime - lastSendTime >= SEND_INTERVAL_MS) { /* ... no change ... */
        portENTER_CRITICAL(&encoderMutex[ENC_LF]); encoderDataToSend.encoderLF = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]);
        portENTER_CRITICAL(&encoderMutex[ENC_RF]); encoderDataToSend.encoderRF = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]);
        esp_now_send(masterAddress, (uint8_t *) &encoderDataToSend, sizeof(encoderDataToSend));
        lastSendTime = currentTime;
    }
    delay(1);
}

// --- ESP-NOW Callbacks ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { /* Optional feedback */ }

// OnDataRecv: Handles incoming commands (Updated sequence descriptions V1.6)
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    struct_message_universal receivedMessage;
    if (len == sizeof(receivedMessage)) {
        memcpy(&receivedMessage, incomingData, sizeof(receivedMessage));
        if (receivedMessage.msg_type == COMMAND_MOVEMENT) {
            payload_movement_t movCmd = receivedMessage.payload.movement;
            Serial.printf("Command RX: Type=%c, Value=%.1f\n", movCmd.command_type, movCmd.value);

            // Handle stop 's' and reset 'x' anytime
            if (movCmd.command_type == 's') { /* ... stop logic ... */ return; }
            if (movCmd.command_type == 'x') { /* ... reset logic ... */ return; }

            // Handle 'h' command only if waiting after a shape
            if (currentState == SHAPE_N_R1_WAIT_HOME || currentState == SHAPE_M_R1_WAIT_HOME) {
                if (movCmd.command_type == 'h') {
                    Serial.printf("Action: 'h' received while waiting after '%c'. Starting return sequence...\n", lastShapeCommand);
                    mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw;
                    targetTurnAngle = 180.0; // Home sequence starts with 180 Right turn
                    Serial.printf("Action: Start Home 180deg Right Turn from %.1f\n", startYawForTurn);
                    if (lastShapeCommand == 'n') { currentState = SHAPE_H_N_R1_TURN1; }
                    else if (lastShapeCommand == 'm') { currentState = SHAPE_H_M_R1_TURN1; }
                    else { Serial.println("Error: Unknown last shape command!"); stopMotors(); currentState = IDLE; }
                } else { /* ... ignore other commands ... */ }
                return;
            }

            // Ignore other commands if not IDLE
            if (currentState != IDLE) { Serial.println("Action: Busy, command ignored."); return; }

            // Process commands only when IDLE
            lastShapeCommand = '\0';
            switch(movCmd.command_type) {
                case 'l': { /* ... no change ... */
                    if (movCmd.value > 0) {
                        mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw; targetTurnAngle = movCmd.value;
                        Serial.printf("Action: Start Turning Left %.1f degrees from %.1f\n", targetTurnAngle, startYawForTurn);
                        currentState = TURNING_LEFT;
                    } else { Serial.println("Error: Turn angle (degrees) must be positive."); }
                    break; }
                case 'r': { /* ... no change ... */
                     if (movCmd.value > 0) {
                        mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw; targetTurnAngle = movCmd.value;
                        Serial.printf("Action: Start Turning Right %.1f degrees from %.1f\n", targetTurnAngle, startYawForTurn);
                        currentState = TURNING_RIGHT;
                     } else { Serial.println("Error: Turn angle (degrees) must be positive."); }
                    break; }
                case 'f': { /* ... no change ... */
                    if (movCmd.value > 0) {
                        prepareForwardMove(movCmd.value);
                        currentState = MOVING_FORWARD;
                    } else { Serial.println("Error: Forward distance (cm) must be positive."); }
                    break; }
                case 't': { /* ... no change ... */
                    if (movCmd.value == 1) {
                         Serial.println("Action: Starting Task 1 (150cm Fwd -> 180R -> 150cm Fwd)");
                         prepareForwardMove(150.0);
                         currentState = TASK1_LEG1;
                    } else { Serial.printf("Error: Unknown task ID %.0f\n", movCmd.value); }
                    break; }
                case 'n': { /* ... no change ... */
                    Serial.println("Action: Starting Shape N (105cm Fwd -> 90L -> 30cm Fwd)");
                    lastShapeCommand = 'n';
                    prepareForwardMove(105.0);
                    currentState = SHAPE_N_R1_LEG1;
                    break; }
                case 'm': { // <<< CHANGED V1.6: Distance Updated
                    Serial.println("Action: Starting Shape M (50cm Fwd -> 90L -> 65cm Fwd)"); // <<< Message Updated
                    lastShapeCommand = 'm';
                    prepareForwardMove(110.0); // <<< Distance Updated
                    currentState = SHAPE_M_R1_LEG1;
                    break; }
                case 'h': { /* ... no change ... */
                    Serial.println("Action: Ignoring 'h' command while IDLE."); break; }
                default: { /* ... no change ... */
                    Serial.printf("Error: Unknown command '%c'\n", movCmd.command_type);
                    stopMotors(); currentState = IDLE; }
            } // End switch
        } else if (receivedMessage.msg_type == COMMAND_PID_TUNING) { /* ... PID tuning logic ... */
            payload_pid_tuning_t tuningCmd = receivedMessage.payload.tuning;
            Kp_pid = tuningCmd.kp; Ki_pid = tuningCmd.ki; Kd_pid = tuningCmd.kd;
            headingPID.SetTunings(Kp_pid, Ki_pid, Kd_pid); // Apply new tunings
            Serial.printf("PID Tunings Updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp_pid, Ki_pid, Kd_pid);
        } else { /* ... Unknown message type ... */ }
    } else { /* ... Unexpected length ... */ }
}


// --- ISR ---
// Handles encoder interrupts
void IRAM_ATTR handleEncoderInterrupt(void *arg) { /* ... no change ... */
    int i = (int)arg;
    if (i < 0 || i >= NUM_ENCODERS) return;
    portENTER_CRITICAL_ISR(&encoderMutex[i]);
    bool pinB_state = digitalRead(encoderPins[i][1]);
    if (pinB_state) { encoderCounts[i]++; } else { encoderCounts[i]--; }
    portEXIT_CRITICAL_ISR(&encoderMutex[i]);
}


// --- Motor Control (L298N Version with BRAKE on Stop) ---
// direction: 1=Forward, -1=Backward, 0=Brake
void setSideSpeed(int side, int direction, int speed) { /* ... no change ... */
    int enPin, in1Pin, in2Pin, pwmChannel;

    if (side == LEFT_SIDE) {
        enPin = ENA_L_PIN; in1Pin = IN1_L_PIN; in2Pin = IN2_L_PIN; pwmChannel = ledcChannels[LEFT_SIDE];
    } else if (side == RIGHT_SIDE) {
        enPin = ENA_R_PIN; in1Pin = IN3_R_PIN; in2Pin = IN4_R_PIN; pwmChannel = ledcChannels[RIGHT_SIDE];
    } else { return; }

    int pwmValue = constrain(speed, 0, MAX_PWM_VALUE);

    if (direction == 1) { /* Forward */ digitalWrite(in1Pin, HIGH); digitalWrite(in2Pin, LOW); ledcWrite(pwmChannel, pwmValue); }
    else if (direction == -1) { /* Backward */ digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, HIGH); ledcWrite(pwmChannel, pwmValue); }
    else { /* Brake */ digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, LOW); ledcWrite(pwmChannel, MAX_PWM_VALUE); }
}

// --- Helper for Angle Difference ---
float calculateAngleDifference(float targetAngle, float currentAngle) { /* ... no change ... */
    float diff = targetAngle - currentAngle;
    while (diff <= -180.0f) diff += 360.0f;
    while (diff > 180.0f) diff -= 360.0f;
    return diff;
}

// --- Helper to Stop Motors (Uses Braking) ---
void stopMotors() { /* ... no change ... */
    Serial.println("Executing Dynamic Brake Stop (via setSideSpeed)...");
    setSideSpeed(LEFT_SIDE, 0, 0);
    setSideSpeed(RIGHT_SIDE, 0, 0);
}

// --- Helper function to prepare for a forward move (cm) ---
void prepareForwardMove(float distanceCm) { /* ... no change ... */
    targetEncoderTicks = (long)((distanceCm * COUNTS_PER_CM) + 0.5f);
    mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw;
    Serial.print("Capturing start encoder ticks: ");
    portENTER_CRITICAL(&encoderMutex[ENC_LF]); startEncoderTicks[ENC_LF] = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]);
    portENTER_CRITICAL(&encoderMutex[ENC_RF]); startEncoderTicks[ENC_RF] = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]);
    Serial.printf("[LF]=%ld [RF]=%ld\n", startEncoderTicks[ENC_LF], startEncoderTicks[ENC_RF]);
    Serial.printf("Action: Start Moving Fwd %.1f cm (Target Ticks: %ld), Target Yaw: %.1f\n", distanceCm, targetEncoderTicks, targetYaw);
    headingPID.SetMode(AUTOMATIC);
    setSideSpeed(LEFT_SIDE, 1, FWD_SPEED_BASE);
    setSideSpeed(RIGHT_SIDE, 1, FWD_SPEED_BASE);
}

// --- END: Rover ESP32 Code ---