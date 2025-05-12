// --- START: Rover ESP32 Code (V4.14 - Shape Home Logic) ---

#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h> // Using MPU6050_light library
#include <PID_v1.h>       // Include PID library
#include <cmath>          // For trig functions (cos, sin, atan2, sqrt, pow)

// --- Network Config (Added for WiFi connection) ---
const char *ssid = "ROVER_CONTROL"; // Must match Master AP SSID
const char *password = "12345678";   // Must match Master AP password

// --- Pin Definitions (VERIFY THESE!) ---
const int leftMotorPins[] = {27, 26}; // Left: {PIN1, PIN2}
const int rightMotorPins[] = {25, 33}; // Right: {PIN1, PIN2}

// Front Encoders Only (LF = Index 0, RF = Index 1)
#define ENC_LF 0
#define ENC_RF 1
const int NUM_ENCODERS = 2;
const int encoderPins[NUM_ENCODERS][2] = { {14, 12}, {13, 16} }; // LF[A,B], RF[A,B]

#define LEFT_SIDE 0
#define RIGHT_SIDE 1
const int NUM_SIDES = 2;
const int PWM_FREQ = 5000; const int PWM_RESOLUTION = 8; const int MAX_PWM_VALUE = 255;
const int ledcChannels[] = {0, 1};

// --- Speed & Control Definitions ---
const int FWD_SPEED_BASE = 50;
const int TURN_SPEED = 50;
const float COUNTS_PER_CM = 75.0; // !!! REPLACE with your value !!!
const float TURN_ANGLE_THRESHOLD = 1.3;
const unsigned long TASK_PAUSE_DURATION_MS = 1000;

// --- PID Tuning Parameters ---
double Kp_pid = 115.0;
double Ki_pid = 4.6;
double Kd_pid = 0.05;

// --- Global Variables ---
volatile long encoderCounts[NUM_ENCODERS] = {0, 0}; // LF, RF
portMUX_TYPE encoderMutex[NUM_ENCODERS];
long startEncoderTicks[NUM_ENCODERS]; // Store individual start ticks
MPU6050 mpu(Wire);
float targetYaw = 0.0; float currentYaw = 0.0;

// PID Variables
double headingError, pidOutputCorrection, headingSetpoint = 0;
PID headingPID(&headingError, &pidOutputCorrection, &headingSetpoint, Kp_pid, Ki_pid, Kd_pid, DIRECT);

// State Machine Variables
// *** ADDED Shape/Home specific states ***
enum RoverState { IDLE, TURNING_LEFT, TURNING_RIGHT, MOVING_FORWARD,
                  TASK1_LEG1, TASK1_PAUSE_AFTER_LEG1, TASK1_TURN, TASK1_PAUSE_AFTER_TURN, TASK1_LEG2,
                  WAITING_HOME, EXECUTING_HOME_TURN, EXECUTING_HOME_MOVE }; // Added states
RoverState currentState = IDLE;
float targetTurnAngle = 0.0; float startYawForTurn = 0.0;
long targetEncoderTicks = 0;
unsigned long pauseStartTime = 0;
char lastShapeCommand = '\0'; // Track if last move was 'n' or 'm'
float homeReturnDistanceCm = 0.0; // Distance to move after home turn

// --- ESP-NOW Configuration ---
uint8_t masterAddress[] = {0xA8, 0x42, 0xE3, 0x47, 0x27, 0x90}; // User's Master MAC
esp_now_peer_info_t peerInfo;

// --- Data Structures (Must match Master sending this) ---
typedef enum { COMMAND_MOVEMENT, COMMAND_PID_TUNING } message_type_t;
typedef struct { char command_type; float value; } payload_movement_t;
typedef struct { double kp; double ki; double kd; } payload_pid_tuning_t;
typedef struct struct_message_universal { message_type_t msg_type; union { payload_movement_t movement; payload_pid_tuning_t tuning; } payload; } struct_message_universal;

// Encoder data sent to Master (LF/RF only)
typedef struct struct_message_encoder_data { long encoderLF; long encoderRF; } struct_message_encoder_data;
struct_message_encoder_data encoderDataToSend;

// --- Function Prototypes ---
void setSideSpeed(int side, int direction, int speed);
void IRAM_ATTR handleEncoderInterrupt(void *arg);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
float calculateAngleDifference(float targetAngle, float currentAngle);
void stopMotors();

// --- Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("Rover Setup (Main V4.14 - Shape Home Logic)..."); // Version Bump

    // Initialize MPU6050
    Wire.begin(23, 22); byte status = mpu.begin(); Serial.print(F("MPU6050 status: ")); Serial.println(status); while(status != 0) { delay(100); status = mpu.begin(); }
    Serial.println(F("Calculating MPU offsets, do not move...")); delay(1000); mpu.calcOffsets(); Serial.println("MPU Offsets calculated.");

    // Initialize PID
    headingPID.SetMode(MANUAL); headingPID.SetSampleTime(10); headingPID.SetOutputLimits(-FWD_SPEED_BASE, FWD_SPEED_BASE); headingPID.SetTunings(Kp_pid, Ki_pid, Kd_pid); Serial.println("PID Initialized (Manual Mode).");

    // Get initial Yaw angle
    mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw; Serial.printf("Initial Yaw: %.1f\n", currentYaw);

    // Initialize mutexes, Motors, Encoders
    for (int i = 0; i < NUM_ENCODERS; i++) { encoderMutex[i] = portMUX_INITIALIZER_UNLOCKED; }
    pinMode(leftMotorPins[0], OUTPUT); pinMode(leftMotorPins[1], OUTPUT); digitalWrite(leftMotorPins[0], LOW); digitalWrite(leftMotorPins[1], LOW); ledcSetup(ledcChannels[LEFT_SIDE], PWM_FREQ, PWM_RESOLUTION);
    pinMode(rightMotorPins[0], OUTPUT); pinMode(rightMotorPins[1], OUTPUT); digitalWrite(rightMotorPins[0], LOW); digitalWrite(rightMotorPins[1], LOW); ledcSetup(ledcChannels[RIGHT_SIDE], PWM_FREQ, PWM_RESOLUTION);
    Serial.println("Attaching Encoder Interrupts (LF, RF only)...");
    for (int i = 0; i < NUM_ENCODERS; i++) { pinMode(encoderPins[i][0], INPUT_PULLUP); pinMode(encoderPins[i][1], INPUT_PULLUP); attachInterruptArg(digitalPinToInterrupt(encoderPins[i][0]), handleEncoderInterrupt, (void*)i, RISING); Serial.printf("   Attached ISR to pin %d (Encoder Index %d)\n", encoderPins[i][0], i); }

    // Setup WiFi and ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password); // Connect to Master AP
    Serial.print("Connecting to WiFi "); Serial.print(ssid);
    int connect_timeout = 20; // Wait 10 seconds max
    while (WiFi.status() != WL_CONNECTED && connect_timeout > 0) { delay(500); Serial.print("."); connect_timeout--; }
    if (WiFi.status() == WL_CONNECTED) { Serial.println(" Connected!"); } else { Serial.println(" Connection Failed!"); }
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

    // --- State Machine Logic ---
    switch (currentState) {
        case IDLE: if (headingPID.GetMode() == AUTOMATIC) { headingPID.SetMode(MANUAL); } break;

        case TURNING_LEFT:
        case TURNING_RIGHT: { // Handles 'l' or 'r' commands
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                Serial.printf("Turn Command Complete. Turned: %.1f / Target: %.1f\n", angleTurned, targetTurnAngle);
                stopMotors(); currentState = IDLE;
            } break; }

        case MOVING_FORWARD: // Handles 'f', 'n', 'm' command
        case TASK1_LEG1:     // Handles Task 1, Leg 1
        case TASK1_LEG2: {   // Handles Task 1, Leg 2
            headingError = calculateAngleDifference(targetYaw, currentYaw); headingPID.Compute();
            int leftSpeed = FWD_SPEED_BASE + (int)pidOutputCorrection; int rightSpeed = FWD_SPEED_BASE - (int)pidOutputCorrection;
            leftSpeed = constrain(leftSpeed, 0, MAX_PWM_VALUE); rightSpeed = constrain(rightSpeed, 0, MAX_PWM_VALUE);
            setSideSpeed(LEFT_SIDE, 1, leftSpeed); setSideSpeed(RIGHT_SIDE, 1, rightSpeed);

            // Check Distance Target based on LF OR RF encoder
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
                stopMotors(); if (headingPID.GetMode() == AUTOMATIC) { headingPID.SetMode(MANUAL); }
                // *** MODIFIED: Check if completing a shape leg ***
                if (currentState == MOVING_FORWARD && (lastShapeCommand == 'n' || lastShapeCommand == 'm')) {
                    Serial.printf("Shape '%c' initial move complete. Waiting for 'h'.\n", lastShapeCommand);
                    currentState = WAITING_HOME; // Enter waiting state
                }
                // *** ---------------------------------------- ***
                else if (currentState == TASK1_LEG1) { Serial.printf("Task 1 Leg 1 Complete. Pausing...\n"); pauseStartTime = millis(); currentState = TASK1_PAUSE_AFTER_LEG1; }
                else if (currentState == TASK1_LEG2) { Serial.printf("Task 1 Leg 2 Complete.\n"); Serial.println("Task 1 Finished."); lastShapeCommand = '\0'; currentState = IDLE; } // Clear shape command on task finish
                else { Serial.printf("Distance Fwd Complete.\n"); lastShapeCommand = '\0'; currentState = IDLE; } // Must have been 'f' command
            }
            break; // End forward movement states
        }

        case TASK1_PAUSE_AFTER_LEG1: { if (millis() - pauseStartTime >= TASK_PAUSE_DURATION_MS) { Serial.println("Task 1 Pause 1 Finished. Starting Turn..."); mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw; targetTurnAngle = 180.0; setSideSpeed(LEFT_SIDE, -1, TURN_SPEED); setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED); currentState = TASK1_TURN; } break; }
        case TASK1_TURN: { float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw)); Serial.printf("TASK1_TURN: Current=%.1f, Start=%.1f, Turned=%.1f, Target=%.1f\n", currentYaw, startYawForTurn, angleTurned, targetTurnAngle); if (angleTurned >= targetTurnAngle - (TURN_ANGLE_THRESHOLD * 1.5)) { Serial.printf("Task 1 Turn Complete. Turned: %.1f / Target: %.1f\n", angleTurned, targetTurnAngle); stopMotors(); Serial.println("Task 1 Turn Finished. Pausing..."); pauseStartTime = millis(); currentState = TASK1_PAUSE_AFTER_TURN; } break; }
        case TASK1_PAUSE_AFTER_TURN: { if (millis() - pauseStartTime >= TASK_PAUSE_DURATION_MS) { Serial.println("Task 1 Pause 2 Finished. Starting Leg 2..."); float distCm = 150.0; targetEncoderTicks = (long)((distCm * COUNTS_PER_CM) + 0.5f); mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw; Serial.print("Capturing start ticks for Task 1 Leg 2: "); portENTER_CRITICAL(&encoderMutex[ENC_LF]); startEncoderTicks[ENC_LF] = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]); portENTER_CRITICAL(&encoderMutex[ENC_RF]); startEncoderTicks[ENC_RF] = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]); Serial.printf("[LF]=%ld [RF]=%ld\n", startEncoderTicks[ENC_LF], startEncoderTicks[ENC_RF]); Serial.printf("Action: Start Task 1 Leg 2 (Target Ticks: %ld, Target Yaw: %.1f)\n", targetEncoderTicks, targetYaw); headingPID.SetMode(AUTOMATIC); setSideSpeed(LEFT_SIDE, 1, FWD_SPEED_BASE); setSideSpeed(RIGHT_SIDE, 1, FWD_SPEED_BASE); currentState = TASK1_LEG2; } break; }

        // *** ADDED: Shape/Home Specific States ***
        case WAITING_HOME: {
            // Do nothing, just wait for 'h' command in OnDataRecv
            // Maybe add a periodic print?
            // unsigned long now = millis();
            // if (now - lastLoopPrint > 2000) { // Print every 2 seconds
            //    Serial.printf("State: WAITING_HOME (Last Shape: %c)\n", lastShapeCommand);
            //    lastLoopPrint = now;
            // }
            break;
        }

        case EXECUTING_HOME_TURN: { // Turning 180 Left
            float angleTurned = abs(calculateAngleDifference(startYawForTurn, currentYaw));
            if (angleTurned >= targetTurnAngle - TURN_ANGLE_THRESHOLD) {
                Serial.printf("Home Turn Complete. Turned: %.1f / Target: %.1f\n", angleTurned, targetTurnAngle);
                stopMotors();
                // Prepare for return move
                targetEncoderTicks = (long)((homeReturnDistanceCm * COUNTS_PER_CM) + 0.5f);
                mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw; // Target heading is current after turn
                Serial.print("Capturing start ticks for Home Move: ");
                portENTER_CRITICAL(&encoderMutex[ENC_LF]); startEncoderTicks[ENC_LF] = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]);
                portENTER_CRITICAL(&encoderMutex[ENC_RF]); startEncoderTicks[ENC_RF] = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]);
                Serial.printf("[LF]=%ld [RF]=%ld\n", startEncoderTicks[ENC_LF], startEncoderTicks[ENC_RF]);
                Serial.printf("Action: Start Home Move %.1f cm (Target Ticks: %ld), Target Yaw: %.1f\n", homeReturnDistanceCm, targetEncoderTicks, targetYaw);
                headingPID.SetMode(AUTOMATIC);
                setSideSpeed(LEFT_SIDE, 1, FWD_SPEED_BASE); setSideSpeed(RIGHT_SIDE, 1, FWD_SPEED_BASE);
                currentState = EXECUTING_HOME_MOVE;
            }
            break;
        }

        case EXECUTING_HOME_MOVE: { // Moving forward the return distance
            headingError = calculateAngleDifference(targetYaw, currentYaw); headingPID.Compute();
            int leftSpeed = FWD_SPEED_BASE + (int)pidOutputCorrection; int rightSpeed = FWD_SPEED_BASE - (int)pidOutputCorrection;
            leftSpeed = constrain(leftSpeed, 0, MAX_PWM_VALUE); rightSpeed = constrain(rightSpeed, 0, MAX_PWM_VALUE);
            setSideSpeed(LEFT_SIDE, 1, leftSpeed); setSideSpeed(RIGHT_SIDE, 1, rightSpeed);

            // Check Distance Target based on LF OR RF encoder
            bool targetReached = false; long current_enc[NUM_ENCODERS];
            portENTER_CRITICAL(&encoderMutex[ENC_LF]); current_enc[ENC_LF] = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]);
            portENTER_CRITICAL(&encoderMutex[ENC_RF]); current_enc[ENC_RF] = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]);
            long moved_lf = abs(current_enc[ENC_LF] - startEncoderTicks[ENC_LF]);
            long moved_rf = abs(current_enc[ENC_RF] - startEncoderTicks[ENC_RF]);
            if (moved_lf >= targetEncoderTicks || moved_rf >= targetEncoderTicks) {
                targetReached = true;
                Serial.printf("Home Move Distance Complete: Target Ticks: %ld (LF moved: %ld, RF moved: %ld)\n", targetEncoderTicks, moved_lf, moved_rf);
            }

            if (targetReached) {
                stopMotors(); if (headingPID.GetMode() == AUTOMATIC) { headingPID.SetMode(MANUAL); }
                Serial.printf("Shape '%c' sequence finished.\n", lastShapeCommand);
                lastShapeCommand = '\0'; // Reset tracker
                currentState = IDLE;     // Return to normal idle
            }
            break;
        }
        // *** -------------------------------- ***

    } // End switch

    // --- Send Encoder Data via ESP-NOW periodically ---
    unsigned long currentTime = millis();
    if (currentTime - lastSendTime >= SEND_INTERVAL_MS) {
        portENTER_CRITICAL(&encoderMutex[ENC_LF]); encoderDataToSend.encoderLF = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]);
        portENTER_CRITICAL(&encoderMutex[ENC_RF]); encoderDataToSend.encoderRF = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]);
        esp_now_send(masterAddress, (uint8_t *) &encoderDataToSend, sizeof(encoderDataToSend));
        lastSendTime = currentTime;
    }
    delay(1);
}

// --- ESP-NOW Callbacks ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { /* Optional feedback */ }
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    struct_message_universal receivedMessage;
    if (len == sizeof(receivedMessage)) {
        memcpy(&receivedMessage, incomingData, sizeof(receivedMessage));
        if (receivedMessage.msg_type == COMMAND_MOVEMENT) {
            payload_movement_t movCmd = receivedMessage.payload.movement;
            Serial.printf("Command RX: Type=%c, Value=%.1f\n", movCmd.command_type, movCmd.value);

            // Handle stop 's' and reset 'x' anytime (These will override shape waiting)
            if (movCmd.command_type == 's') { stopMotors(); currentState = IDLE; lastShapeCommand = '\0'; if (headingPID.GetMode() == AUTOMATIC) { headingPID.SetMode(MANUAL); } return; }
            if (movCmd.command_type == 'x') { stopMotors(); delay(500); ESP.restart(); return; }

            // *** MODIFIED: Handle WAITING_HOME state ***
            if (currentState == WAITING_HOME) {
                if (movCmd.command_type == 'h') {
                    Serial.println("Action: 'h' received while waiting.");
                    // Determine return distance based on which shape was last run
                    if (lastShapeCommand == 'n') { homeReturnDistanceCm = 58.0; }
                    else if (lastShapeCommand == 'm') { homeReturnDistanceCm = 83.0; }
                    else { homeReturnDistanceCm = 0.0; Serial.println("Error: Waiting home but no last shape known!"); currentState = IDLE; return; } // Should not happen

                    // Initiate 180 Left turn
                    mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw;
                    targetTurnAngle = 180.0; // Relative turn
                    Serial.printf("Action: Start Home 180deg Left Turn from %.1f\n", startYawForTurn);
                    setSideSpeed(LEFT_SIDE, 1, TURN_SPEED); // Left FWD for Left turn
                    setSideSpeed(RIGHT_SIDE, -1, TURN_SPEED); // Right BWD for Left turn
                    currentState = EXECUTING_HOME_TURN;
                } else {
                    Serial.printf("Action: Ignoring command '%c' while WAITING_HOME.\n", movCmd.command_type);
                    // Do nothing, wait for 'h'
                }
                return; // Handled waiting state, exit callback
            }
            // *** --------------------------------- ***

            // Ignore other commands if not IDLE (and not waiting for home)
            if (currentState != IDLE) { Serial.println("Action: Busy, command ignored."); return; }

            // Process commands only when IDLE
            lastShapeCommand = '\0'; // Clear tracker for normal commands
            switch(movCmd.command_type) {
                case 'l': if (movCmd.value > 0) { mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw; targetTurnAngle = movCmd.value; Serial.printf("Action: Start Turning Left %.1f deg from %.1f\n", targetTurnAngle, startYawForTurn); setSideSpeed(LEFT_SIDE, 1, TURN_SPEED); setSideSpeed(RIGHT_SIDE, -1, TURN_SPEED); currentState = TURNING_LEFT; } else { Serial.println("Error: Turn angle must be positive."); } break;
                case 'r': if (movCmd.value > 0) { mpu.update(); currentYaw = mpu.getAngleZ(); startYawForTurn = currentYaw; targetTurnAngle = movCmd.value; Serial.printf("Action: Start Turning Right %.1f deg from %.1f\n", targetTurnAngle, startYawForTurn); setSideSpeed(LEFT_SIDE, -1, TURN_SPEED); setSideSpeed(RIGHT_SIDE, 1, TURN_SPEED); currentState = TURNING_RIGHT; } else { Serial.println("Error: Turn angle must be positive."); } break;
                case 'f': if (movCmd.value > 0) { float distanceCm = movCmd.value; targetEncoderTicks = (long)((distanceCm * COUNTS_PER_CM) + 0.5f); mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw; Serial.print("Capturing start encoder ticks: "); portENTER_CRITICAL(&encoderMutex[ENC_LF]); startEncoderTicks[ENC_LF] = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]); portENTER_CRITICAL(&encoderMutex[ENC_RF]); startEncoderTicks[ENC_RF] = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]); Serial.printf("[LF]=%ld [RF]=%ld\n", startEncoderTicks[ENC_LF], startEncoderTicks[ENC_RF]); Serial.printf("Action: Start Moving Fwd %.1f cm (Target Ticks: %ld), Target Yaw: %.1f\n", distanceCm, targetEncoderTicks, targetYaw); headingPID.SetMode(AUTOMATIC); setSideSpeed(LEFT_SIDE, 1, FWD_SPEED_BASE); setSideSpeed(RIGHT_SIDE, 1, FWD_SPEED_BASE); currentState = MOVING_FORWARD; } else { Serial.println("Error: Forward distance must be positive."); } break;
                case 't': if (movCmd.value == 1) { Serial.println("Action: Starting Task 1 (150cm Fwd -> 180R -> 150cm Fwd)"); float distCm = 150.0; targetEncoderTicks = (long)((distCm * COUNTS_PER_CM) + 0.5f); mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw; Serial.print("Capturing start ticks for Task 1 Leg 1: "); portENTER_CRITICAL(&encoderMutex[ENC_LF]); startEncoderTicks[ENC_LF] = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]); portENTER_CRITICAL(&encoderMutex[ENC_RF]); startEncoderTicks[ENC_RF] = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]); Serial.printf("[LF]=%ld [RF]=%ld\n", startEncoderTicks[ENC_LF], startEncoderTicks[ENC_RF]); Serial.printf("--> Task 1 Leg 1 Start (Target Ticks: %ld, Yaw: %.1f)\n", targetEncoderTicks, targetYaw); headingPID.SetMode(AUTOMATIC); setSideSpeed(LEFT_SIDE, 1, FWD_SPEED_BASE); setSideSpeed(RIGHT_SIDE, 1, FWD_SPEED_BASE); currentState = TASK1_LEG1; } else { Serial.printf("Error: Unknown task ID %.0f\n", movCmd.value); } break;
                case 'n': { float distanceCm = 55.0; targetEncoderTicks = (long)((distanceCm * COUNTS_PER_CM) + 0.5f); mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw; Serial.print("Capturing start encoder ticks: "); portENTER_CRITICAL(&encoderMutex[ENC_LF]); startEncoderTicks[ENC_LF] = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]); portENTER_CRITICAL(&encoderMutex[ENC_RF]); startEncoderTicks[ENC_RF] = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]); Serial.printf("[LF]=%ld [RF]=%ld\n", startEncoderTicks[ENC_LF], startEncoderTicks[ENC_RF]); Serial.printf("Action: Start Shape 'n' - Moving Fwd %.1f cm (Target Ticks: %ld), Target Yaw: %.1f\n", distanceCm, targetEncoderTicks, targetYaw); lastShapeCommand = 'n'; headingPID.SetMode(AUTOMATIC); setSideSpeed(LEFT_SIDE, 1, FWD_SPEED_BASE); setSideSpeed(RIGHT_SIDE, 1, FWD_SPEED_BASE); currentState = MOVING_FORWARD; break; }
                case 'm': { float distanceCm = 80.0; targetEncoderTicks = (long)((distanceCm * COUNTS_PER_CM) + 0.5f); mpu.update(); currentYaw = mpu.getAngleZ(); targetYaw = currentYaw; Serial.print("Capturing start encoder ticks: "); portENTER_CRITICAL(&encoderMutex[ENC_LF]); startEncoderTicks[ENC_LF] = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]); portENTER_CRITICAL(&encoderMutex[ENC_RF]); startEncoderTicks[ENC_RF] = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]); Serial.printf("[LF]=%ld [RF]=%ld\n", startEncoderTicks[ENC_LF], startEncoderTicks[ENC_RF]); Serial.printf("Action: Start Shape 'm' - Moving Fwd %.1f cm (Target Ticks: %ld), Target Yaw: %.1f\n", distanceCm, targetEncoderTicks, targetYaw); lastShapeCommand = 'm'; headingPID.SetMode(AUTOMATIC); setSideSpeed(LEFT_SIDE, 1, FWD_SPEED_BASE); setSideSpeed(RIGHT_SIDE, 1, FWD_SPEED_BASE); currentState = MOVING_FORWARD; break; }
                case 'h': Serial.println("Ignoring 'h' command while IDLE."); break; // Ignore 'h' if idle
                default: stopMotors(); currentState = IDLE;
            } // End switch
        } else if (receivedMessage.msg_type == COMMAND_PID_TUNING) {
            payload_pid_tuning_t tuningCmd = receivedMessage.payload.tuning;
            Kp_pid = tuningCmd.kp; Ki_pid = tuningCmd.ki; Kd_pid = tuningCmd.kd;
            headingPID.SetTunings(Kp_pid, Ki_pid, Kd_pid);
            Serial.printf("PID Tunings Updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp_pid, Ki_pid, Kd_pid);
        }
    }
}

// --- ISR ---
void IRAM_ATTR handleEncoderInterrupt(void *arg) { int i = (int)arg; if (i < 0 || i >= NUM_ENCODERS) return; portENTER_CRITICAL_ISR(&encoderMutex[i]); bool pinB = digitalRead(encoderPins[i][1]); if (pinB) { encoderCounts[i]++; } else { encoderCounts[i]--; } portEXIT_CRITICAL_ISR(&encoderMutex[i]); }

// --- Motor Control (Original 2-pin Style) ---
void setSideSpeed(int side, int direction, int speed) { if (side < 0 || side >= NUM_SIDES) return; int pin1, pin2; if (side == LEFT_SIDE) { pin1 = leftMotorPins[0]; pin2 = leftMotorPins[1]; } else { pin1 = rightMotorPins[0]; pin2 = rightMotorPins[1]; } int channel = ledcChannels[side]; int pwmValue = constrain(speed, 0, MAX_PWM_VALUE); ledcDetachPin(pin1); ledcDetachPin(pin2); pinMode(pin1, OUTPUT); pinMode(pin2, OUTPUT); if (direction == 1) { digitalWrite(pin2, LOW); ledcAttachPin(pin1, channel); ledcWrite(channel, pwmValue); } else if (direction == -1) { digitalWrite(pin1, LOW); ledcAttachPin(pin2, channel); ledcWrite(channel, pwmValue); } else { digitalWrite(pin1, LOW); digitalWrite(pin2, LOW); ledcWrite(channel, 0); } }

// --- Helper for Angle Difference ---
float calculateAngleDifference(float targetAngle, float currentAngle) { float diff = targetAngle - currentAngle; while (diff <= -180.0f) diff += 360.0f; while (diff > 180.0f) diff -= 360.0f; return diff; }

// --- Helper to Stop Motors ---
void stopMotors() { setSideSpeed(LEFT_SIDE, 0, 0); setSideSpeed(RIGHT_SIDE, 0, 0); }

// --- Helper to get Average Encoder Ticks --- (LF/RF Only) ---
long getAverageEncoderTicks() {
    long lf, rf; // Only need LF and RF
    portENTER_CRITICAL(&encoderMutex[ENC_LF]); lf = encoderCounts[ENC_LF]; portEXIT_CRITICAL(&encoderMutex[ENC_LF]); // Index 0
    portENTER_CRITICAL(&encoderMutex[ENC_RF]); rf = encoderCounts[ENC_RF]; portEXIT_CRITICAL(&encoderMutex[ENC_RF]); // Index 1
    return (lf + rf) / 2; // Average of LF and RF
}

// Odometry functions removed

// --- END: Rover ESP32 Code --