
// --- START: ESP32 Serial2-to-Bluetooth Bridge (V2.0 - Simplified) ---
// Reads PID data (e.g., "E:1.23 C:-4.56") from Serial2.
// Parses the data to extract Error and Correction values.
// Sends the parsed values via Bluetooth Serial (SPP).
// No Wi-Fi or ESP-NOW functionality.

#include <Arduino.h>
#include "BluetoothSerial.h" // Bluetooth Serial library

// --- Bluetooth Serial Setup ---
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT; // Bluetooth Serial object
String deviceName = "PID_BT"; // Name for Bluetooth discovery

// --- Serial Communication (from other ESP32) ---
#define SERIAL2_RXPIN 16
#define SERIAL2_TXPIN 17 // Not used, but defined for Serial2.begin
#define SERIAL_BAUD_RATE 115200
String serialBuffer = ""; // Buffer to hold incoming Serial2 line

// --- Global Variables to store latest parsed data ---
// Kept global in case needed elsewhere later, but only used for BT print now.
float currentHeadingError = 0.0;
float currentPidCorrection = 0.0;

// --- Function Prototypes ---
void parseSerialData(String data);

// --- Setup ---
void setup() {
    // Initialize Primary Serial (USB) for debugging this ESP32
    Serial.begin(115200);
    Serial.println("\nESP32 Serial2-to-Bluetooth Bridge (V2.0) Setup...");

    // Initialize Bluetooth Serial
    if (!SerialBT.begin(deviceName)) {
        Serial.println("FATAL: An error occurred initializing Bluetooth Serial!");
        ESP.restart(); // Restart if Bluetooth fails
    } else {
        Serial.printf("Bluetooth Serial started. Device name: '%s'. Ready to pair.\n", deviceName.c_str());
    }

    // Initialize Serial2 to receive data from the first ESP32
    Serial2.begin(SERIAL_BAUD_RATE, SERIAL_8N1, SERIAL2_RXPIN, SERIAL2_TXPIN);
    Serial.printf("Serial2 initialized (RX:%d) at %d baud. Waiting for data...\n", SERIAL2_RXPIN, SERIAL_BAUD_RATE);
    serialBuffer.reserve(100); // Pre-allocate buffer

    Serial.println("Setup Complete. Forwarding parsed Serial2 data to Bluetooth...");
}

// --- Loop ---
void loop() {
    // Read data from Serial2 continuously
    while (Serial2.available() > 0) {
        char incomingChar = Serial2.read();

        // Process line by line, ending with newline '\n'
        if (incomingChar == '\n') {
            // Optionally print raw received line to USB Serial for debugging
            // Serial.print("Raw Serial2 RX: ");
            // Serial.println(serialBuffer);

            parseSerialData(serialBuffer); // Parse and send via Bluetooth
            serialBuffer = ""; // Reset buffer for next line
        }
        // Append printable characters (and ignore carriage return '\r')
        else if (incomingChar >= 32 && incomingChar != '\r') {
            if (serialBuffer.length() < 99) { // Prevent buffer overflow
                serialBuffer += incomingChar;
            } else {
                Serial.println("WARN: Serial2 buffer overflow! Clearing buffer.");
                serialBuffer = ""; // Clear overflowing buffer
            }
        }
    }
    // No delay needed - loop depends on Serial2 data arrival
}

// --- Function Definitions ---

// Parses the string received from Serial2 (e.g., "E:12.34 C:-5.67")
// and sends parsed data via Bluetooth Serial
void parseSerialData(String data) {
    int errorIndex = data.indexOf("E:");
    int correctionIndex = data.indexOf("C:");

    // Check if both markers "E:" and "C:" are found and in the correct order
    if (errorIndex != -1 && correctionIndex != -1 && correctionIndex > errorIndex) {
        String errorStr = data.substring(errorIndex + 2, correctionIndex);
        String correctionStr = data.substring(correctionIndex + 2);
        errorStr.trim(); // Remove leading/trailing whitespace
        correctionStr.trim();

        // Convert substrings to float
        float parsedError = errorStr.toFloat();
        float parsedCorrection = correctionStr.toFloat();

        // Basic validation for toFloat()
        bool errorValid = (parsedError != 0.0 || errorStr.equals("0") || errorStr.equals("0.0") || errorStr.equals("-0.0"));
        bool correctionValid = (parsedCorrection != 0.0 || correctionStr.equals("0") || correctionStr.equals("0.0") || correctionStr.equals("-0.0"));

        if (errorValid && correctionValid) {
            // Update global variables (optional if only used here)
            currentHeadingError = parsedError;
            currentPidCorrection = parsedCorrection;

            // Print parsed values to USB Serial Monitor for local debugging
            Serial.printf("Parsed OK -> E: %.2f, C: %.2f --> Sending via BT\n", currentHeadingError, currentPidCorrection);

            // Send parsed data over Bluetooth Serial IF a client is connected
            if (SerialBT.hasClient()) {
                // Send formatted string with newline
                SerialBT.printf("E:%.2f C:%.2f\n", currentHeadingError, currentPidCorrection);
            } else {
                // Optionally notify via USB Serial if no BT client is connected
                // Serial.println("(BT Client not connected)");
            }

        } else {
             // Log parsing failure (toFloat likely failed) to USB Serial
             Serial.println("Serial parsing failed (toFloat). Data: [" + data + "]");
        }
    } else {
        // Log format errors (markers not found/out of order) to USB Serial
        Serial.println("Serial format error (E: or C: missing/order). Data: [" + data + "]");
    }
}

// --- END: ESP32 Serial2-to-Bluetooth Bridge ---