// --- START: Master ESP32 Code (V6.4 - Consistent Structs) ---

// --- Includes ---
#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// --- Network Configuration ---
const char *ssid = "ROVER_CONTROL"; // SSID for the Access Point
const char *password = "12345678";   // Password for AP (min 8 chars)

// --- ESP-NOW Configuration ---
#define NUM_ROVERS 5 // Support up to 5 rovers

// !!!>>> REPLACE with actual Rover MAC Addresses <<<!!!
uint8_t roverAddresses[NUM_ROVERS][6] = {
  {0xF4, 0x65, 0x0B, 0xE9, 0x29, 0xF0}, // Rover 0 MAC (Index 0)
  {0x78, 0x42, 0x1C, 0x6C, 0xC8, 0xA0}, // Rover 1 MAC (Index 1)
  {0xF4, 0x65, 0x0B, 0x54, 0x7D, 0x80}, // Rover 2 MAC (Index 2)
  {0xF4, 0x65, 0x0B, 0xE8, 0xB9, 0xBC}, // Rover 3 MAC (Index 3)
  {0x78, 0x42, 0x1C, 0x6C, 0x6A, 0x38}  // Rover 4 MAC (Index 4)
};
// !!!>>> --------------------------------------- <<<!!!
esp_now_peer_info_t peerInfo;
bool peerAdded[NUM_ROVERS]; // Track if peer was added successfully

// --- Data Structures ---
// *** Using Universal Struct for Master -> Rover communication ***
typedef enum { COMMAND_MOVEMENT, COMMAND_PID_TUNING } message_type_t; // Keep both types
typedef struct { char command_type; float value; } payload_movement_t;
typedef struct { double kp; double ki; double kd; } payload_pid_tuning_t; // Keep for struct definition
typedef struct struct_message_universal {
    message_type_t msg_type;
    union {
        payload_movement_t movement;
        payload_pid_tuning_t tuning; // Keep for struct definition
    } payload;
} struct_message_universal;
// *** ------------------------------------------------------- ***

// Structure for Rover->Master (Expected, but content ignored)
typedef struct struct_message_encoder_data { long encoderLF; long encoderRF; } struct_message_encoder_data;

// --- Web Server ---
AsyncWebServer server(80);

// --- HTML Page ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Rover Multi-Select Control V6.4</title> <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta charset="UTF-8">
    <style>
        body{font-family:Arial,sans-serif;text-align:center;background-color:#f4f4f4;margin:0;padding:10px;}
        h2,h3{color:#333; margin-bottom: 10px;}
        .control-section{background-color:#fff;border:1px solid #ddd;border-radius:8px;padding:15px;margin:10px auto;max-width:600px;box-shadow:0 2px 4px rgba(0,0,0,.1)}
        button{font-size:16px;padding:8px 12px;margin:5px;border:none;border-radius:5px;cursor:pointer;background-color:#e0e0e0;color:#333;transition:background-color .2s ease; min-width: 70px;}
        button:hover{background-color:#ccc} button:active{background-color:#b0b0b0}
        input[type="number"]{font-size:16px;padding:8px;margin:5px 10px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box; width: 120px; text-align: right;}
        label{margin: 0 5px; font-weight: bold; display: inline-block; min-width: 100px; text-align: right;}
        .button_stop{background-color:#dc3545;color:#fff;} .button_stop:hover{background-color:#c82333}
        #statusArea{margin-top:15px;padding:10px;background-color:#e9ecef;border:1px solid #ced4da;border-radius:5px;color:#495057;min-height:1.5em;max-width:600px;margin-left:auto;margin-right:auto;font-family:monospace; font-size: 0.9em; text-align: left;}
        #resetButton, .preset-button {background-color:#ffc107;color:#333;} #resetButton:hover, .preset-button:hover {background-color:#e0a800;}
        .action-group { margin-top: 10px; padding-bottom: 10px; border-bottom: 1px solid #eee;}
        .action-group:last-child { border-bottom: none; }
        .action-group button { width: 140px; }
        .preset-button { width: auto; padding: 8px 15px; }
        .rover-checkbox-group label {min-width: 60px; text-align: left; margin-right: 15px;}
        .rover-checkbox-group input {width: auto; margin-right: 5px;}
    </style>
</head>
<body>
    <h2>Multi-Rover Control Panel</h2>

    <div class="control-section">
        <h3>Target Rover(s)</h3>
        <div class="rover-checkbox-group">
            <input type="checkbox" id="rover_0" name="rover_select" value="0"><label for="rover_0">Rover 1</label>
            <input type="checkbox" id="rover_1" name="rover_select" value="1"><label for="rover_1">Rover 2</label>
            <input type="checkbox" id="rover_2" name="rover_select" value="2"><label for="rover_2">Rover 3</label>
            <input type="checkbox" id="rover_3" name="rover_select" value="3"><label for="rover_3">Rover 4</label>
            <input type="checkbox" id="rover_4" name="rover_select" value="4"><label for="rover_4">Rover 5</label>
        </div>
        <div><button onclick="selectAllRovers(true)">Select All</button> <button onclick="selectAllRovers(false)">Deselect All</button></div>
    </div>
    <div class="control-section">
         <h3>Movement Commands</h3>
         <div class="action-group">
             <label for="angleInput">Angle (Deg):</label>
             <input type="number" id="angleInput" value="90" step="1"> <br>
             <button onclick="sendCommand('l')">Turn Left</button>
             <button onclick="sendCommand('r')">Turn Right</button>
        </div>
         <div class="action-group">
             <label for="fwdCmInput">Fwd Dist (cm):</label>
             <input type="number" id="fwdCmInput" value="50" step="1"> <br>
             <button onclick="sendCommand('f')">Move Forward</button>
        </div>
         <div class="action-group">
             <button class="button_stop" onclick="sendCommand('s')" style="width: 140px;">STOP</button>
             <button id="resetButton" onclick="sendCommand('x')" style="width: 140px;">Reset Selected</button>
        </div>
    </div>

     <div class="control-section">
        <h3>Preset Tasks (Sent to Selected Rovers)</h3>
        <button class="preset-button" onclick="runPresetCommand('t', 1)">Run Task 1 (Out-Back)</button>
        <button class="preset-button" onclick="runPresetCommand('n')">Shape: X</button>
        <button class="preset-button" onclick="runPresetCommand('m')">Shape: +</button>
        <button class="preset-button" onclick="runPresetCommand('h')">Go Home</button>
     </div>

    <div id="statusArea">Master Ready.</div>

    <script>
        const statusArea = document.getElementById('statusArea');
        function updateStatus(message){if(statusArea){statusArea.innerText=message;/*console.log("Status:",message)*/}}
        window.addEventListener('load', () => { updateStatus("Master Ready.")});

        function getSelectedRoverIds() {
            const checkboxes = document.querySelectorAll('input[name="rover_select"]:checked');
            const selectedIds = [];
            checkboxes.forEach((checkbox) => { selectedIds.push(checkbox.value); });
            return selectedIds.join(',');
        }

        function selectAllRovers(select) {
             document.querySelectorAll('input[name="rover_select"]').forEach((checkbox) => { checkbox.checked = select; });
        }

        // Send Basic Movement/Stop/Reset commands
        function sendCommand(cmdType){
            const selectedRovers = getSelectedRoverIds();
            if (selectedRovers === '') { alert('Please select at least one rover.'); return; }
            let value = 0; let valueStr = '';
            if (cmdType === 'l' || cmdType === 'r') { valueStr = document.getElementById('angleInput').value; if (valueStr === '') { alert('Please enter an angle.'); return; } value = parseFloat(valueStr); }
            else if (cmdType === 'f') { valueStr = document.getElementById('fwdCmInput').value; if (valueStr === '') { alert('Please enter forward distance (cm).'); return; } value = parseFloat(valueStr); }
            let url = `/command?rover=${encodeURIComponent(selectedRovers)}&cmd=${encodeURIComponent(cmdType)}&val=${encodeURIComponent(value)}`;
            // console.log("Sending URL:", url); // Removed log
            updateStatus(`Sending: ${cmdType} (${value}) to Rovers [${selectedRovers}]...`);
            fetch(url).then(response => { if (!response.ok) { updateStatus(`Error: Command '${cmdType}' to [${selectedRovers}] failed! Status ${response.status}`); throw new Error(`HTTP error! status:${response.status}`); } updateStatus(`Command '${cmdType}' sent to Rovers [${selectedRovers}].`); }).catch(error => { updateStatus(`Error sending '${cmdType}' to [${selectedRovers}]. ${error}`); /*console.error('Fetch error:', error);*/ });
        }

         // Send Preset command (Task, Shape, or Home)
         function runPresetCommand(cmdType, cmdValue = 0) {
             const selectedRovers = getSelectedRoverIds();
             if (selectedRovers === '') { alert('Please select at least one rover for the preset task.'); return; }
             let value = (cmdType === 't') ? cmdValue : 0;
             let url = `/preset?rover=${encodeURIComponent(selectedRovers)}&cmd=${encodeURIComponent(cmdType)}&val=${encodeURIComponent(value)}`;
             // console.log("Sending Preset URL:", url); // Removed log
             updateStatus(`Sending Preset ${cmdType} (${value}) to Rovers [${selectedRovers}]...`);
             fetch(url).then(response => { if (!response.ok) { updateStatus(`Error: Preset '${cmdType}' to [${selectedRovers}] failed! Status ${response.status}`); throw new Error(`HTTP error! status:${response.status}`); } updateStatus(`Preset '${cmdType}' command sent to Rovers [${selectedRovers}].`); }).catch(error => { updateStatus(`Error sending preset '${cmdType}' to [${selectedRovers}]. ${error}`); /*console.error('Fetch error:', error);*/ });
         }
    </script>
</body>
</html>
)rawliteral";

// --- Function Prototypes ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
String mac_addr_to_str(const uint8_t *mac_addr);
int getRoverIndex(const uint8_t *macAddr);
// *** MODIFIED: sendCmdToRover prototype now takes the universal struct ***
bool sendCmdToRover(int roverIndex, const struct_message_universal& msg);

// --- Setup ---
void setup() {
    // Serial.begin(115200); // Serial removed
    // Serial.println("Master ESP32 Setup (V6.4 - Consistent Structs)..."); // Version Bump
    memset(peerAdded, false, sizeof(peerAdded));

    // WiFi AP & ESP-NOW Init
    WiFi.softAP(ssid, password); WiFi.mode(WIFI_AP_STA);
    if (esp_now_init() != ESP_OK) { ESP.restart(); }
    esp_now_register_send_cb(OnDataSent);
    // Add ALL Rover peers
    for (int i = 0; i < NUM_ROVERS; i++) {
        bool isPlaceholder = true; for(int j=0; j<6; j++) { if(roverAddresses[i][j] != 0xFF) {isPlaceholder=false; break;} }
        if (!isPlaceholder) { memcpy(peerInfo.peer_addr, roverAddresses[i], 6); peerInfo.channel = 0; peerInfo.encrypt = false;
            if (esp_now_add_peer(&peerInfo) == ESP_OK) { peerAdded[i] = true; } else { peerAdded[i] = false;}
        } else { peerAdded[i] = false; }
    }
    esp_now_register_recv_cb(OnDataRecv);
    // Serial.println("ESP-NOW Initialized."); // Serial removed

    // --- Setup Async Web Server Handlers ---
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", index_html); });

    // Command handler - parses rover list, sends basic commands using universal struct
    server.on("/command", HTTP_GET, [](AsyncWebServerRequest *request){
        // *** MODIFIED: Create and send universal struct ***
        struct_message_universal msgToSend;
        msgToSend.msg_type = COMMAND_MOVEMENT; // Set type
        bool cmdValid = false; float value = 0; String roverIdStr = "";

        if (!request->hasParam("rover") || !request->hasParam("cmd")) { request->send(400, "text/plain", "Bad Request: Missing params"); return; }
        roverIdStr = request->getParam("rover")->value(); String cmdStr = request->getParam("cmd")->value();
        if (cmdStr.length() == 1) {
             msgToSend.payload.movement.command_type = cmdStr.charAt(0); // Set command char
             if (String("lrf").indexOf(msgToSend.payload.movement.command_type) != -1) { if (request->hasParam("val")) { value = request->getParam("val")->value().toFloat(); cmdValid = true; } }
             else if (String("sx").indexOf(msgToSend.payload.movement.command_type) != -1) { value = 0; cmdValid = true; }
        }
        if (!cmdValid) { request->send(400, "text/plain", "Bad Command"); return; }
        msgToSend.payload.movement.value = value; // Set value

        // Parse roverIdStr and send individually
        bool overallSuccess = true; int sentCount = 0; int startIndex = 0; int commaIndex = roverIdStr.indexOf(',');
        while (commaIndex != -1 || startIndex < roverIdStr.length()) {
            String currentIdStr = (commaIndex != -1) ? roverIdStr.substring(startIndex, commaIndex) : roverIdStr.substring(startIndex);
            int roverIndex = currentIdStr.toInt();
            // *** MODIFIED: Pass the full struct to sendCmdToRover ***
            if (sendCmdToRover(roverIndex, msgToSend)) { sentCount++; } else { overallSuccess = false; }
            if (commaIndex == -1) break; startIndex = commaIndex + 1; commaIndex = roverIdStr.indexOf(',', startIndex);
        }
        if (sentCount == 0 && roverIdStr.length() > 0) { overallSuccess = false; } else if (roverIdStr.length() == 0) { overallSuccess = false; }
        request->send(overallSuccess ? 200 : 500, "text/plain", overallSuccess ? "OK" : "ESP-NOW Send Fail (One or more)");
    });

     // Preset handler for Tasks/Shapes/Home ('t', 'n', 'm', 'h') using universal struct
    server.on("/preset", HTTP_GET, [](AsyncWebServerRequest *request){
        // *** MODIFIED: Create and send universal struct ***
        struct_message_universal msgToSend;
        msgToSend.msg_type = COMMAND_MOVEMENT; // Still a movement command type
        bool cmdValid = false; float value = 0; String roverIdStr = "";

        if (!request->hasParam("rover") || !request->hasParam("cmd")) { request->send(400, "text/plain", "Bad Request: Missing params"); return; }
        roverIdStr = request->getParam("rover")->value(); String cmdStr = request->getParam("cmd")->value();
        if (cmdStr.length() == 1) {
             msgToSend.payload.movement.command_type = cmdStr.charAt(0); // Set command char ('t','n','m','h')
             if (String("tnmh").indexOf(msgToSend.payload.movement.command_type) != -1) {
                 value = 0; if (msgToSend.payload.movement.command_type == 't' && request->hasParam("val")) { value = request->getParam("val")->value().toFloat(); }
                 cmdValid = true;
             }
        }
        if (!cmdValid) { request->send(400, "text/plain", "Bad Preset Command"); return; }
        msgToSend.payload.movement.value = value; // Set value (task ID or 0)

        // Parse roverIdStr and send individually
        bool overallSuccess = true; int sentCount = 0; int startIndex = 0; int commaIndex = roverIdStr.indexOf(',');
        while (commaIndex != -1 || startIndex < roverIdStr.length()) {
             String currentIdStr = (commaIndex != -1) ? roverIdStr.substring(startIndex, commaIndex) : roverIdStr.substring(startIndex);
             int roverIndex = currentIdStr.toInt();
             // *** MODIFIED: Pass the full struct to sendCmdToRover ***
             if (sendCmdToRover(roverIndex, msgToSend)) { sentCount++; } else { overallSuccess = false; }
             if (commaIndex == -1) break; startIndex = commaIndex + 1; commaIndex = roverIdStr.indexOf(',', startIndex);
        }
        if (sentCount == 0 && roverIdStr.length() > 0) { overallSuccess = false; } else if (roverIdStr.length() == 0) { overallSuccess = false; }
        request->send(overallSuccess ? 200 : 500, "text/plain", overallSuccess ? "OK" : "ESP-NOW Send Fail (One or more)");
    });

    server.onNotFound([](AsyncWebServerRequest *request){ request->send(404, "text/plain", "Not Found"); });
    server.begin();
    // Serial removed
}

// --- Loop --- (Empty)
void loop() {
     delay(100);
}

// --- Callback Function Definitions ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { /* Optional logging */ }
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) { /* Ignoring incoming data */ }

// --- Helper Functions ---
String mac_addr_to_str(const uint8_t *mac_addr) { char macStr[18]; snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]); return String(macStr); }
int getRoverIndex(const uint8_t *macAddr) { for (int i = 0; i < NUM_ROVERS; i++) { if (memcmp(roverAddresses[i], macAddr, 6) == 0) { return i; } } return -1; }

// *** MODIFIED: sendCmdToRover helper sends the universal struct ***
bool sendCmdToRover(int roverIndex, const struct_message_universal& msg) {
    if (roverIndex < 0 || roverIndex >= NUM_ROVERS || !peerAdded[roverIndex]) { return false; }
    esp_err_t result = esp_now_send(roverAddresses[roverIndex], (uint8_t *) &msg, sizeof(msg));
    // Add delay after each send?
    delay(2);
    // if (result != ESP_OK) { Serial.printf("--> Send FAILED for Rover %d\n", roverIndex + 1); } // Serial removed
    return (result == ESP_OK);
}
// *** --------------------------------------------------------- ***

// --- END: Master ESP32 Code ---