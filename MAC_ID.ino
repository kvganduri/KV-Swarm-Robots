// Sketch to get MAC Address
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_read_mac

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 MAC Address: ");
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA); // Read the STA MAC address
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(macStr);
}

void loop(){
  // Nothing needed here
}