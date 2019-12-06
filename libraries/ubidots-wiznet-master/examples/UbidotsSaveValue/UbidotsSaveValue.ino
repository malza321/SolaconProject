#include "UbidotsWizFi250.h"
#define TOKEN "Your_token_here"  // Replace it with your Ubidots token
#define ID "Your_id_here" // Replace it with your Ubidots' variable ID

#define WLAN_SSID       "OpenWRT"  // Your WiFi SSID, cannot be longer than 32 characters!
#define WLAN_PASS       "Your_pass_here"  // Replace it with your WiFi pass
// Security can be OPEN, WEP, WPA, WPAAES, WPA2AES, WPA2TKIP, WPA2
#define WLAN_SECURITY   WEP

Ubidots client(TOKEN);

void setup() {
  Serial.begin(115200);
  while(!client.wifiConnection(WLAN_SSID, WLAN_PASS, WLAN_SECURITY));

}

void loop() {
  float value = analogRead(A0);
  client.add(ID,value);
  client.sendAll();
}