#include "UbidotsWizFi250.h"
#define WLAN_SSID       "OpenWRT"  // Your WiFi SSID, cannot be longer than 32 characters!
#define WLAN_PASS       "Your_pass_here"  // Replace it with your WiFi pass
// Security can be OPEN, WEP, WPA, WPAAES, WPA2AES, WPA2TKIP, WPA2
#define WLAN_SECURITY   WEP

#define TOKEN "Your_token_here"  // Replace it with your Ubidots token
#define ID "Your_id_here" // Replace it with your Ubidots' variable ID
#define ID2 "Your_id2_here" 
#define ID3 "Your_id3_here"
#define ID4 "Your_id4_here" 
#define ID5 "Your_id5_here"

Ubidots client(TOKEN);

void setup() {
  Serial.begin(115200);
  while(!client.wifiConnection(WLAN_SSID, WLAN_PASS, WLAN_SECURITY));

}

void loop() {
  float value = analogRead(A0);
  float value2 = analogRead(A1);
  float value3 = analogRead(A2);
  float value4 = analogRead(A3);
  float value5 = analogRead(A4);
  client.add(ID,value);
  client.add(ID2,value2);
  client.add(ID3,value3);
  client.add(ID4,value4);
  client.add(ID5,value5);
  client.sendAll();
}