# Ubidots WIZnet WizFi250 of Seeedstudio

<div style="text-align:center "><img src ="http://www.seeedstudio.com/wiki/images/thumb/e/e0/Fi250_board1.jpg/350px-Fi250_board1.jpg" /></div>
Wifi shield （Fi250）is a economic internet solution with Arduino. The Wi-Fi module support IEEE 802.11b/g/n mode, maximum speed up to 65Mbit/s. Wifi shield (Fi250) combine on board antenna; it’s easy to build you project in a small box. The module reserve a UFL connector, you can use an extra antenna to improve the signal range. There is a key on the board, just push one time to change wifi shield as AP mode. The module have a computer interface software, you can control and upgrade the module via USB-UART convertor. The wifi shield contain a Micro SD card socket, it’s like a recorder when wifi shield working as TCP, UDP server. 

## Requirements

* [An Arduino UNO](http://arduino.cc/en/Main/arduinoBoardUno)
* [A seeedstudio WiFi Shield](https://www.adafruit.com/product/1491)
* [Arduino IDE 1.6.5 or higher](https://www.arduino.cc/en/Main/Software)
* [Ubidots WizFi250 library](https://github.com/ubidots/ubidots-wiznet/archive/master.zip)
 
## Setup

1. Download the UbidotsWizFi250 library [here](https://github.com/ubidots/ubidots-wiznet/archive/master.zip)
2. Go to the Arduino IDE, click on **Sketch -> Include Library -> Add .ZIP Library**
3. Select the .ZIP file of UbidotsWizFi250 and then "Accept" or "Choose"
4. Close the Arduino IDE and open it again.

<aside class="warning">
This module have some problems with SoftwareSerial library for this reason is impossible to implement a getValue function.
</aside>
    
## Send one value to Ubidots

To send a value to Ubidots, go to **Sketch -> Examples -> UbidotsWizFi250 library** and select the "saveValue" example. 
Put your Ubidots token and variable ID where indicated, as well as the WiFi settings.
Upload the code, open the Serial monitor to check the results. If no response is seen, try unplugging your Arduino and then plugging it again. Make sure the baud rate of the Serial monitor is set to the same one specified in your code.

```c++
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
```


## Send multiple values to Ubidots 

To send multiple values to Ubidots, go to **Sketch -> Examples -> Ubidots UbidotsWizFi250** and select the "saveValues" example. 
Put your Ubidots token and variable ID's where indicated, as well as the WiFi settings.
Upload the code, open the Serial monitor to check the results. If no response is seen, try unplugging your Arduino and then plugging it again. Make sure the baud rate of the Serial monitor is set to the same one specified in your code.

```c++
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
```