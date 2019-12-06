/* This library is for WiFi shield of Seeedstudio with WIZnet WizFi250
   Made by Mateo Velez - Metavix for Ubidots.
   Feb 2016
*/

#ifndef __UbidotsWizFi250_H_
#define __UbidotsWizFi250_H_
#include <Arduino.h>
#include <SoftwareSerial.h>

#define SERVER "50.23.124.68"
#define  REMOTE_PORT    "80"

#define TX 2
#define RX 3
#define BAUDRATE 115200
//#define DEBUG_UBIDOTS


typedef struct Value {
  char  *id;
  float value_id;
} Value;

class Ubidots {
   public:
      Ubidots(char* token);
      bool sendAll();
      bool wifiConnection(const char *ssid, const char *phrase, const char *auth);
      //float getValue(char* id);
      void add(char *variable_id, double value);
   
   private:
      SoftwareSerial _client = SoftwareSerial(TX, RX);
      char* _token;
      char* readData(uint16_t timeout);
      uint8_t maxValues;
      uint8_t currentValue;
      Value * val;
      float parseValue(String body);
            
};
#endif