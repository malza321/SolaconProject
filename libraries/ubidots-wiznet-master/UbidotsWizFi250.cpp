#include "UbidotsWizFi250.h"
#include <SoftwareSerial.h>
/**
 * Constructor.
 */
Ubidots::Ubidots(char* token){
    _token = token;
    _client.begin(BAUDRATE);         // WiFly UART Baud Rate: 9600
    maxValues = 5;
    currentValue = 0;
    val = (Value *)malloc(maxValues*sizeof(Value));
}
/** 
 * This function is to read the data from GPRS pins. This function is from Adafruit_FONA library
 * @arg timeout, time to delay until the data is transmited
 * @return replybuffer the data of the GPRS
 */
char* Ubidots::readData(uint16_t timeout){
  uint16_t replyidx = 0;
  char replybuffer[500];
  while (timeout--) {
    if (replyidx >= 500) {
      break;
    }
    while(_client.available()) {
      int c =  _client.read();
      if ((c>31 && c<126) || c=='\n') {
        replybuffer[replyidx] = char(c);
        replyidx++;
      }
      
    }

    if (timeout == 0) {
      break;
    }
    delay(1);
  }
  replybuffer[replyidx] = '\0';  // null term
#ifdef DEBUG_UBIDOTS
  Serial.println(F("Response of GPRS:"));
  Serial.println(replybuffer);
#endif
  while(_client.available()){
    _client.read();
  }
  return replybuffer;
}
/** 
 * This function is to read the data from GPRS pins. This function is from Adafruit_FONA library
 * @arg timeout, time to delay until the data is transmited
 * @return replybuffer the data of the GPRS
 */
bool Ubidots::wifiConnection(const char *ssid, const char *phrase, const char *auth){
    _client.write("AT\r\n");
    if(strstr(readData(1000),"[OK]")==NULL){
#ifdef DEBUG_UBIDOTS
        Serial.println(F("Error with AT+CSQ"));
#endif
            return false;
    }
    _client.write("AT+WSET=0,");
    _client.write(ssid);
    _client.write("\r\n");
    if(strstr(readData(1000),"[OK]")==NULL){
#ifdef DEBUG_UBIDOTS
        Serial.println(F("Error with AT+CSQ"));
#endif
        return false;
    }
    _client.write("AT+WSEC=0,");
    _client.write(auth);
    _client.write(",");
    _client.write(phrase);
    _client.write("\r\n");
    if(strstr(readData(1000),"[OK]")==NULL){
#ifdef DEBUG_UBIDOTS
        Serial.println(F("Error with AT+CSQ"));
#endif
        return false;
    }
    _client.write("AT+WJOIN\r\n");
    if(strstr(readData(8000),"Successfully joined")==NULL){
#ifdef DEBUG_UBIDOTS
        Serial.println(F("Error with AT+CSQ"));
#endif
        return false;
    }
    delay(2000);
    return true;
    
}
/** 
 * This function is to get value from the Ubidots API
 * @arg id the id where you will get the data
 * @return num the data that you get from the Ubidots API
 */
/*float Ubidots::parseValue(String body){
  String rawvalue;
  float num;
  char reply[15];
  uint8_t bodyPosinit;
  uint8_t bodyPosend;
  bodyPosinit = 4 + body.indexOf("\r\n\r\n");
  rawvalue = body.substring(bodyPosinit);
  bodyPosinit = 9 + rawvalue.indexOf("\"value\": ");
  bodyPosend = rawvalue.indexOf(", \"timestamp\"");
  rawvalue.substring(bodyPosinit,bodyPosend).toCharArray(reply, 10);
  num = atof(reply); 
  return num;
}*/
/** 
 * This function is to get value from the Ubidots API
 * @arg id the id where you will get the data
 * @return num the data that you get from the Ubidots API
 */
/*float Ubidots::getValue(char* id){
    _client.write("AT+SCON=O,TCN,");
    _client.write(SERVER);
    _client.write(",");
    _client.write(REMOTE_PORT);
    _client.write(",");
    _client.write(",1\r\n");
    if(strstr(readData(2000),"[CONNECT 0]")==NULL){
#ifdef   DEBUG_UBIDOTS
        Serial.println(F("Error with AT+CSQ"));
#endif
      }
    _client.write("GET /api/v1.6/variables/");
    _client.write(id);
    _client.write("/values?page_size=1 HTTP/1.1");
    _client.write("\r\n");
    _client.write("Host: things.ubidots.com");
    _client.write("\r\n");
    _client.write("User-Agent: User-Agent: Arduino-WizFi250/1.0");
    _client.write("\r\n");
    _client.write("X-Auth-Token: ");
    _client.write(_token);
    _client.write("\r\n");
    _client.write("Connection: close");
    _client.write("\r\n");
    _client.write("\r\n");
    while(strstr(readData(5000),"[OK]")!=NULL){
#ifdef DEBUG_UBIDOTS
        Serial.println(F("Error with AT+CSQ"));
#endif
        delay(2000);
        currentValue = 0;
        return false;
        
    }
  
}*/
/**
 * Add a value of variable to save
 * @arg variable_id variable id to save in a struct
 * @arg value variable value to save in a struct
 */
void Ubidots::add(char *variable_id, double value){
  (val+currentValue)->id = variable_id;
  (val+currentValue)->value_id = value;
  currentValue++;
  if(currentValue>maxValues){
    Serial.println(F("You are sending more than 5 consecutives variables, you just could send 5 variables. Then other variables will be deleted!"));
    currentValue = maxValues;
  }
}
/**
 * Send all data of all variables that you saved
 * @reutrn true upon success, false upon error.
 */
bool Ubidots::sendAll(){
    int i;
    char vals[5];
    char data[350];    
    sprintf(data,"[");
    for(i=0; i<currentValue;){
      dtostrf((val + i)->value_id, 5, 3, vals);
      sprintf(data, "%s{\"variable\": \"%s\", \"value\":\"%s\"}", data, (val + i)->id, vals);
      i++;
      if(i<currentValue){
        sprintf(data,"%s, ", data);
      }
    }
    sprintf(data, "%s]", data);
    Serial.println(data);
    _client.write("AT+SCON=O,TCN,");
    _client.write(SERVER);
    _client.write(",");
    _client.write(REMOTE_PORT);
    _client.write(",");
    _client.write(",1\r\n");
    if(strstr(readData(2000),"[CONNECT 0]")==NULL){
#ifdef   DEBUG_UBIDOTS
        Serial.println(F("Error with AT+CSQ"));
#endif
      }
      sprintf(vals,"%d",strlen(data));
      
    _client.write("POST /api/v1.6/collections/values/?force=true HTTP/1.1");
    _client.write("\r\n");
    _client.write("Host: things.ubidots.com");
    _client.write("\r\n");
    _client.write("User-Agent: User-Agent: Arduino-WizFi250/1.0");
    _client.write("\r\n");
    _client.write("X-Auth-Token: ");
    _client.write(_token);
    _client.write("\r\n");
    _client.write("Content-Type: application/json");
    _client.write("\r\n");
    _client.write("Connection: close");
    _client.write("\r\n");
    _client.write("Content-Length: ");
    _client.write(vals);    
    _client.write("\r\n");
    _client.write("\r\n");
    Serial.println(data);
    _client.write(data);
    //_client.write(data);
    _client.write("\r\n");
    _client.write("\r\n");

    while(strstr(readData(5000),"[OK]")!=NULL){
#ifdef DEBUG_UBIDOTS
        Serial.println(F("Error with AT+CSQ"));
#endif
        delay(2000);
        currentValue = 0;
        return false;
        
    }
    //_client.write("AT+SMGMT=ALL\r\n");
    currentValue = 0;
    delay(4000);
    return true;    
}
