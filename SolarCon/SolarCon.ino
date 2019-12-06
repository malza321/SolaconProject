#include <Adafruit_Sensor.h>



#include <SPI.h>
#include "WizFi250.h"

#include <DHT.h>
#include <DHT_U.h>

//Ubidots information
#define TOKEN      "BBFF-PmEuOaWQECQVO3btKnpW5AmMf18TDm"
#define Light_val      "5d8f8adc1d8472209d0092c5"
#define Temp_val      "5d8f8adf1d8472209d0092ca"
#define Dust_val      "5d8f9b251d847232de6089d7"
#define Hum_val      "5d8f8aee1d84721f9eed3775"

#define DHTPIN 6     // Digital pin connected to the DHT sensor
#define DHTTYPE    DHT11     // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);
#define FanPIN 5   // 환풍기용 팬

int dust_sensor = A2;   // 미세먼지 핀 번호

float dust_value = 0;  // 센서에서 입력 받은 미세먼지 값
float dustDensityug=0;  // ug/m^3 값을 계산


int fanCount;
int tempCount;
int humdCount;
int dustCount;
int jodoCount;
float temp;
float hum;
int sensor_led = 8;      // 미세먼지 센서 안에 있는 적외선 led 핀 번호
int sampling = 280;    // 적외선 led를 키고, 센서 값을 읽어 들여 미세먼지를 측정하는 샘플링 시간
int waiting = 40;    
float stop_time = 9680;   // 센서를 구동하지 않는 시간

//Parse JSON
#define PARSE       "\"value\""
#define ENDPARSE    ","

uint32_t delayMS;
char ssid[] = "nanai";       // your network SSID (name)
char pass[] = "12341234";        // your network password
int status = WL_IDLE_STATUS;       // the Wifi radio's status

//Hardware Pin status
#define Pin_Light      A0

char server[] = "thing.ubidots.com";

unsigned long lastConnectionTime = 0;         // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 3 * 1000L; // delay between updates, in milliseconds

String rcvbuf;

// Initialize the Ethernet client object
WiFiClient client;

void postData(String VARID, float data);
void printWifiStatus();
char * floatToString(char * outstr, double val, byte precision, byte widthp);

void setup()
{
  // initialize serial for debugging
  Serial.begin(115200);
  Serial.println(F("\r\nSerial Init"));
  dht.begin();
  pinMode(sensor_led,OUTPUT); // 미세먼지 적외선 led를 출력으로 설정
  pinMode(FanPIN, OUTPUT);
  digitalWrite(FanPIN, LOW);     //HIGH가 키는 것임.
  tempCount=0;
  humdCount=0;
  dustCount=0;
  jodoCount=0;
  sensor_t sensor;
  
  WiFi.init();

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println("You're connected to the network");

  printWifiStatus();
}

void loop() {
  delay(1000);
  //measure Temparature
  sensors_event_t event;
 
  
  digitalWrite(sensor_led, LOW);    // LED 켜기
  delayMicroseconds(sampling);   // 샘플링해주는 시간. 
 
  dust_value = analogRead(dust_sensor); // 센서 값 읽어오기
  
  delayMicroseconds(waiting);  // 너무 많은 데이터 입력을 피해주기 위해 잠시 멈춰주는 시간. 

  digitalWrite(sensor_led, HIGH); // LED 끄기
  delayMicroseconds(stop_time);   // LED 끄고 대기  
 
  
  while (client.available()) {
    char c = client.read();
    if ( c != NULL ) {
      if (rcvbuf.length() > 10)
        rcvbuf = "";
      rcvbuf += c;
      Serial.write(c);
    }
  }
    dht.temperature().getEvent(&event);
    temp = event.temperature;
    if(event.temperature >30){
        tempCount=1;
        }
    else {  
        tempCount=0;
    } 
    
    dht.humidity().getEvent(&event);
    hum = event.relative_humidity;  
    if(event.relative_humidity >70){
      humdCount=1;
    }
     else {
      humdCount=0;
    }  
   dustDensityug = (0.17 * (dust_value * (5.0 / 1024)) - 0.1) * 1000;    // 미세먼지 값 계산
  
    if(dustDensityug <= 30.0){       // 대기 중 미세먼지가 좋음 일 때 파란색 출력
     dustCount=0;
    }else if(30.0 < dustDensityug && dustDensityug <= 100.0){      // 대기 중 미세먼지가 보통 일 때 녹색 출력
     dustCount=0;    
    }else if (100.0 < dustDensityug && dustDensityug <= 150.0){    // 대기 중 미세먼지가 나쁨 일 때 노란색 출력
     dustCount=0;        
    }else{                                                     // 대기 중 미세먼지가 매우 나쁨 일 때 빨간색 출력  
     dustCount=0;
    }
    float Light = analogRead(Pin_Light);
    if (Light < 50) {
    jodoCount=0;
  } else if (Light < 200) {
    jodoCount=0;
  } else if (Light < 300) {
    jodoCount=1;
  } else {
    jodoCount=1;
  }
  Serial.println(jodoCount);
     if(dustCount==1 || jodoCount==1){        // 에어컨 키지 않고 패스해버리기
    Serial.println("미세먼지 과다 또는 태양광 없음으로 에어컨 끔");
    digitalWrite(FanPIN, LOW);
    }
    if(dustCount==0 && jodoCount==0){
      if(tempCount==1 || humdCount==1){
        digitalWrite(FanPIN,HIGH);   // HIGH가 팬을 키는 것임
           Serial.println("on!");
     } 
    if(tempCount==0 && humdCount==0){
      digitalWrite(FanPIN,LOW);
        Serial.println("off");
   }
 }
 
 Serial.print("Dust Density [ug/m3]: ");            // 시리얼 모니터에 미세먼지 값 출력
    Serial.println(dustDensityug);
    Serial.print("Light Sensor = ");
    Serial.println(Light);     // the raw analog reading
    Serial.print(F("Temperature: "));
    Serial.println(temp);
    Serial.print(F("Humidity: "));
    Serial.println(hum);
    Serial.println("------------------------------");
    Serial.println();
  if (millis() - lastConnectionTime > postingInterval) {
    Serial.println("ccccc");
    postData(Light_val, Light);
    
    postData(Temp_val, temp);    
       
    postData(Hum_val, event.relative_humidity); 
    
    postData(Dust_val, dustDensityug);
    
    
  }
  rcvbuf = "";
}

void postData(String VARID, float data) {
  uint8_t content_len[6] = {0};
  String TxData;
  char charDATA[20] = "";
  floatToString(charDATA, data, 2, 7 );

  String dataString = "{\"value\": ";
  dataString += charDATA;
  dataString += '}';

  client.stop();
  if (client.connect(server, 80)) {
    Serial.println("send POST");

    // send the HTTP PUT request
    client.print("POST /api/v1.6/variables/");
    client.print(VARID);
    client.println("/values HTTP/1.1");
    client.println("Host: things.ubidots.com");
    client.print("X-Auth-Token: ");
    client.println(TOKEN);
    client.print("Content-Length:");
    client.println(itoa(dataString.length(), (char*)content_len, 10));
    client.println("Content-Type: application/json");
    client.println("Connection: close\r\n");
    client.print(dataString);
    client.println("\r\n");

    // note the time that the connection was made
    lastConnectionTime = millis();
  }
  else {
    // if you couldn't make a connection
    Serial.println("Connection failed");
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

char * floatToString(char * outstr, double val, byte precision, byte widthp)
{
  char temp[16];
  byte i;

  // compute the rounding factor and fractional multiplier
  double roundingFactor = 0.5;
  unsigned long mult = 1;
  for (i = 0; i < precision; i++)
  {
    roundingFactor /= 10.0;
    mult *= 10;
  }
  temp[0] = '\0';
  outstr[0] = '\0';

  if (val < 0.0)
  {
    strcpy(outstr, "-\0");
    val = -val;
  }

  val += roundingFactor;

  strcat(outstr, itoa(int(val), temp, 10)); //prints the int part
  if ( precision > 0)
  {
    strcat(outstr, ".\0"); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision - 1;

    while (precision--)
      mult *= 10;

    if (val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val) - val ) * mult;

    unsigned long frac1 = frac;

    while (frac1 /= 10)
      padding--;

    while (padding--)
      strcat(outstr, "0\0");

    strcat(outstr, itoa(frac, temp, 10));
  }

  // generate space padding
  if ((widthp != 0) && (widthp >= strlen(outstr)))
  {
    byte J = 0;
    J = widthp - strlen(outstr);

    for (i = 0; i < J; i++)
    {
      temp[i] = ' ';
    }

    temp[i++] = '\0';
    strcat(temp, outstr);
    strcpy(outstr, temp);
  }
  return outstr;
}
