//----------------------------------------Load libraries
#include <esp_now.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>


// WiFi connect timeout per AP. Increase when connecting takes longer.
const uint32_t connectTimeoutMs = 10000;
//////////////// TDS Sensor ////////////////
#define TdsSensorPin 32
#define VREF 1 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;
//----------------------------------------
#define INTERVAL_TIME_PUMP_1 10000 // 10วินาที
#define INTERVAL_TIME_PUMP_2 1500 // 1.5วินาที
#define Pump_1 26
#define Pump_2 27
//----------------------------------------Define variables to store incoming readings
int receive_Button;
int receive_SendSpeadsheet;
int Memory_data_1;
unsigned long time_1 = 0;
unsigned long time_2 = 0;
#define DO_PIN 33
void sendData();

#define VREF 3300    //VREF (mv)
#define ADC_RES 4096 //ADC Resolution

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define pHPin 36 
float analog_pH_value = 0;
float ph_Value;

#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;
String DO_Value;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

  String t;
#define ON_Board_LED 2  //--> Defining an On Board LED, used for indicators when the process of connecting to a wifi router
//
//----------------------------------------SSID dan Password wifi mu gan.
const char* ssid = "CE-ESL"; //--> Nama Wifi / SSID.
const char* password = "ceeslonly"; //-->  Password wifi .
//----------------------------------------

//----------------------------------------Host & httpsPort
const char* host = "script.google.com";
const int httpsPort = 443;
//----------------------------------------
// Initialize DHT sensor.
WiFiClientSecure client; //--> Create a WiFiClientSecure object.

// Timers auxiliar variables
long now = millis();
long lastMeasure = 0;

String GAS_ID = "AKfycbx4B70amh_4hWVnHGY-LieehsfvYmIUm_WxEe7_RuWS3FiJzEaq6Qg56mEegpOQ78fRLg"; //--> spreadsheet script ID

//----------------------------------------

//----------------------------------------Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int rnd_1;
} struct_message;

struct_message receive_Data; //--> Create a struct_message to receive data.
//----------------------------------------

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Callback when data is received
void sendData(String value,float value2, float value3) {
  Serial.println("==========");
  Serial.print("connecting to ");
  Serial.println(host);
  
  //----------------------------------------Connect to Google host
  if (!client.connect(host, httpsPort)) {
    Serial.println("connection failed");
    return;
  }
  //----------------------------------------

  //----------------------------------------Proses dan kirim data  

  String string_DO = value; 
  float string_ph = value2;
  float string_tds = value3;
  String url = "/macros/s/" + GAS_ID + "/exec?DO_Value=" + string_DO + "&ph_Value="+string_ph+ "&tds_Value="+string_tds; //  2 variables 
  Serial.print("requesting URL: ");
  Serial.println(url);

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
         "Host: " + host + "\r\n" +
         "User-Agent: BuildFailureDetectorESP8266\r\n" +
         "Connection: close\r\n\r\n");

  Serial.println("request sent");
  client.stop();
  //----------------------------------------
//  //---------------------------------------
//  while (client.connected()) {
//    String line = client.readStringUntil('\n');
//    if (line == "\r") {
//      Serial.println("headers received");
//      break;
//    }
//  }
//  String line = client.readStringUntil('\n');
//  if (line.startsWith("{\"state\":\"success\"")) {
//    Serial.println("esp8266/Arduino CI successfull!");
//  } else {
//    Serial.println("esp8266/Arduino CI has failed");
//  }
//  Serial.print("reply was : ");
//  Serial.println(line);
//  Serial.println("closing connection");
//  Serial.println("==========");
//  Serial.println();
//  //----------------------------------------

//===============================================
  delay(1000);
}


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&receive_Data, incomingData, sizeof(receive_Data));
  Serial.println();
  Serial.println("<<<<< Receive Data:");
  Serial.print("Bytes received: ");
  Serial.println(len);
  receive_Button = receive_Data.rnd_1;
  Serial.println("Button: ");
  Serial.println(receive_Button);
  Serial.println("Receive Data: ");
  Serial.println("<<<<<");
  
  if(receive_Button == 1){
  digitalWrite(Pump_1,HIGH);
  }
  else if(receive_Button == 2){
  digitalWrite(Pump_2,HIGH);
  }
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ VOID SETUP
void Wificlientsetup(){
          WiFi.begin(ssid, password); //--> Connect to your WiFi router
        Serial.println("");
    digitalWrite(ON_Board_LED, HIGH); //--> 
  
    //----------------------------------------Wait for connection
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      //----------------------------------------Make the On Board Flashing LED on the process of connecting to the wifi router.
      digitalWrite(ON_Board_LED, LOW);
      delay(250);
      digitalWrite(ON_Board_LED, HIGH);
      delay(250);
      //----------------------------------------
    }
    //----------------------------------------
    digitalWrite(ON_Board_LED, HIGH); //--> Turn off the On Board LED when it is connected to the wifi router.
    //----------------------------------------If successfully connected to the wifi router, the IP Address that will be visited is displayed in the serial monitor
    Serial.println("");
    Serial.print("Successfully connected to : ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println();
    //----------------------------------------
  
    client.setInsecure();    WiFi.begin(ssid, password); //--> Connect to your WiFi router
        Serial.println("");
      
    pinMode(ON_Board_LED,OUTPUT); //--> On Board LED port Direction output
    digitalWrite(ON_Board_LED, HIGH); //--> 
  
    //----------------------------------------Wait for connection
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      //----------------------------------------Make the On Board Flashing LED on the process of connecting to the wifi router.
      digitalWrite(ON_Board_LED, LOW);
      delay(250);
      digitalWrite(ON_Board_LED, HIGH);
      delay(250);
      //----------------------------------------
    }
    //----------------------------------------
    digitalWrite(ON_Board_LED, HIGH); //--> Turn off the On Board LED when it is connected to the wifi router.
    //----------------------------------------If successfully connected to the wifi router, the IP Address that will be visited is displayed in the serial monitor
    Serial.println("");
    Serial.print("Successfully connected to : ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println();
    //----------------------------------------
  
    client.setInsecure();
}

void pH_Sensor() {
  analog_pH_value = analogRead(pHPin);
  float voltage = analog_pH_value * (3.3 / 4095.0);
  ph_Value = (3.3 * voltage);
}

void setup() {
  Serial.begin(115200);
  pinMode(Pump_1,OUTPUT);
  pinMode(Pump_2,OUTPUT);
  pinMode(TdsSensorPin, INPUT);
  pinMode(pHPin,INPUT);

  // Connect to WiFi in STA mode
  WiFi.mode(WIFI_STA);
  //----------------------------------------Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //----------------------------------------
  
  esp_now_register_recv_cb(OnDataRecv); //--> Register for a callback 
}

void DO_Sensor(){
  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;

  DO_Value = String(readDO(ADC_Voltage, Temperaturet));
   
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

void loop() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    tdsValue = fmod(tdsValue, 400);
  }
  DO_Sensor();
  pH_Sensor();
  
  if(receive_Button == 1){
      delay(INTERVAL_TIME_PUMP_1);
      digitalWrite(Pump_1,LOW);   
  }
  else if(receive_Button == 2){
      delay(INTERVAL_TIME_PUMP_2);
      digitalWrite(Pump_2,LOW);
    }
  else if(receive_Button == 3){
    Wificlientsetup();
    float tds_Value = tdsValue;
    sendData(DO_Value,ph_Value,tds_Value);
    ESP.restart();
  }
}
