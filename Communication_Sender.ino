#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xD0, 0x30, 0x30}; //--> REPLACE WITH THE MAC Address of your receiver / ESP32 Receiver.

//----------------------------------------Variables to accommodate the data to be sent.
int send_rnd_val_1;
int send_rnd_val_2;
int coolDown;
//////////////////////////////////////// Debounce button /////////////////
//int runn = 0;
int Button[4] = {2,4,5,18};
//bool reading[4];
//bool buttonstate[4] = {LOW,LOW,LOW,LOW};
//bool lastbuttonstate[4] = {LOW,LOW,LOW,LOW};
//unsigned long long int lastDebouncetime[4];
//////initfunctions

int debounce(int pin) {
  const int delay_ms = 50; // debounce time in milliseconds
  static int last_readings[4] = {0, 0, 0, 0}; // last 4 readings of each button
  int reading = digitalRead(Button[pin]); // read the current state of the button
  if (reading != last_readings[pin]) { // if the current state is different from the last 4 readings
    delay(delay_ms); // wait for the debounce time
    reading = digitalRead(Button[pin]); // read the current state again
    if (reading != last_readings[pin]) { // if it's still different
      last_readings[pin] = reading; // update the last readings
      return reading; // return the current state
    }
  }
  return -1; // return -1 if the state hasn't changed or has been debounced
}


//----------------------------------------

String success;

//----------------------------------------Structure Pump Relay_1 to send data
// Must match the receiver structure
typedef struct struct_Pump {
    int rnd_1;
} struct_Pump;

struct_Pump send_Data;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Switch Pump Success :)";
  }
  else{
    success = "Switch Pump Fail :(";
  }
  Serial.println(">>>>>");
}

void setup() {
  Serial.begin(115200);
  for(int i=0;i<=3;i++){
    pinMode(Button[i],INPUT);
  }
  WiFi.mode(WIFI_STA); //--> Set device as a Wi-Fi Station.

  //----------------------------------------Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //----------------------------------------
  
  //----------------------------------------Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  //----------------------------------------
  
  //----------------------------------------Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  //----------------------------------------
  
  //----------------------------------------Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  //----------------------------------------
}

int key[4]={0,0,0,0};
void loop() {
  debounce(0);
  debounce(1);
  debounce(2);
  debounce(3);
/////////////////// Button 1 - Pump_1 ///////////////////////////
  if(digitalRead(Button[0]) == HIGH && key[0]==0){
    if(millis() - coolDown > 80){
    //----------------------------------------Set values to send Pump-1
    send_rnd_val_1 = 1;
    send_Data.rnd_1 = send_rnd_val_1;
    //----------------------------------------
    Serial.println();
    Serial.print(">>>>> ");
    Serial.println("Send BUTTON_1");
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &send_Data, sizeof(send_Data));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
    coolDown = millis();
    key[0]=1;
   }
  }
/////////////////// Button 2 - Pump_2 ///////////////////////////
  if(digitalRead(Button[1]) == HIGH && key[1]==0){
    if(millis() - coolDown > 80){
    //----------------------------------------Set values to send Pump-1
    send_rnd_val_1 = 2;
    send_Data.rnd_1 = send_rnd_val_1;
    //---------------------------------------- 
    Serial.println();
    Serial.print(">>>>> ");
    Serial.println("Send BUTTON_2");
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &send_Data, sizeof(send_Data));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
   key[1]=1;
   coolDown = millis();
   }
  }
/////////////////// Button 3 - Google Sheet ///////////////////////////
  if(digitalRead(Button[2]) == HIGH && key[2]==0){
    if(millis() - coolDown > 70){
    //----------------------------------------Set values to send Pump-1
    send_rnd_val_1 = 3;
    send_Data.rnd_1 = send_rnd_val_1;
    //---------------------------------------- 
      Serial.println("Button 3 : Google Sheets");
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &send_Data, sizeof(send_Data));
      key[2]=1;
    }
   coolDown = millis();
  }
/////////////////// Button 4 - Line Notify ///////////////////////////
  if(digitalRead(Button[3]) == HIGH && key[3]==0){
    if(millis() - coolDown > 70){
    //----------------------------------------Set values to send Pump-1
    send_rnd_val_1 = 4;
    send_Data.rnd_1 = send_rnd_val_1;
    //---------------------------------------- 
      Serial.println("Button 4 : LINE Notify");
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &send_Data, sizeof(send_Data));
      key[3]=1;
    }
   coolDown = millis();
  }
////////------------- Release key Button --------------////////////
   if(digitalRead(Button[0]) == LOW && key[0]==1){
    key[0]=0; 
   }  
   if(digitalRead(Button[1]) == LOW && key[1]==1){
    key[1]=0; 
   }
   if(digitalRead(Button[2]) == LOW && key[2]==1){
    key[2]=0; 
   }  
   if(digitalRead(Button[3]) == LOW && key[3]==1){
    key[3]=0; 
   }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
