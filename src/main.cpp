#include "libs.h"
#include "credentials.h"

#define SLEEP_TIME 300e6 // = 300 seconds = 5 minutes
#define VCC_CALIB_FACTOR 969
#define SENSOR_PWR 4

ADC_MODE(ADC_VCC); // to be able to use ESP.getVcc()

uint8_t SDA_PIN = 14;
uint8_t SCL_PIN = 13;

uint8_t MAX_RETRIES = 3;
uint8_t buffer[5];
//uint8_t error = 0;
uint8_t packetsFiredAndNotArrived = 0;
float hum, temp = 0.0;
bool MqttIsConnected = false;
//bool EverythingIsSent = false;
uint8_t retries = 0;
time_t t1 = 0;

const char fail[] PROGMEM = "Fehler!";
const char ok[] PROGMEM = "OK!";

// Read vcc
float readVccCalibrated(){
  uint32 vcc = ESP.getVcc();
  //vcc = vcc / 1024;
  return vcc;
}

char* addDecPointAtPosAndConvertToCharArray(int in, int pos, char* buff){
  sprintf(buff, "%d", in);
  int n = sizeof(in);
  for (int c = n - 1; c >= pos-1; c--){
   buff[c+1] = buff[c];
 }
 buff[pos-1] = '.';
}


#ifdef HTU_TYP
HTU21D HTUSensor;
#endif


#ifdef ASYNCMQTT
AsyncMqttClient mqttClient;
// Ticker mqttReconnectTimer;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
// Ticker wifiReconnectTimer;

// Function to send MQTT messages - returns number of sent messages if successful - otherwise returns 0
uint8_t publishTempHumVccToMQTT(){
  char cBuff[6] = {0};
  Serial.println(F("Sending..."));
  // mqttClient.publish("INSIDE", 1, true, "PUB");
  // Publish temperature
  //Serial.print(" Tmp.: ");
  //Serial.print(temp);
  dtostrf(temp,5,2,cBuff);
  if(mqttClient.publish(F_TEMP, 1, true, cBuff)) {
    packetsFiredAndNotArrived++;
  }
  else{
    return 0;
  }
  // Publish humidity
  //cBuff[5] = 0;
  //Serial.print(" Hum.: ");
  //Serial.print(hum);
  dtostrf(hum,5,2,cBuff);
  if(mqttClient.publish(F_HUM, 1, true, cBuff)) {
    packetsFiredAndNotArrived++;
  }
  else{
    return 0;
  }
  // Publish voltage
  // Serial.print(" VCC: ");
  // Serial.print(readVccCalibrated());
  //dtostrf(readVccCalibrated(),5,2,cBuff);
  int vcc = ESP.getVcc();
  #ifdef _DEBUG
  Serial.println(vcc);
  #endif
  vcc = vcc * 1006 / 1024;
  #ifdef _DEBUG
  Serial.println(vcc);
  #endif
  addDecPointAtPosAndConvertToCharArray(vcc,2,cBuff);

  // itoa(getVcc(),cBuff,10);
  if(mqttClient.publish(F_VCC, 1, true, cBuff)) {
    packetsFiredAndNotArrived++;
  }
  else{
    return 0;
  }
  // Publish time
  //Serial.print(" TIME: ");
  time_t timer_val = millis()-t1;
  //Serial.print(timer_val);
  itoa(timer_val,cBuff,10);
  if(mqttClient.publish(F_TIME, 1, true, cBuff)) {
    packetsFiredAndNotArrived++;
  }
  else{
    return 0;
  }
  //Serial.println("");
  #ifdef _DEBUG
  Serial.println(packetsFiredAndNotArrived);
  #endif
  return packetsFiredAndNotArrived;
}

// void handleMqttMessage(char* topic, char* payload, size_t len){
//   if (strcmp(topic,USER PREAMBLE F_TIMER) == 0){
//     uint8_t val = atoi((char*)payload);
//     Serial.print(F("MQTT -> "));
//     Serial.println(val);
//     if (val == 0){newStateOfSwitch = 0;}
//     else if (val > 0){newStateOfSwitch = 1;}
//     else{newStateOfSwitch = 0;}
//     timer_val = val;
//   }
// }
void connectToWifi() {
  //Serial.println("Connecting to Wi-Fi...");
  //WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}
void connectToMqtt() {
  //Serial.println(F("Connecting to MQTT..."));
  mqttClient.connect();
}
void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  //Serial.println(F("Connected to WiFi."));
  connectToMqtt();
}
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  // Serial.println(F("Disconnected from Wi-Fi."));
  // mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  // wifiReconnectTimer.once(2, connectToWifi);
}
void onMqttConnect(bool sessionPresent) {
  MqttIsConnected = true;
  //Serial.println(F("Connected to MQTT."));
  //Serial.print(F("Session present: "));
  //Serial.println(sessionPresent);
  //mqttClient.publish("INSIDE", 1, true, "CONNECT");
}
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  //Serial.println(F("Disconnected from MQTT."));

  if (reason == AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT) {
    //Serial.println(F("Bad server fingerprint."));
  }

  // if (WiFi.isConnected()) {
  //   mqttReconnectTimer.once(2, connectToMqtt);
  // }
}
//void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
//   Serial.println(F("Subscribe acknowledged."));
//
// }
//void onMqttUnsubscribe(uint16_t packetId) {
//   Serial.println(F("Unsubscribe acknowledged."));
// }
// void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
//
//   handleMqttMessage(topic,payload,len);
//
// }
void onMqttPublish(uint16_t packetId) {
  //Serial.println(F("Publish acknowledged."));
  packetsFiredAndNotArrived--;
  if (packetsFiredAndNotArrived == 0){
    // going to sleep if all messages arrived
    ESP.deepSleep(SLEEP_TIME, WAKE_RF_DEFAULT);
  }
}
#endif


#ifdef WIFIMANAGER
bool shouldSaveConfig = false;
//callback notifying us of the need to save config
void saveConfigCallback() {
  //Serial.println("Should save config");
  shouldSaveConfig = true;
}
#endif


void setup() {
  //t1 = millis();
  #ifdef _DEBUG
  Serial.begin(74880);
  Serial.println(F("Booting..."));
  #endif

  //Serial.printf("Flash size: %d Bytes \n", ESP.getFlashChipRealSize());
  pinMode(SENSOR_PWR, OUTPUT);
  digitalWrite(SENSOR_PWR, HIGH);
  // brzo_i2c_setup(SDA_PIN, SCL_PIN, SCL_STRETCH_TIMEOUT);
  // Serial.println("brzo is set up!...");

  Wire.begin(SDA_PIN, SCL_PIN); // custom i2c ports (SDA, SCL)
  HTUSensor.begin();

  #ifdef ASYNCMQTT
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  //wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(USER, PASS);
  //mqttClient.setKeepAlive(KEEPALIVE);
  //mqttClient.setWill(LW_TOPIC,LW_QOS,LW_RETAIN,LW_PAYLOAD,LW_LENGTH);

  #if ASYNC_TCP_SSL_ENABLED
  mqttClient.setSecure(MQTT_SECURE);
  if (MQTT_SECURE) {
    mqttClient.addServerFingerprint((const uint8_t[])MQTT_SERVER_FINGERPRINT);
  }
  #endif
  #endif

  #ifdef WIFIMANAGER
  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setTimeout(300);
  if(!wifiManager.autoConnect()) {
    //Serial.println(F("Failed to connect and hit timeout..."));
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    //ESP.deepSleep(SLEEP_TIME, WAKE_RF_DEFAULT);
  }
  //Serial.print(F("WiFi connected - IP address: "));
  IPAddress ip = WiFi.localIP();
  //Serial.println(ip);
  #endif
}

void loop() {
  hum = HTUSensor.readHumidity();
  temp = HTUSensor.readTemperature();
  //Serial.printf("hum[%f] | temp[%f] | vcc[%i]: ", hum, temp, ESP.getVcc());
  if (MqttIsConnected){
    //mqttClient.publish("INSIDE", 1, true, "LOOP");
    if (publishTempHumVccToMQTT()){
      //Serial.println(millis()-t1);
      delay(1000);
      ESP.deepSleep(SLEEP_TIME, WAKE_RF_DEFAULT);
    }
    else if (retries < MAX_RETRIES){
      retries ++;
      //Serial.println("let's try again...");
    }
    else{
      //Serial.println("no success... maybe next time...");
      ESP.deepSleep(SLEEP_TIME, WAKE_RF_DEFAULT);
    }
  }
  //uint16_t packetIdPub2 = mqttClient.publish(USER PREAMBLE F_STATE, 1, true, "alive!");
  // put your main code here, to run repeatedly:
}
