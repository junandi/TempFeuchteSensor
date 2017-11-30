#ifndef LIBS_H
#define LIBS_H

// #define DEBUG

#define WIFIMANAGER
#define ASYNCMQTT
#define HTU_TYP
//#define OTA

//#include <Arduino.h>

// #include <ESP8266WiFi.h>

//#include <ESP8266mDNS.h>

//#include <TimeLib.h>
//#include <WiFiClientSecure.h>
//#include <Task.h>
//#include <ArduinoJson.h>
//#include <FS.h>


#ifdef ASYNCMQTT
//#include <Ticker.h>
#include <AsyncMqttClient.h>
#endif

#ifdef NTP
#include <WiFiUdp.h>
#include <NTPClient.h>
#endif

#ifdef WIFIMANAGER
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#endif

#ifdef HTU_TYP
// #include <brzo_i2c.h>
#include <Wire.h>
#include <SparkFunHTU21D.h>
#endif

#endif
