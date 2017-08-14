/*

  Copyright (c) 2017 @KmanOz and Mitchell A. Cox (mitch at enox dot co dot za)
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  ==============================================================================
  Features:
  
    - Reads and reports temperature via MQTT (DS18 temperature sensor)
    - Controls a relay (on/off) which is connected to heater element for temperature control
    - Overall on/off control (separate relay) 
    - Power measurement via Current Transformer
    - OTA Firmware Upgradable
  ==============================================================================

  **** USE THIS Firmware for: ESPLive (or generic ESP8266 module) and DS18B20 temperature + relay as thermostat ****

*/

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define BUTTON          0                                    // 
#define POWERRELAY      12                                   // Relay that is connected to everything (pump, heater, etc.)
#define HEATERRELAY     13                                    // Relay that is connected to heater
#define LED             5                                   // 
#define TEMPERATUREPIN  14                                   // Pin that the DS18B20 is on
#define WALLSWITCH      4                                    // Overall on/off toggle

#define MAXTEMPERATURE  50                                   //for safety (degrees celsius)

#define MQTT_CLIENT     "JacuzziController"                     // mqtt client_id (Must be unique for each Sonoff)
#define MQTT_SERVER     "192.168.0.6"                        // mqtt server
#define MQTT_PORT       1883                                 // mqtt port
#define MQTT_TOPIC      "home/jacuzzi/"                      // mqtt topic (Must be unique for each Sonoff)
#define MQTT_USER       "user"                               // mqtt user
#define MQTT_PASS       "pass"                               // mqtt password

#define WIFI_SSID       "homewifi"                           // wifi ssid
#define WIFI_PASS       "homepass"                           // wifi password

#define VERSION    "\n\n--------------  Hot Water Controller v1.0  ---------------"

bool rememberRelayState = false;                             // If 'true' remembers the state of the relay before power loss.
bool OTAupdate = false;                                      // (Do not Change)
bool requestRestart = false;                                 // (Do not Change)
bool sendStatus = false;                                     // (Do not Change)
bool tempReport = false;                                     // (Do not Change)
bool thermostatOn = false;

int kUpdFreq = 1;                                            // Update frequency in Mintes to check for mqtt connection
int kRetries = 10;                                           // WiFi retry count. Increase if not connecting to router.
int wallSwitch = 1;                                          // (Do not Change)
int lastWallSwitch = 1;                                      // (Do not Change)
int lastRelayState;                                          // (Do not Change)
int temperatureSP;

OneWire oneWire(TEMPERATUREPIN);
DallasTemperature tempSensor(&oneWire);

unsigned long TTasks;                                        // (Do not Change)
unsigned long count = 0;                                     // (Do not Change)

/* EEPROM Map:
 * 0 = rememberRelayState
 * 1 = temperatureSP
 */

extern "C" { 
  #include "user_interface.h" 
}

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient, MQTT_SERVER, MQTT_PORT);
Ticker btn_timer;

void callback(const MQTT::Publish& pub) {
  if (pub.payload_string() == "stat") {
  }
  else if (pub.payload_string() == "poweron") { //overall power on
    digitalWrite(POWERRELAY, HIGH);
  }
  else if (pub.payload_string() == "poweroff") { //overall power off
    digitalWrite(POWERRELAY, LOW);
  }
  else if (pub.payload_string() == "thermostaton") { //turn on thermostat (does nothing if overall off)
    thermoOn = true;
  }
  else if (pub.payload_string() == "thermostatoff") { //turn off thermostat
    thermoOn = false;
  }
  else if (pub.payload_string() == "reset") {
    requestRestart = true;
  }
  else if (pub.payload_string() == "temp") { //send current and setpoint temperature
    tempReport = true;
  }
  sendStatus = true;
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(POWERRELAY, OUTPUT);
  pinMode(HEATERRELAY, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(WALLSWITCH, INPUT);
  
  digitalWrite(LED, HIGH);
  digitalWrite(POWERRELAY, LOW);

  Serial.begin(115200);

  tempSensor.begin();
  
  EEPROM.begin(8);
  lastRelayState = EEPROM.read(0);
  temperatureSP = EEPROM.read(1);
  
  if (rememberRelayState && lastRelayState == 1) {
     digitalWrite(POWERRELAY, HIGH);
  }
  if (temperatureSP > MAXTEMPERATURE) { 
    temperatureSP = 25; //in case EEPROM goes corrupt set to some low temperature
  }
  
  btn_timer.attach(0.05, button);
  
  mqttClient.set_callback(callback);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  ArduinoOTA.onStart([]() {
    OTAupdate = true;
    blinkLED(LED, 400, 2);
    digitalWrite(LED, HIGH);
    Serial.println("OTA Update Initiated . . .");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Ended . . .s");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    digitalWrite(LED, LOW);
    delay(5);
    digitalWrite(LED, HIGH);
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    blinkLED(LED, 40, 2);
    OTAupdate = false;
    Serial.printf("OTA Error [%u] ", error);
    if (error == OTA_AUTH_ERROR) Serial.println(". . . . . . . . . . . . . . . Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println(". . . . . . . . . . . . . . . Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println(". . . . . . . . . . . . . . . Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println(". . . . . . . . . . . . . . . Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println(". . . . . . . . . . . . . . . End Failed");
  });
  ArduinoOTA.begin();
  
  Serial.println(VERSION);
  Serial.print("\nUnit ID: ");
  Serial.print("esp8266-");
  Serial.print(ESP.getChipId(), HEX);
  Serial.print("\nConnecting to "); Serial.print(WIFI_SSID); Serial.print(" Wifi"); 
  while ((WiFi.status() != WL_CONNECTED) && kRetries --) {
    delay(500);
    Serial.print(" .");
  }
  if (WiFi.status() == WL_CONNECTED) {  
    Serial.println(" DONE");
    Serial.print("IP Address is: "); Serial.println(WiFi.localIP());
    Serial.print("Connecting to ");Serial.print(MQTT_SERVER);Serial.print(" Broker . .");
    delay(500);
    while (!mqttClient.connect(MQTT::Connect(MQTT_CLIENT).set_keepalive(90).set_auth(MQTT_USER, MQTT_PASS)) && kRetries --) {
      Serial.print(" .");
      delay(1000);
    }
    if(mqttClient.connected()) {
      Serial.println(" DONE");
      Serial.println("\n----------------------------  Logs  ----------------------------");
      Serial.println();
      mqttClient.subscribe(MQTT_TOPIC);
      blinkLED(LED, 40, 8);
      digitalWrite(LED, LOW);
    }
    else {
      Serial.println(" FAILED!");
      Serial.println("\n----------------------------------------------------------------");
      Serial.println();
    }
  }
  else {
    Serial.println(" WiFi FAILED!");
    Serial.println("\n----------------------------------------------------------------");
    Serial.println();
  }
  getTemp();
}

void loop() { 
  ArduinoOTA.handle();
  if (OTAupdate == false) {
    mqttClient.loop();
    timedTasks();
    checkStatus();
    checkWallSwitch();
    if (tempReport) {
      getTemp();
    }
  }
}

void blinkLED(int pin, int duration, int n) {             
  for(int i=0; i<n; i++)  {  
    digitalWrite(pin, HIGH);        
    delay(duration);
    digitalWrite(pin, LOW);
    delay(duration);
  }
}

void button() {
  if (!digitalRead(BUTTON)) {
    count++;
  } 
  else {
    if (count > 1 && count <= 40) {   
      digitalWrite(POWERRELAY, !digitalRead(POWERRELAY));
      sendStatus = true;
    } 
    else if (count >40){
      Serial.println("\n\nSonoff Rebooting . . . . . . . . Please Wait"); 
      requestRestart = true;
    } 
    count=0;
  }
}

void checkConnection() {
  if (WiFi.status() == WL_CONNECTED)  {
    if (mqttClient.connected()) {
      Serial.println("mqtt broker connection . . . . . . . . . . OK");
    } 
    else {
      Serial.println("mqtt broker connection . . . . . . . . . . LOST");
      requestRestart = true;
    }
  }
  else { 
    Serial.println("WiFi connection . . . . . . . . . . LOST");
    requestRestart = true;
  }
}

void checkStatus() {
  if (sendStatus) {
    char[64] json;
    bool powerRelayStatus = false;
    bool heaterRelayStatus = false;
    
    if(digitalRead(POWERRELAY) == LOW)  {
      if (rememberRelayState) {
        EEPROM.write(0, 0);
      }
      powerRelayStatus = false;
      Serial.println(PSTR("Power Relay . . . . . . . . . . . . . . . . . . OFF"));
    } else {
      if (rememberRelayState) {
        EEPROM.write(0, 1);
      }
      powerRelayStatus = true;
      Serial.println(PSTR("Power Relay . . . . . . . . . . . . . . . . . . ON"));
    }
    if (rememberRelayState) {
      EEPROM.commit();
    }
    if(digitalRead(HEATERRELAY) == LOW)  {
      heaterRelayStatus = false;
      Serial.println(PSTR("Heater Relay . . . . . . . . . . . . . . . . . . OFF"));
    } else {
      heaterRelayStatus = true;
      Serial.println(PSTR("Heater Relay . . . . . . . . . . . . . . . . . . ON"));
    }

    snprintf_P(json,sizeof(json), PSTR("{\"powerRelay\":\"%s\",\"heaterRelay\":\"%s\",\"thermostat\":\"%s\"}"), 
        powerRelayStatus ? "on" : "off", heaterRelayStatus ? "on" : "off", thermostatOn ? "on" : "off");
    mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/stat", json).set_retain().set_qos(1));
    sendStatus = false;
  }
  if (requestRestart) {
    blinkLED(LED, 400, 4);
    ESP.restart();
  }
}

void checkWallSwitch() {
  wallSwitch = digitalRead(WALLSWITCH);
  if (wallSwitch != lastWallSwitch) {
    digitalWrite(POWERRELAY, !digitalRead(POWERRELAY));
    sendStatus = true;
  }
  lastWallSwitch = wallSwitch;
}

void getTemp() {
  char[64] json;
  char[6] temperatureStr;
  float temperature;
  
  Serial.print(PSTR("Temperature . . . . . . . . . . . . . . . . . "));

  //get the temperature...
  tempSensor.requestTemperatures();
  //we only expect to have one sensor (for now)
  temperature = tempSensor.getTempCByIndex(0);

  snprintf_P(json,sizeof(json), PSTR("{\"tempCV\":\"%2.1f\",\"tempSP\":\"%u\"}"), 
        temperature, temperatureSP);
  mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/temp", message_buff).set_retain().set_qos(1));
  
  Serial.println("OK");
  tempReport = false;
}

void doThermostat() {
  if (thermostatOn) {
    
  }
}

void timedTasks() {
  //this happens every kUpdFreq minutes
  if ((millis() > TTasks + (kUpdFreq*60000)) || (millis() < TTasks)) { 
    TTasks = millis();
    checkConnection();
    doThermostat();
    tempReport = true;
  }
}
