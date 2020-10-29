extern "C"{
#include "user_interface.h"
}

#include "ESP8266WiFi.h"
#include "Blink.h"
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "Seeed_BME280.h"
#include <Wire.h>
#include <BH1750.h>

#define HOSTNAME "wemos-weather"

#define TOPIC_TEMP "weather/wemos/temp"
#define TOPIC_HUM "weather/wemos/hum"
#define TOPIC_PRESS "weather/wemos/press"
#define TOPIC_LIGHT "weather/wemos/light"
#define TOPIC_VCC "weather/wemos/vcc"

#define VCC_PIN A0

#define LED_PIN D3

#define SENSOR_POWER_PIN D8

BME280 bme;

BH1750 lightMeter;

Blink blinkerInternal(LED_BUILTIN);
Blink blinkerLed(LED_PIN);

ESP8266WiFiMulti wifiMulti;
//IPAddress ip(10, 10, 10, 17);
//IPAddress dns(10, 10, 10, 1);
//IPAddress gw(10, 10, 10, 1);
//IPAddress subnet(255, 255, 255, 0);

//IPAddress mqttServer(192, 168, 1, 66);
IPAddress mqttServer(10, 10, 10, 3);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
const int mqttPort = 1883;
unsigned long mqttLastReconnectAttempt = 0;
wl_status_t lastWiFiState = WL_CONNECTED;

const unsigned long WIFI_WAIT_TIMEOUT = 60 * 5;  // wait for WiFi for 60 secs 
const unsigned long SLEEP_TIME = 10 * 60 * 1e6;  // sleep for 10 mins 

enum DeviceState {
  STATE_INIT,
  STATE_OK,
  STATE_ERROR
};

DeviceState deviceState = STATE_INIT;
DeviceState lastDeviceState = STATE_INIT;

struct MyData {
  float temp;
  float hum;
  float pressure;
  uint16_t light;
  float vcc;
  unsigned long lastWeatherSent;  // last time weather data was sent to cloud
};

MyData data;


void setupBme() {
//  dht.begin();
  if (!bme.init()) {  
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
  }
}

void setupBh1750() {
//  Wire.begin();
  lightMeter.begin();
}

void setupVccMeasurement() {
  pinMode(A0, INPUT);
}

void setupWiFi() {
  WiFi.forceSleepWake();
  delay(1);

  // Bring up the WiFi connection
  WiFi.mode(WIFI_STA);
//  WiFi.printDiag(Serial);
//  WiFi.config(ip, dns, gw, subnet); 

//  WiFi.begin("intelis", "password");

  // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
  WiFi.persistent(false);

  String hostname(HOSTNAME);
  WiFi.hostname(hostname);
  Serial.print(F("Hostname: "));
  Serial.println(hostname.c_str());

  wifiMulti.addAP("intelis", "password");
  wifiMulti.addAP("KOLDA_WLAN2", "password");
}

void setupMqtt() {
  mqttClient.setServer(mqttServer, mqttPort);
}

bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    deviceState = STATE_OK;
    return true;
  }

  String hostname(HOSTNAME);
  WiFi.hostname(hostname);
  
  if (wifiMulti.run() != WL_CONNECTED) {
//  if (WiFi.status() != WL_CONNECTED) {
    
    if (lastWiFiState == WL_CONNECTED) {
      Serial.print(F("Connecting to WiFi"));
      deviceState = STATE_ERROR;
    }
    Serial.print(F("."));
    lastWiFiState = WiFi.status();
  } else {
    if (lastWiFiState != WL_CONNECTED) {
      Serial.println(F("WiFi connected"));
      Serial.print(F("IP address: "));
      Serial.println(WiFi.localIP().toString().c_str());
      Serial.print(F("WiFi connected in: "));
      Serial.println(millis());
      deviceState = STATE_OK;
    }
    lastWiFiState = WiFi.status();
    return true;
  }
  return false;
}

void checkDeviceState() {
  if (deviceState != lastDeviceState) {
    Serial.print(F("Device state changed - NEW: "));
    Serial.print(deviceState);
    Serial.print(F(", OLD: "));
    Serial.println(lastDeviceState);
/*
    if (deviceState == STATE_OK) {
      blinkerLed.init({100, 200, 100, 3000}, 0); // heartbeat
    } else {
      blinkerLed.init({10, 50}, 0); // blink quickly
    }
    blinkerLed.start();
*/
    lastDeviceState = deviceState;
  }
}

void checkWifiConnected() {
  // restart if wifi not connected for some time
  for (int i = 0; i < WIFI_WAIT_TIMEOUT; i++) { // timeout checking for WiFi connection
    if (connectWiFi()) {
      return; // connected
    } else {
      delay(200);
    }
  }
  deviceState = STATE_ERROR;
  Serial.print(F("WiFi not found"));
  ESP.restart(); // restart
  delay(1000);
}

void checkMqttConnected() {
  if (!mqttClient.connected() && lastWiFiState == WL_CONNECTED) {
    mqttReconnect();
  }
}

void mqttReconnect() {
  if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) {
    deviceState = STATE_ERROR;
    unsigned long now = millis();
    if (now - mqttLastReconnectAttempt > 5000) {
      mqttLastReconnectAttempt = now;
      // Attempt to reconnect
      Serial.println(F("Connecting to MQTT..."));
      // Create a random client ID
      String clientId = "WeMosClient-";
      clientId += String(random(0xffff), HEX);
      // Attempt to connect
      if (mqttClient.connect(clientId.c_str())) {
        Serial.println(F("MQTT connected"));
        mqttLastReconnectAttempt = 0;
        deviceState = STATE_OK;
      } else {
        Serial.print(F("MQTT connection failed,rc=: "));
        Serial.println(mqttClient.state());
      }
    }
  } else {
    // Client connected
    mqttClient.loop();
  }
}

void readWeather(MyData& data) {
  for (int i=0; i<20;i++) {
    float tempCorrection = 0.0;

    float t = bme.getTemperature() + tempCorrection;
    float p = bme.getPressure();
    float h = bme.getHumidity();
    float alt = bme.calcAltitude(p);

    Serial.print("T: ");
    Serial.print(t);
    Serial.print(", H: ");
    Serial.print(h);
    Serial.print(", P: ");
    Serial.print(p);
    Serial.print(", ALT: ");
    Serial.println(alt);

    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from BME sensor!"));
      delay(200);
    } else {
      data.temp = t;
      data.hum = h;
      data.pressure = p;
      break;
    }
  }
}

void readVcc(MyData& data) {
  unsigned int raw = analogRead(A0);
  Serial.print(F("Raw VCC: "));
  Serial.println(raw);
  
  float vcc = raw / 1023.0;
  vcc = vcc * 4.2;

  if (isnan(vcc)) {
    Serial.println(F("Failed to read VCC!"));
  } else {
    data.vcc = vcc;
    Serial.print(F("VCC: "));
    Serial.println(data.vcc);
  }
}

uint16_t readLight(MyData& data) {
  uint16_t lux = lightMeter.readLightLevel();
  if (isnan(lux)) {
    Serial.print(F("Failed to read LIGHT!"));
  } else {
    data.light = lux;
    Serial.print(F("Light: "));
    Serial.println(data.light);
  }
}

String createJson(String topic, String meter, float value) {
  // {"topic" : "weather", "meter" : "temp", "dev" : "wemos-weather", "value" : 0.01}
  String result = "{\"topic\" : \"";
  result += topic;
  result += "\", \"meter\" : \"";
  result += meter;
  result += "\", \"dev\" : \"";
  result += HOSTNAME;
  result += "\", \"value\" : ";
  result += value;
  result += "}";
  return result;
}

void sendWeather(MyData& data) {
  readWeather(data);
  readLight(data);
  readVcc(data);
    
  mqttPublish(TOPIC_TEMP, createJson("weather", "temp", data.temp));
  mqttPublish(TOPIC_HUM, createJson("weather", "hum", data.hum));
  mqttPublish(TOPIC_PRESS, createJson("weather", "press", data.pressure));
  mqttPublish(TOPIC_LIGHT, createJson("weather", "light", data.light));
  mqttPublish(TOPIC_VCC, createJson("energy", "vcc", data.vcc));
  data.lastWeatherSent = millis();
  blinkerInternal.init({0, 50, 200, 50, 10}, 1); // double blink
  blinkerInternal.start();
}

bool mqttPublish(char *topic, String msg) {
  if (mqttClient.connected()) {
    if (mqttClient.publish(topic, msg.c_str())) {
      Serial.print(F("MQTT data sent: "));
      Serial.println(msg.c_str());
    } else {
      Serial.println(F("MQTT data NOT SENT!"));
    }

    blinkerInternal.init({0, 50, 10}, 1); // one blink
    blinkerInternal.start();
    delay(50);
  }
}

void goToSleep(unsigned long sleepTime) {
  Serial.print(F("Going to sleep (ms): "));
  Serial.println(millis());

  digitalWrite(SENSOR_POWER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  WiFi.disconnect();
  delay(10);

//  blinkerLed.stop();
//  blinkerInternal.stop();

  Serial.println(F("Ready to sleep."));
  
  // WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up (to prevent current spikes on power on)
//  ESP.deepSleep(sleepTime, WAKE_RF_DISABLED);
  ESP.deepSleep(sleepTime);
  delay(100);
}

void setup() {
  // power on sensors
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);
  
  // start with WiFi OFF to get some mA saved
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  Serial.begin(9600);

  Serial.println(F("Starting..."));
  
  // WeMos has inverse LED (LOW - on, HIGH - no off)
  blinkerInternal.inverse(true);

  setupVccMeasurement();
  setupBme();
  setupBh1750();
  
  setupWiFi();
  setupMqtt();

  readLight(data);

  deviceState = STATE_ERROR;
}

void loop() {
  checkWifiConnected();
  checkMqttConnected();
  checkDeviceState();

  if (mqttClient.connected() && (millis() - data.lastWeatherSent > 2000)) {
    sendWeather(data);
    delay(100); // needed to properly send MQTT messages
    goToSleep(SLEEP_TIME);
  }
}
