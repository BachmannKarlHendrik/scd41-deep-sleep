#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Arduino.h>
#include "credentials.h"

//Wifi info and debugging led
int LED_BUILTIN = 2;

const char* ssid     = WIFINAME;
const char* password = WIFIPASS;
const char* url = URL; //Endpoint server
const char* tenant = TENANT;
const char* passwordiot = IOTPASS;
String clientId = "KarliESP-SCD41-2";
String command = "";

//Deep sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// SCD4x
const int16_t SCD_ADDRESS = 0x62;

// Helper functions declarations
void reconnect();


void setup() {
  // Start serial output
  Serial.begin(115200);
  while(!Serial);

  // Set deep sleep timer
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  //=========================================
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("WiFi connected");
  //=========================================
  //MQTT
  mqttClient.setServer(url, 1883);
  reconnect();
  //=========================================
  

  // Start measurement
  //=========================================
  // output format
  Serial.println("CO2(ppm)\tTemperature(degC)\tRelativeHumidity(percent)");
  
  // init I2C
  Wire.begin();

  // wait until sensors are ready, > 1000 ms according to datasheet
  delay(1000);
  
  // start scd measurement in one shot mode
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x21);
  Wire.write(0x9d);
  Wire.endTransmission();

  // wait for first measurement to be finished
  delay(5000);

  float co2, temperature, humidity;
  uint8_t data[12], counter;
  
  // read measurement data: 2 bytes co2, 1 byte CRC,
  // 2 bytes T, 1 byte CRC, 2 bytes RH, 1 byte CRC,
  // 2 bytes sensor status, 1 byte CRC
  // stop reading after 12 bytes (not used)
  // other data like  ASC not included
  Wire.requestFrom(SCD_ADDRESS, 12);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }
  
  // floating point conversion according to datasheet
  co2 = (float)((uint16_t)data[0] << 8 | data[1]);
  // convert T in degC
  temperature = -45 + 175 * (float)((uint16_t)data[3] << 8 | data[4]) / 65536;
  // convert RH in %
  humidity = 100 * (float)((uint16_t)data[6] << 8 | data[7]) / 65536;

  Serial.print(co2);
  Serial.print("\t");
  Serial.print(temperature);
  Serial.print("\t");
  Serial.print(humidity);
  Serial.println();

  command = "200,co2Measurement,particles per million,"+String(co2)+",ppm";
  mqttClient.publish("s/us", (char*) command.c_str());
  command = "211,"+String(temperature);
  mqttClient.publish("s/us", (char*) command.c_str());
  command = "200,humidityMeasurement,percent,"+String(humidity)+",%";
  mqttClient.publish("s/us", (char*) command.c_str());

  // Wait for mqtt send
  delay(5000);
  // Disconnect mqtt
  mqttClient.disconnect();

  //Start sleep
  Serial.println("Sleep for 5 minutes.");
  digitalWrite(LED_BUILTIN, LOW);
  esp_deep_sleep_start();
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(),tenant,passwordiot)) {
      Serial.println("Connected");
      String command = "100,"+clientId+",c8y_MQTTdevice";
      mqttClient.publish("s/us", (char*) command.c_str());
      // Connected - do something useful - subscribe to topics, publish messages, etc.
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      // Wait 5 seconds before retrying
      Serial.println("Disconnecting wifi");
      WiFi.disconnect();
      Serial.println("Reconnecting wifi in 5 seconds");
      Serial.print("Connecting to ");
      Serial.println(ssid);
      delay(5000);
      WiFi.reconnect();
      int wifiCounter = 0;
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        wifiCounter +=1;
        if(wifiCounter > 30) {
          Serial.println("Wifi reconnect timed out. Restarting ESP.");
          ESP.restart();
        }
      }
    }
  }
}

void loop() {
}