// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <Config.h>

bool configSend = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
#define MSG_BUFFER_SIZE	(200)
char msg[MSG_BUFFER_SIZE];

char topicNameTempConfig[] = "homeassistant/sensor/sensorDHT22v1Temp/config";
char topicNameHumdConfig[] = "homeassistant/sensor/sensorDHT22v1Humd/config";
char topicNameState[] = "homeassistant/sensor/sensorDHT22v1/state";

char topicNameTempConfig2[] = "homeassistant/sensor/sensorDHT22v2Temp/config";
char topicNameHumdConfig2[] = "homeassistant/sensor/sensorDHT22v2Humd/config";
char topicNameState2[] = "homeassistant/sensor/sensorDHT22v2/state";

char topicNameTempConfig3[] = "homeassistant/sensor/sensorDHT22v3Temp/config";
char topicNameHumdConfig3[] = "homeassistant/sensor/sensorDHT22v3Humd/config";
char topicNameState3[] = "homeassistant/sensor/sensorDHT22v3/state";

char topicNameTempConfig4[] = "homeassistant/sensor/sensorDHT22v4Temp/config";
char topicNameHumdConfig4[] = "homeassistant/sensor/sensorDHT22v4Humd/config";
char topicNameState4[] = "homeassistant/sensor/sensorDHT22v4/state";

#define DHTPIN 0     // Digital pin connected to the DHT sensor 
#define DHTPIN2 4     // Digital pin connected to the DHT sensor 
#define DHTPIN3 5     // Digital pin connected to the DHT sensor 
#define DHTPIN4 12     // Digital pin connected to the DHT sensor 

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);
DHT_Unified dht2(DHTPIN2, DHTTYPE);
DHT_Unified dht3(DHTPIN3, DHTTYPE);
DHT_Unified dht4(DHTPIN4, DHTTYPE);

void setup_wifi() {
  // WIFI connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.println();
  Serial.println("Connecting to WiFi ..");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_sensor() {
  // Initialize device.
  dht.begin();
  dht2.begin();
  dht3.begin();
  dht4.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Create a random client ID
    String clientId = "ESPclient-";
    clientId += String(random(0xffff), HEX);

    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
      if (!configSend) {
        Serial.println();
        Serial.println("Send MQTT homeassistant config...");

        // snprintf (msg, MSG_BUFFER_SIZE, "{\"device_class\": \"humidity\", \"name\": \"Humidity\", \"state_topic\": \"homeassistant/sensor/sensorTest/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.humidity}}\" }");
        // mqttClient.publish(topicNameHumdConfig, msg);

        snprintf (msg, MSG_BUFFER_SIZE, "{\"device_class\": \"temperature\", \"name\": \"Temperature\", \"state_topic\": \"homeassistant/sensor/sensorDHT22v1/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.temperature}}\" }");
        mqttClient.publish(topicNameTempConfig, msg);

        snprintf (msg, MSG_BUFFER_SIZE, "{\"device_class\": \"humidity\", \"name\": \"Humidity\", \"state_topic\": \"homeassistant/sensor/sensorDHT22v1/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.humidity}}\" }");
        mqttClient.publish(topicNameHumdConfig, msg);

        snprintf (msg, MSG_BUFFER_SIZE, "{\"device_class\": \"temperature\", \"name\": \"Temperature\", \"state_topic\": \"homeassistant/sensor/sensorDHT22v2/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.temperature}}\" }");
        mqttClient.publish(topicNameTempConfig2, msg);

        snprintf (msg, MSG_BUFFER_SIZE, "{\"device_class\": \"humidity\", \"name\": \"Humidity\", \"state_topic\": \"homeassistant/sensor/sensorDHT22v2/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.humidity}}\" }");
        mqttClient.publish(topicNameHumdConfig2, msg);

        snprintf (msg, MSG_BUFFER_SIZE, "{\"device_class\": \"temperature\", \"name\": \"Temperature\", \"state_topic\": \"homeassistant/sensor/sensorDHT22v3/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.temperature}}\" }");
        mqttClient.publish(topicNameTempConfig3, msg);

        snprintf (msg, MSG_BUFFER_SIZE, "{\"device_class\": \"humidity\", \"name\": \"Humidity\", \"state_topic\": \"homeassistant/sensor/sensorDHT22v3/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.humidity}}\" }");
        mqttClient.publish(topicNameHumdConfig3, msg);

        snprintf (msg, MSG_BUFFER_SIZE, "{\"device_class\": \"temperature\", \"name\": \"Temperature\", \"state_topic\": \"homeassistant/sensor/sensorDHT22v4/state\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.temperature}}\" }");
        mqttClient.publish(topicNameTempConfig4, msg);

        snprintf (msg, MSG_BUFFER_SIZE, "{\"device_class\": \"humidity\", \"name\": \"Humidity\", \"state_topic\": \"homeassistant/sensor/sensorDHT22v4/state\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.humidity}}\" }");
        mqttClient.publish(topicNameHumdConfig4, msg);

        configSend = 1;
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);

  Serial.println("");
  Serial.println("Wake up");

  setup_wifi();

  delay(1000);

  mqttClient.setServer(MQTT_SERVER, 1883);

  setup_sensor();
}

float getValue(DHT_Unified sensor) {
  float temp = 0.0;
  // float humd = 0.0;

  // Get temperature event and print its value.
  sensors_event_t event;

  sensor.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    temp = event.temperature;
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  return temp;
}

float getHumidValue(DHT_Unified sensor) {
  // float temp = 0.0;
  float humd = 0.0;

  // Get temperature event and print its value.
  sensors_event_t event;
  // Get humidity event and print its value.
  sensor.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    humd = event.relative_humidity;
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
  return humd;
}

void loop() {
  delay(60000);

  float temp = getValue(dht);
  float humd = getHumidValue(dht);
  bool hasValue = true;

  float temp2 = getValue(dht2);
  float humd2 = getHumidValue(dht2);
  // bool hasValue2 = true;

  float temp3 = getValue(dht3);
  float humd3 = getHumidValue(dht3);
  // bool hasValue3 = true;

  float temp4 = getValue(dht4);
  float humd4 = getHumidValue(dht4);
  // bool hasValue4 = true;

  if (!mqttClient.connected()) {
    reconnect();
  }

  // mqttClient.loop();
  if (hasValue) {
    snprintf (msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\" }", temp, humd);
    Serial.print("Publish message: ");
    Serial.println(msg);
    mqttClient.publish(topicNameState, msg);

    snprintf (msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\" }", temp2, humd2);
    Serial.print("Publish message: ");
    Serial.println(msg);
    mqttClient.publish(topicNameState2, msg);

    snprintf (msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\" }", temp3, humd3);
    Serial.print("Publish message: ");
    Serial.println(msg);
    mqttClient.publish(topicNameState3, msg);

    snprintf (msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\" }", temp4, humd4);
    Serial.print("Publish message: ");
    Serial.println(msg);
    mqttClient.publish(topicNameState4, msg);

  }
}