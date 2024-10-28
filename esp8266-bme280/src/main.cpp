#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h> //  Remove if using HardwareSerial or non-uno compatible device
#include <Wire.h>
#include <SPI.h>
#include "Config.h"
#include <DHT.h>
#include <DHT_U.h>
#include <iostream>
#include <string>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

bool configSend = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
#define MSG_BUFFER_SIZE (204)
char msg[MSG_BUFFER_SIZE];

// DHT
#define DHTPIN 14     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);


char topicNameTempConfig[] = "homeassistant/sensor/sensorBmeTestTemp/config";
char topicNameHumdConfig[] = "homeassistant/sensor/sensorBmeTestHumd/config";
// char topicNameCo2Config[] = "homeassistant/sensor/sensorBmeTestCo2/config";
char topicNameState[] = "homeassistant/sensor/sensorBmeTest/state";

unsigned long getDataTimer = 0;

void setup_wifi()
{
    // WIFI connection
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.println();
    Serial.println("Connecting to WiFi ..");

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print('.');
        delay(1000);
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void setup_sensor()
{
    // default settings
    Serial.println(F("BME280:"));
    bme.begin(0x76);  
    // You can also pass in a Wire library object like &Wire2
    sensor_t sensor;
    bme.getTemperatureSensor()->getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print(F("Sensor Type: "));
    Serial.println(sensor.name);
    Serial.print(F("Driver Ver:  "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:   "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:   "));
    Serial.print(sensor.max_value);
    Serial.println(F("°C"));
    Serial.print(F("Min Value:   "));
    Serial.print(sensor.min_value);
    Serial.println(F("°C"));
    Serial.print(F("Resolution:  "));
    Serial.print(sensor.resolution);
    Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
    // Print humidity sensor details.
    bme.getHumiditySensor()->getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print(F("Sensor Type: "));
    Serial.println(sensor.name);
    Serial.print(F("Driver Ver:  "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:   "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:   "));
    Serial.print(sensor.max_value);
    Serial.println(F("%"));
    Serial.print(F("Min Value:   "));
    Serial.print(sensor.min_value);
    Serial.println(F("%"));
    Serial.print(F("Resolution:  "));
    Serial.print(sensor.resolution);
    Serial.println(F("%"));
    Serial.println(F("------------------------------------"));
}

void setup_dht_sensor()
{
    // Initialize device.
    Serial.println(F("DHT22:"));
    dht.begin();
    // Print temperature sensor details.
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print(F("Sensor Type: "));
    Serial.println(sensor.name);
    Serial.print(F("Driver Ver:  "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:   "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:   "));
    Serial.print(sensor.max_value);
    Serial.println(F("°C"));
    Serial.print(F("Min Value:   "));
    Serial.print(sensor.min_value);
    Serial.println(F("°C"));
    Serial.print(F("Resolution:  "));
    Serial.print(sensor.resolution);
    Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
    // Print humidity sensor details.
    dht.humidity().getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print(F("Sensor Type: "));
    Serial.println(sensor.name);
    Serial.print(F("Driver Ver:  "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:   "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:   "));
    Serial.print(sensor.max_value);
    Serial.println(F("%"));
    Serial.print(F("Min Value:   "));
    Serial.print(sensor.min_value);
    Serial.println(F("%"));
    Serial.print(F("Resolution:  "));
    Serial.print(sensor.resolution);
    Serial.println(F("%"));
    Serial.println(F("------------------------------------"));
}

void reconnect()
{
    // Loop until we're reconnected
    while (!mqttClient.connected())
    {
        Serial.print("Attempting MQTT connection...");

        // Create a random client ID
        String clientId = "ESPclient-";
        clientId += String(random(0xffff), HEX);

        // Attempt to connect
        if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS))
        {
            Serial.println("connected");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void send_sonsor_config() {
    if (!mqttClient.connected())
    {
        reconnect();
    }

    Serial.println();
    Serial.println("Send MQTT homeassistant config...");

    snprintf(msg, MSG_BUFFER_SIZE, "{\"dev_cla\": \"temperature\", \"name\": \"Temperature\", \"stat_t\": \"homeassistant/sensor/sensorBmeTest/state\", \"unit_of_meas\": \"°C\", \"val_tpl\": \"{{ value_json.temperature}}\", \"uniqe_id\": \"dvbmetst01-T\" }");
    Serial.println(msg);
    mqttClient.publish(topicNameTempConfig, msg);

    snprintf(msg, MSG_BUFFER_SIZE, "{\"dev_cla\": \"humidity\", \"name\": \"Humidity\", \"stat_t\": \"homeassistant/sensor/sensorBmeTest/state\", \"unit_of_meas\": \"%%\", \"val_tpl\": \"{{ value_json.humidity}}\", \"uniqe_id\": \"dvbmetst01-H\" }");
    Serial.println(msg);
    mqttClient.publish(topicNameHumdConfig, msg);

    // snprintf(msg, MSG_BUFFER_SIZE, "{\"dev_cla\": \"carbon_dioxide\", \"name\": \"CO2\", \"stat_t\": \"homeassistant/sensor/sensorBmeTest/state\", \"unit_of_meas\": \"ppm\", \"val_tpl\": \"{{ value_json.co2}}\", \"uniqe_id\": \"dvbmetst01-C\" }");
    // Serial.println(msg);
    // mqttClient.publish(topicNameCo2Config, msg);

    Serial.println();
    mqttClient.disconnect();
}

void setup()
{
    Serial.begin(115200);

    setup_wifi();

    delay(200);

    mqttClient.setServer(MQTT_SERVER, 1883);

    setup_sensor();

    delay(200);

    setup_dht_sensor();

    delay(200);

    if (!configSend) {
        send_sonsor_config();
        configSend = 1;
    }
}

void print_dht_values() {
    Serial.println(F("------------------------------------"));
    Serial.println(F("...................................."));
    Serial.println(F("DHT Values:"));
    // Get temperature event and print its value.
    sensors_event_t event;

    dht.temperature().getEvent(&event);

    if (isnan(event.temperature))
    {
        Serial.println(F("Error reading temperature!"));
    }
    else
    {
        Serial.print(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.println(F("°C"));
        char myMsg[20];
        snprintf(myMsg, MSG_BUFFER_SIZE, "%.2f", event.temperature);
        mqttClient.publish("test/temp/dht", myMsg);
    }

    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity))
    {
        Serial.println(F("Error reading humidity!"));
    }
    else
    {
        Serial.print(F("Humidity: "));
        Serial.print(event.relative_humidity);
        Serial.println(F("%"));
        char myMsg[20];
        snprintf(myMsg, MSG_BUFFER_SIZE, "%.2f", event.relative_humidity);
        mqttClient.publish("test/humd/dht", myMsg);
    }
    Serial.println(F("...................................."));
    Serial.println(F("------------------------------------"));
}

void loop()
{
    // delay(120000);

    if (!mqttClient.connected())
    {
        reconnect();
    }

    float temp = 0.0;
    float humd = 0.0;
    // int co2 = 0;

    // Get temperature event and print its value.
    sensors_event_t event;

    bme.getTemperatureSensor()->getEvent(&event);

    if (isnan(event.temperature))
    {
        Serial.println(F("Error reading temperature!"));
    }
    else
    {
        temp = event.temperature;
        Serial.print(F("Temperature: "));
        Serial.print(temp);
        Serial.println(F("°C"));
        char myMsg[20];
        snprintf(myMsg, MSG_BUFFER_SIZE, "%.2f", event.temperature);
        mqttClient.publish("test/temp/bme", myMsg);
    }

    bme.getHumiditySensor()->getEvent(&event);
    if (isnan(event.relative_humidity))
    {
        Serial.println(F("Error reading humidity!"));
    }
    else
    {
        humd = event.relative_humidity;
        Serial.print(F("Humidity: "));
        Serial.print(humd);
        Serial.println(F("%"));
        char myMsg[20];
        snprintf(myMsg, MSG_BUFFER_SIZE, "%.2f", event.relative_humidity);
        mqttClient.publish("test/humd/bme", myMsg);
    }

    print_dht_values();

    snprintf(msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\" }", temp, humd);

    Serial.print("Publish message: ");
    Serial.println(msg);

    mqttClient.publish(topicNameState, msg);
    mqttClient.disconnect();

    delay(30000);
}
