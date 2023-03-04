#include <Arduino.h>
#include <MHZ19.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SoftwareSerial.h> //  Remove if using HardwareSerial or non-uno compatible device
#include <Config.h>

#define DHTPIN 5     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)

#define RX_PIN 14      // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 12      // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600 // Device to MH-Z19 Serial baudrate (should not be changed)

bool configSend = 0;
DHT_Unified dht(DHTPIN, DHTTYPE);

MHZ19 myMHZ19;                           // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN); // (Uno example) create device to MH-Z19 serial

WiFiClient espClient;
PubSubClient mqttClient(espClient);
#define MSG_BUFFER_SIZE (204)
char msg[MSG_BUFFER_SIZE];

char topicNameTempConfig[] = "homeassistant/sensor/sensorAirQualityTemp/config";
char topicNameHumdConfig[] = "homeassistant/sensor/sensorAirQualityHumd/config";
char topicNameCo2Config[] = "homeassistant/sensor/sensorAirQualityCo2/config";
char topicNameState[] = "homeassistant/sensor/sensorAirQuality/state";

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
    // Initialize device.
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
            if (!configSend)
            {
                Serial.println();
                Serial.println("Send MQTT homeassistant config...");

                snprintf(msg, MSG_BUFFER_SIZE, "{\"dev_cla\": \"temperature\", \"name\": \"Temperature\", \"stat_t\": \"homeassistant/sensor/sensorAirQuality/state\", \"unit_of_meas\": \"°C\", \"val_tpl\": \"{{ value_json.temperature}}\", \"uniq_id\": \"dvairqual01-T\" }");
                Serial.println(msg);
                mqttClient.publish(topicNameTempConfig, msg);

                snprintf(msg, MSG_BUFFER_SIZE, "{\"dev_cla\": \"humidity\", \"name\": \"Humidity\", \"stat_t\": \"homeassistant/sensor/sensorAirQuality/state\", \"unit_of_meas\": \"%%\", \"val_tpl\": \"{{ value_json.humidity}}\", \"uniq_id\": \"dvairqual01-H\" }");
                Serial.println(msg);
                mqttClient.publish(topicNameHumdConfig, msg);

                snprintf(msg, MSG_BUFFER_SIZE, "{\"dev_cla\": \"carbon_dioxide\", \"name\": \"CO2\", \"stat_t\": \"homeassistant/sensor/sensorAirQuality/state\", \"unit_of_meas\": \"ppm\", \"val_tpl\": \"{{ value_json.co2}}\", \"uniq_id\": \"dvairqual01-C\" }");
                Serial.println(msg);
                mqttClient.publish(topicNameCo2Config, msg);

                Serial.println();

                configSend = 1;
            }
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

void setup()
{
    Serial.begin(115200); // Device to serial monitor feedback

    mySerial.begin(BAUDRATE); // (Uno example) device to MH-Z19 serial start

    setup_wifi();

    delay(200);

    mqttClient.setServer(MQTT_SERVER, 1883);

    setup_sensor();

    myMHZ19.begin(mySerial);  // *Serial(Stream) reference must be passed to library begin().

    myMHZ19.autoCalibration(false); // Turn auto calibration ON (OFF autoCalibration(false))
}

void loop()
{
    delay(60000);

    if (!mqttClient.connected())
    {
        reconnect();
    }

    float temp = 0.0;
    float humd = 0.0;
    int co2 = 0;

    // Get temperature event and print its value.
    sensors_event_t event;

    dht.temperature().getEvent(&event);
    if (isnan(event.temperature))
    {
        Serial.println(F("Error reading temperature!"));
    }
    else
    {
        temp = event.temperature;
        Serial.print(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.println(F("°C"));
    }

    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity))
    {
        Serial.println(F("Error reading humidity!"));
    }
    else
    {
        humd = event.relative_humidity;
        Serial.print(F("Humidity: "));
        Serial.print(event.relative_humidity);
        Serial.println(F("%"));
    }

    co2 = myMHZ19.getCO2(); // Request CO2 (as ppm)

    Serial.print("CO2 (ppm): ");
    Serial.println(co2);

    int8_t Temp;
    Temp = myMHZ19.getTemperature(); // Request Temperature (as Celsius)
    Serial.print("Temperature (C): ");
    Serial.println(Temp);

    snprintf(msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\", \"co2\": \"%i\" }", temp, humd, co2);
    Serial.print("Publish message: ");
    Serial.println(msg);

    mqttClient.publish(topicNameState, msg);
}
