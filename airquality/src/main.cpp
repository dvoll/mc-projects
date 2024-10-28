#include <Arduino.h>
#include <MHZ19.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SoftwareSerial.h> //  Remove if using HardwareSerial or non-uno compatible device
#include <Config.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>

#define DHTPIN 14     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)

#define MH_RX_PIN 12      // Rx pin which the MHZ19 Tx pin is attached to
#define MH_TX_PIN 13      // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600 // Device to MH-Z19 Serial baudrate (should not be changed)

#define USER_INPUT_PIN 3

bool configSend = 0;
DHT_Unified dht(DHTPIN, DHTTYPE);

MHZ19 myMHZ19;                           // Constructor for library
SoftwareSerial softwareSerialMH(MH_RX_PIN, MH_TX_PIN); // (Uno example) create device to MH-Z19 serial

WiFiClient espClient;
PubSubClient mqttClient(espClient);
#define MSG_BUFFER_SIZE (204)
char msg[MSG_BUFFER_SIZE];

// char topicNameTempConfig[] = "homeassistant/sensor/sensorAirQuality2Temp/config";
// char topicNameHumdConfig[] = "homeassistant/sensor/sensorAirQuality2Humd/config";
// char topicNameCo2Config[] = "homeassistant/sensor/sensorAirQuality2Co2/config";
// char topicNameState[] = "homeassistant/sensor/sensorAirQuality2/state";
char topicNameState[] = "airquality2/state";

unsigned long getDataTimer = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int lastCo2 = 0;

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

// void send_sonsor_config() {
//     if (!mqttClient.connected())
//     {
//         reconnect();
//     }

//     Serial.println();
//     Serial.println("Send MQTT homeassistant config...");

//     snprintf(msg, MSG_BUFFER_SIZE, "{\"dev_cla\": \"temperature\", \"name\": \"Temperature\", \"stat_t\": \"homeassistant/sensor/sensorAirQuality2/state\", \"unit_of_meas\": \"°C\", \"val_tpl\": \"{{ value_json.temperature}}\", \"uniq_id\": \"dvairqual02-T\" }");
//     Serial.println(msg);
//     mqttClient.publish(topicNameTempConfig, msg);

//     snprintf(msg, MSG_BUFFER_SIZE, "{\"dev_cla\": \"humidity\", \"name\": \"Humidity\", \"stat_t\": \"homeassistant/sensor/sensorAirQuality2/state\", \"unit_of_meas\": \"%%\", \"val_tpl\": \"{{ value_json.humidity}}\", \"uniq_id\": \"dvairqual02-H\" }");
//     Serial.println(msg);
//     mqttClient.publish(topicNameHumdConfig, msg);

//     snprintf(msg, MSG_BUFFER_SIZE, "{\"dev_cla\": \"carbon_dioxide\", \"name\": \"CO2\", \"stat_t\": \"homeassistant/sensor/sensorAirQuality2/state\", \"unit_of_meas\": \"ppm\", \"val_tpl\": \"{{ value_json.co2}}\", \"uniq_id\": \"dvairqual02-C\" }");
//     Serial.println(msg);
//     mqttClient.publish(topicNameCo2Config, msg);

//     Serial.println();
//     mqttClient.disconnect();
// }

void drawTextValues(float temp, float humid, float co2) {
  display.clearDisplay();

  display.setFont(&FreeSans9pt7b);

  // display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  display.println("");
  char buf[64];
  sprintf(buf, "%.0f", temp);
  display.println(buf);
  sprintf(buf, "%.0f", humid);
  display.println(buf);
  sprintf(buf, "%.0f", co2);
  display.println(buf);

  display.display();
}

void setupDisplay() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  display.setRotation(3);
}

void setup()
{
    Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY); // Device to serial monitor feedback

    pinMode(3, INPUT_PULLUP); // Make RX pin input with pullup
    pinMode(USER_INPUT_PIN, INPUT_PULLUP);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    softwareSerialMH.begin(BAUDRATE); // (Uno example) device to MH-Z19 serial start

    myMHZ19.begin(softwareSerialMH);  // *Serial(Stream) reference must be passed to library begin().

    myMHZ19.autoCalibration(false); // Turn auto calibration ON (OFF autoCalibration(false))
    Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");

    setup_wifi();

    delay(200);

    mqttClient.setServer(MQTT_SERVER, 1883);

    setup_sensor();

    delay(200);

    setupDisplay();

    delay(400);

    if (digitalRead(USER_INPUT_PIN) == LOW) {

        digitalWrite(LED_BUILTIN, LOW);
        delay(2000);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
        digitalWrite(LED_BUILTIN, HIGH);

        Serial.println("Waiting 20 minutes to stabilize...");
        /* if you don't need to wait (it's already been this amount of time), remove the 2 lines */
        delay(12e5);    //  20 minutes in milliseconds

        Serial.println("Calibrating..");
        myMHZ19.calibrate();    // Take a reading which be used as the zero point for 400 ppm
    }
}

void loop()
{
    if (!mqttClient.connected())
    {
        reconnect();
    }

    float temp = 0.0;
    float tempCorrected = 0.0;
    float humd = 0.0;
    float humdCorrected = 0.0;
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
        tempCorrected = temp - 1.5;
        Serial.print(F("Temperature: "));
        Serial.print(tempCorrected);
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
        humdCorrected = humd + 1.5;
        Serial.print(F("Humidity: "));
        Serial.print(humdCorrected);
        Serial.println(F("%"));
    }

    co2 = myMHZ19.getCO2(); // Request CO2 (as ppm)

    Serial.print("CO2 (ppm): ");
    Serial.println(co2);

    int8_t Temp;
    Temp = myMHZ19.getTemperature(); // Request Temperature (as Celsius)
    Serial.print("Temperature (C): ");
    Serial.println(Temp);

    if (co2 > 0) {
        snprintf(msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\", \"co2\": \"%i\" }", tempCorrected, humdCorrected, co2);
        lastCo2 = co2;
    } else if (lastCo2 > 0) {
        snprintf(msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\", \"co2\": \"%i\" }", tempCorrected, humdCorrected, lastCo2);
    } else {
        snprintf(msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\" }", tempCorrected, humd);
    }

    drawTextValues(tempCorrected, humdCorrected, co2);

    Serial.print("Publish message: ");
    Serial.println(msg);

    mqttClient.publish(topicNameState, msg);
    mqttClient.disconnect();

    delay(120000);
}
