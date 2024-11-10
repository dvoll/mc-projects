#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <MHZ19.h>
#include <PubSubClient.h>

#include <SoftwareSerial.h>
#include <Config.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>
#include "dht_helper.h"

#define SEALEVELPRESSURE_HPA (1013.25)

#define MH_RX_PIN 12      // Rx pin which the MHZ19 Tx pin is attached to
#define MH_TX_PIN 13      // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600 // Device to MH-Z19 Serial baudrate (should not be changed)

#define USER_INPUT_PIN 3

unsigned long currentMillis = 0;
unsigned long loopInterval = 30000;

unsigned long calibratingDuration = 20 * 60 * 1000;
unsigned long calibratingMillis = 0;
bool isCalibrating = false;

MHZ19 myMHZ19;
SoftwareSerial softwareSerialMH(MH_RX_PIN, MH_TX_PIN);

WiFiClient espClient;
PubSubClient mqttClient(espClient);
#define MSG_BUFFER_SIZE (204)
char msg[MSG_BUFFER_SIZE];

char topicNameState[] = "airquality2/state";
char topicNameSetCalibration[] = "airquality2/calibr/set";

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();


void setupWifi()
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

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    drawInfo(display, "MSG");
    for (unsigned int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    // Switch on the LED if an 1 was received as first character
    if (strcmp(topic, topicNameSetCalibration) == 0)
    {
        if ((char)payload[0] == '1')
        {
            isCalibrating = true;
            calibratingMillis = millis();
            snprintf(msg, MSG_BUFFER_SIZE, "{ \"calibr\": 1 }");
            mqttClient.publish(topicNameState, msg);
        }
    }
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
            mqttClient.subscribe(topicNameSetCalibration, 1);
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
    setupCo2(myMHZ19);

    setupDisplay();

    delay(200);

    drawInfo(display, "...");

    setupWifi();
    mqttClient.setServer(MQTT_SERVER, 1883);
    mqttClient.setCallback(callback);

    setupOTA(ArduinoOTA);

    drawInfo(display, "CON");

    if (bme.begin(0x76))
    {
      bme_temp->printSensorDetails();
      bme_humidity->printSensorDetails();
      bme_pressure->printSensorDetails();
    } else {
      Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    }

    delay(200);

    if (digitalRead(USER_INPUT_PIN) == LOW) {
        calibrateCo2(myMHZ19);
    }
}

void loop()
{
    // check for WiFi OTA updates
    ArduinoOTA.handle();

    if (!mqttClient.connected())
    {
        reconnect();
    }
    mqttClient.loop();

    if (isCalibrating && millis() - calibratingMillis >= calibratingDuration) {
        calibrateCo2(myMHZ19);

        drawInfo(display, "DO NE");
        isCalibrating = false;
        calibratingMillis = 0;

    } else if (isCalibrating)
    {
        drawInfo(display, "CAL");
        snprintf(msg, MSG_BUFFER_SIZE, "{ \"calibr\": 1 }");
    }
    else if (millis() - currentMillis >= loopInterval)
    {
    
        std::tuple<float, float, float> tempHumPres = bmeReadSensorValues(bme_temp, bme_humidity, bme_pressure);

        int co2 = 0;
        if (currentMillis > 0) // Skip first iteration of loop to wait for sensor to stabilize
        {
            co2 = getCo2(myMHZ19);
        }

        if (co2 > 0) {
            snprintf(msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\", \"co2\": \"%i\", \"calibr\": 0}", std::get<0>(tempHumPres), std::get<1>(tempHumPres), co2);
        } else {
            snprintf(msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\", \"calibr\": 0}", std::get<0>(tempHumPres), std::get<1>(tempHumPres));
        }

        drawTextValues(std::get<0>(tempHumPres), std::get<1>(tempHumPres), co2);

        Serial.print("Publish message: ");
        Serial.println(msg);

        mqttClient.publish(topicNameState, msg);

        currentMillis = millis();
        // mqttClient.disconnect();
    }
}
