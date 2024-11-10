#include <Arduino.h>

std::tuple<float, float, float> bmeReadSensorValues(Adafruit_Sensor *bme_temp, Adafruit_Sensor *bme_humidity, Adafruit_Sensor *bme_pressure);

// std::tuple<float, float> dhtReadSensorValues(DHT_Unified &dht);

// void dhtSetupSensor(DHT_Unified& dht);

int getCo2(MHZ19& mhz19);

void calibrateCo2(MHZ19& mhz19);

void setupCo2(MHZ19& mhz19);

void setupOTA(ArduinoOTAClass& arduOta);

void drawInfo(Adafruit_SSD1306 &display, String text);
