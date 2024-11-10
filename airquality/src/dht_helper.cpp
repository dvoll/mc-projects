#include <Arduino.h>
#include <ArduinoOTA.h>
#include <tuple>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <MHZ19.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>

using namespace std;

tuple<float, float, float> bmeReadSensorValues(Adafruit_Sensor *bme_temp, Adafruit_Sensor *bme_humidity, Adafruit_Sensor *bme_pressure)
{
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");

    Serial.print(F("Humidity = "));
    Serial.print(humidity_event.relative_humidity);
    Serial.println(" %");

    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");

    Serial.println();
    delay(1000);
    return make_tuple(temp_event.temperature, humidity_event.relative_humidity, pressure_event.pressure);
}
// tuple<float, float> dhtReadSensorValues(DHT_Unified &dht)
// {
//     sensors_event_t event;

//     dht.temperature().getEvent(&event);
//     float tempCorrected = 0.0;
//     float humdCorrected = 0.0;
//     if (isnan(event.temperature))
//     {
//         Serial.println(F("Error reading temperature!"));
//     }
//     else
//     {
//         tempCorrected = event.temperature - 2;
//         Serial.print(F("Temperature: "));
//         Serial.print(tempCorrected);
//         Serial.println(F("°C"));
//     }

//     dht.humidity().getEvent(&event);
//     if (isnan(event.relative_humidity))
//     {
//         Serial.println(F("Error reading humidity!"));
//     }
//     else
//     {
//         humdCorrected = event.relative_humidity + 2;
//         Serial.print(F("Humidity: "));
//         Serial.print(humdCorrected);
//         Serial.println(F("%"));
//     }
//     return make_tuple(tempCorrected, humdCorrected);
// }

// void dhtSetupSensor(DHT_Unified& dht)
// {
//     // Initialize device.
//     dht.begin();
//     // Print temperature sensor details.
//     sensor_t sensor;
//     dht.temperature().getSensor(&sensor);
//     Serial.println(F("------------------------------------"));
//     Serial.println(F("Temperature Sensor"));
//     Serial.print(F("Sensor Type: "));
//     Serial.println(sensor.name);
//     Serial.print(F("Driver Ver:  "));
//     Serial.println(sensor.version);
//     Serial.print(F("Unique ID:   "));
//     Serial.println(sensor.sensor_id);
//     Serial.print(F("Max Value:   "));
//     Serial.print(sensor.max_value);
//     Serial.println(F("°C"));
//     Serial.print(F("Min Value:   "));
//     Serial.print(sensor.min_value);
//     Serial.println(F("°C"));
//     Serial.print(F("Resolution:  "));
//     Serial.print(sensor.resolution);
//     Serial.println(F("°C"));
//     Serial.println(F("------------------------------------"));
//     // Print humidity sensor details.
//     dht.humidity().getSensor(&sensor);
//     Serial.println(F("Humidity Sensor"));
//     Serial.print(F("Sensor Type: "));
//     Serial.println(sensor.name);
//     Serial.print(F("Driver Ver:  "));
//     Serial.println(sensor.version);
//     Serial.print(F("Unique ID:   "));
//     Serial.println(sensor.sensor_id);
//     Serial.print(F("Max Value:   "));
//     Serial.print(sensor.max_value);
//     Serial.println(F("%"));
//     Serial.print(F("Min Value:   "));
//     Serial.print(sensor.min_value);
//     Serial.println(F("%"));
//     Serial.print(F("Resolution:  "));
//     Serial.print(sensor.resolution);
//     Serial.println(F("%"));
//     Serial.println(F("------------------------------------"));
// }

int getCo2(MHZ19 &mhz19)
{
    int co2 = 0;
    co2 = mhz19.getCO2(); // Request CO2 (as ppm)
    Serial.print("MHZ19 CO2 (ppm): ");
    Serial.println(co2);

    int8_t mhz19Temp;
    mhz19Temp = mhz19.getTemperature(); // Request Temperature (as Celsius)
    Serial.print("MHZ19 Temperature (C): ");
    Serial.println(mhz19Temp);
    Serial.println();

    return co2;
}

/**
 * Sensor should be at least 20 minutes powered on before calibration.
 */
void calibrateCo2(MHZ19 &mhz19)
{
    

    // Serial.println("Waiting 2 minutes to stabilize...");
    // delay(120000); //  2 minutes in milliseconds

    Serial.println("Calibrating..");
    mhz19.calibrate(); // Take a reading which be used as the zero point for 400 ppm
}

void setupCo2(MHZ19 &mhz19)
{
    mhz19.autoCalibration(false); // Turn auto calibration ON (OFF autoCalibration(false))
    mhz19.setFilter();            // Values are filtered, and returned value is set to 0. An "errorCode" is set.
    // mhz19.setRange(2000); // Set the range to 2000 ppm (default is 5000 ppm)

    delay(4000);
    // Read sensor details
    Serial.print("Range: ");
    Serial.println(mhz19.getRange());
    Serial.print("Background CO2: ");
    Serial.println(mhz19.getBackgroundCO2());
    Serial.print("Temperature Cal: ");
    Serial.println(mhz19.getTempAdjustment());
    Serial.print("Accuracy: ");
    Serial.println(mhz19.getAccuracy());
    Serial.print("ABC Status: ");
    mhz19.getABC() ? Serial.println("ON") : Serial.println("OFF");
}

void setupOTA(ArduinoOTAClass& arduOta)
{
    // start the WiFi OTA library with internal (flash) based storage
    arduOta.onStart([]() {

        // NOTE: if updating FS this would be the place to unmount FS using FS.end()
        Serial.println("Start updating sketch"); 
    });

    arduOta.onEnd([]()
                     { Serial.println("\nEnd"); });

    arduOta.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

    arduOta.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
        } 
    });
    arduOta.begin();
}

void drawInfo(Adafruit_SSD1306& display, String text)
{
    display.clearDisplay();

    display.setFont(&FreeSans9pt7b);

    // display.setTextSize(2);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner

    display.println("");
    display.println("Info");
    display.println(text);

    display.display();
}