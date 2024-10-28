#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Config.h>

#define DHTPIN 13     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)

#define LED_PIN 4  // LED
#define FAN_PIN 5  // Fan

#define USER_INPUT_PIN 14

void readAndSendData();

bool configSend = 0;
DHT_Unified dht(DHTPIN, DHTTYPE);

WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long lastMsg = 0;
int value = 0;
#define MSG_BUFFER_SIZE (265)
char msg[MSG_BUFFER_SIZE];

// char topicNameTempConfig[] = "printerBoxCtrl/printerBoxCtrlTemp/config";
// char topicNameHumdConfig[] = "printerBoxCtrl/printerBoxCtrlHumd/config";
// char topicNameLedConfig[] = "printerBoxCtrl/printerBoxCtrlLed/config";
// char topicNameFanConfig[] = "homeassistant/switch/printerBoxCtrlFan/config";
char topicNameState[] = "printerctrl/state";
char topicNameSetLed[] = "printerctrl/led/set";
char topicNameSetFan[] = "printerctrl/fan/set";

bool ledIsOn = false;
bool fanIsOn = false;

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

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if (strcmp(topic, topicNameSetLed) == 0)
  {
    if ((char)payload[0] == '1')
    {
      digitalWrite(LED_BUILTIN, LOW);
      ledIsOn = true;
      digitalWrite(LED_PIN, LOW); // Turn the LED on
    }
    else
    {
      digitalWrite(LED_BUILTIN, HIGH);
      ledIsOn = false;
      digitalWrite(LED_PIN, HIGH); // Turn the LED off by making the voltage HIGH
    }
    readAndSendData();
  }

  if (strcmp(topic, topicNameSetFan) == 0)
  {
    if ((char)payload[0] == '1')
    {
      fanIsOn = true;
      digitalWrite(FAN_PIN, LOW); // Turn the LED on
    }
    else
    {
      fanIsOn = false;
      digitalWrite(FAN_PIN, HIGH); // Turn the LED off by making the voltage HIGH
    }
    readAndSendData();
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
      // send_sonsor_config();

      mqttClient.subscribe(topicNameSetLed, 1);
      mqttClient.subscribe(topicNameSetFan, 1);
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
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY); // Device to serial monitor feedback

  // pinMode(3, INPUT_PULLUP); // Make RX pin input with pullup
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(USER_INPUT_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(FAN_PIN, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  setup_wifi();

  delay(200);

  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(callback);

  setup_sensor();

  delay(200);

  // if (!configSend)
  // {
  //   if (!mqttClient.connected())
  //   {
  //     reconnect();
  //   }
  //   configSend = 1;
  // }
}

void readAndSendData() {
  float temp = 0.0;
  float humd = 0.0;
  char ledstate = '0';
  char fanstate = '0';

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

  if (ledIsOn)
  {

    ledstate = '1';
  }

  if (fanIsOn)
  {

    fanstate = '1';
  }

  // TODO: Add led and fan state

  snprintf(msg, MSG_BUFFER_SIZE, "{ \"temperature\": \"%.2f\", \"humidity\": \"%.2f\", \"ledstate\": \"%c\", \"fanstate\": \"%c\" }", temp, humd, ledstate, fanstate);

  Serial.print("Publish message: ");
  Serial.println(msg);

  mqttClient.publish(topicNameState, msg);
}

void loop()
{
  if (!mqttClient.connected())
  {
    reconnect();
  }
  mqttClient.loop();

  unsigned long now = millis();
  if (now - lastMsg > 20000)
  {
    lastMsg = now;
    ++value;
    // snprintf(msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
    readAndSendData();
  }
}
