/*************************************************
      Template for static IP,MQTT &DeepSleep
        Read out a DHT11
https://arduinodiy.wordpress.com/2020/01/23/very-deep-sleep-part-3-mqtt/
 *************************************************/
long SLEEPTIME = 120e6; //2min

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Config.h>

// We make a structure to store connection information
// The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
// so the RTC data structure should be padded to a 4-byte multiple.
struct {
  uint32_t crc32;   // 4 bytes
  uint8_t channel;  // 1 byte,   5 in total
  uint8_t ap_mac[6];// 6 bytes, 11 in total
  uint8_t padding;  // 1 byte,  12 in total
} rtcData;


IPAddress ip(192, 168, 1, 50); // pick your own IP outside the DHCP range of your router
IPAddress gateway(192, 168, 1, 1); //watch out, these are comma's not dots
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);
//---------voor pubsub
WiFiClient espClient;
PubSubClient client(espClient);
//---------Other------
String naam = (__FILE__);     // filenaam
unsigned int batt;
ADC_MODE(ADC_VCC); // vcc uitlezen.
//--------DHT11------
float  humid_hok;
float temp_hok;
#include <DHT.h>
#define DHTTYPE  DHT22
#define DHTPIN 2 // D4
DHT dht(DHTPIN, DHTTYPE); //


void setup() {
  Serial.begin(115200);
  // we disable WiFi, coming from DeepSleep, as we do not need it right away
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay( 1 );
  Serial.println("wifi uit");

  // Try to read WiFi settings from RTC memory
  bool rtcValid = false;
  if ( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) ) {
    // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
    if ( crc == rtcData.crc32 ) {
      rtcValid = true;
    }
  }
  //now do stuff
  Serial.println("do stuff");
  //strip naam
  //strip path van filenaam
  byte p1 = naam.lastIndexOf('\\');
  byte p2 = naam.lastIndexOf('.');
  naam = naam.substring(p1 + 1, p2);
  batt = ESP.getVcc();
  Serial.print("Sketch name ");
  Serial.println(naam);
  Serial.print("Battery Voltage ");
  Serial.println(batt/1023.0F);
  

  // Start connection WiFi
  //Switch Radio back On
  WiFi.forceSleepWake();
  delay( 1 );

  // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings unnecessarily in the flash memory.
  WiFi.persistent( false );
 Serial.println("Start WiFi");
  // Bring up the WiFi connection
  WiFi.mode( WIFI_STA );
  WiFi.config( ip, dns, gateway, subnet );
  //-----------Now we replace the normally used "WiFi.begin();" with a precedure using connection data stored by us
  if ( rtcValid ) {
    // The RTC data was good, make a quick connection
    WiFi.begin( WIFI_SSID, WIFI_PASS, rtcData.channel, rtcData.ap_mac, true );
  }
  else {
    // The RTC data was not valid, so make a regular connection
    WiFi.begin( WIFI_SSID, WIFI_PASS );
  }

  //------now wait for connection
  int retries = 0;
  int wifiStatus = WiFi.status();
  while ( wifiStatus != WL_CONNECTED ) {
    retries++;
    if ( retries == 100 ) {
      // Quick connect is not working, reset WiFi and try regular connection
      WiFi.disconnect();
      delay( 10 );
      WiFi.forceSleepBegin();
      delay( 10 );
      WiFi.forceSleepWake();
      delay( 10 );
      WiFi.begin( WIFI_SSID, WIFI_PASS );
    }
    if ( retries == 600 ) {
      // Giving up after 30 seconds and going back to sleep
      WiFi.disconnect( true );
      delay( 1 );
      WiFi.mode( WIFI_OFF );
      ESP.deepSleep( SLEEPTIME, WAKE_RF_DISABLED );
      return; // Not expecting this to be called, the previous call will never return.
    }
    delay( 50 );
    wifiStatus = WiFi.status();
  }
  //---------

  Serial.println(" WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //-----
  // Write current connection info back to RTC
  rtcData.channel = WiFi.channel();
  memcpy( rtcData.ap_mac, WiFi.BSSID(), 6 ); // Copy 6 bytes of BSSID (AP's MAC address)
  rtcData.crc32 = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
  ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );


  //---------As we are connected to WiFi, begin MQTT connection
  client.setServer("192.168.1.103", 1883); //your MQTT server's IP.Mind you, these are separated by dots again
  //client.setCallback(callback);
  
  //-------Here is where you send data
  Serial.println("Start Sending data");
  sendMQTTmessage();
  //------------------
  
  /* Close WiFi connection */
  //client.stop();

  //----and go to back to sleep
  Serial.println("Go back to sleep");
  WiFi.disconnect( true );
  delay( 1 );

  // WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up
  ESP.deepSleep( SLEEPTIME, WAKE_RF_DISABLED );
}


void loop() {

}

void sendMQTTmessage()
{
  if (!client.connected()) {
    reconnect();
  }
  client.publish("home/sleep/tele/naam", naam.c_str(), false);
  client.publish("home/sleep/tele/batt", String(batt / 1023.0F).c_str(), false);
  client.publish("home/sleep/tele/temp",String(dht.readTemperature()).c_str(),false);//alternatively: read the sensor when wifi off
  client.publish("sleep/sleep/tele/humid",String(dht.readHumidity()).c_str(),false);
  
  /* Close MQTT client cleanly */
  client.disconnect();
}

void reconnect() {
  while (!client.connected())
  {
    String ClientId = "ESP8266";
    ClientId += String(random(0xffff), HEX);
    if (client.connect(ClientId.c_str()))
      //if your MQTT server is protected with a password, use the next line instead of the revious
      //if (client.connect(ClientId.c_str()),mqtt_user,mqtt_password))
    {
      Serial.print("Connected");
      client.publish("home/hok/stat/connection", "OK");
    } else {
      Serial.print("failed, rc= ");
      Serial.print(client.state());
      delay(1000);
    }

  }
}


// the CRC routine
uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while ( length-- ) {
    uint8_t c = *data++;
    for ( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if ( c & i ) {
        bit = !bit;
      }

      crc <<= 1;
      if ( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }

  return crc;
}

