// D1M-WX2-ThingSpeak-OTA-WFM.ino
const int    FW_VERSION = 1000;
const String FW_FILENAME = "D1M-WX2-ThingSpeak-OTA-WFM";

// TODO: find new BME280 library
// TODO: fix adc autocalibrate, prevent out of range

// 2021-08-03 v1000 changed to WX2, reduced from APRS version, changed to LittleFS, BH1750 armborst

/*_____________________________________________________________________________
   Copyright(c) 2018-2021 Karl W. Berger dba IoT Kits https://w4krl.com/iot-kits

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   _____________________________________________________________________________
*/

// *******************************************************
// ******************* INCLUDES **************************
// *******************************************************

// For WiFiManager
#include <LittleFS.h>                 // [builtin] file system for custom parameters
#include <ESP8266WebServer.h>         // [builtin] for captive portal
#include <WiFiManager.h>              // [manager] v2.0.3-alpha by tzapu https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>              // [builtin] Wi-Fi
#include <ArduinoJson.h>              // [manager] v6.15.2 by Benoît Blanchon https://github.com/bblanchon/ArduinoJson

// For general sketch
#include <Wire.h>                     // [builtin] I2C bus
#include <hp_BH1750.h>                // [manager] v1.0.0 by Stefan Armborst https://github.com/Starmbi/hp_BH1750
#include <BME280I2C.h>                // [ZIP]     v3.0.0 by Tyler Glenn https://github.com/finitespace/BME280

// For HTTP OTA
#include <ESP8266HTTPClient.h>        // [builtin] http
#include <WiFiClientSecureBearSSL.h>  // [builtin] https
#include <ESP8266httpUpdate.h>        // [builtin] OTA

// For Double Reset Detector
#include <DoubleResetDetector.h>      // [manager] v1.0.3 by Stephen Denne https://github.com/datacute/DoubleResetDetector

// *******************************************************
// ******* Set true to erase config data *****************
// *******************************************************
// Use only if all else fails.
// If your device is confused or constantly rebooting:
// 1. Set RESET_WIFI = true;
// 2. Upload the firmware:
//    - DO NOT DO ANYTHING ELSE
//    - DO NOT CONFIGURE YOUR DEVICE
//    - Let it run ONCE
// 3. Set RESET_WIFI = false; and reupload.
// 4. Now you can do the normal configuration process.
const boolean RESET_WIFI = false;     // erases WiFI & FS settings

// *******************************************************
// ******************* DEFAULTS **************************
// *******************************************************
//           CHANGEABLE DEFAULTS - CHANGE AT YOUR RISK
const long   SLEEP_INTERVAL = 600;    // must be 15 seconds or more
const int    OTA_SPAN = 60 * 60;      // seconds between OTA checks
const long   MIN_RSSI = -85;          // warning level for low WiFi signal strength
const float  MIN_VCELL = 3.0;         // warning level for low cell voltage

// !!!!!!    DO NOT CHANGE THESE DEFAULTS       !!!!!!
const String THINGSPEAK_SERVER = "api.thingspeak.com";    // ThingSpeak Server
const char   AP_PORTAL_NAME[] = "IoT Kits";               // Captive Portal name
const String CONFIG_FILENAME = "/config.json";            // FS filename
const int    OTA_OFFSET = 32;                             // lower RTC memory used by OTA

// *******************************************************
// ******************* GLOBALS ***************************
// *******************************************************
const float HPA_TO_INHG = 0.0295299830714;  // hPa (mb) to inHg pressure
bool saveConfigFlag = false;
String unitStatus = "";               // holds device error messages
long startTime = millis();            // record time at start of sketch
const int ADC_PIN = A0;               // voltage sense

// user parameter values entered through WiFiManager
const char* wm_elevation;             // elevation in meters
const char* wm_channel;               // ThingSpeak channel ID
const char* wm_write_key;             // ThingSpeak API Write Key

const int USER_PARAMS = 3;            // set to number of user parameters for json memory

float  elevation;
String write_key;

// structure to hold sensor measurements & calculated values
struct
{
  float stationPressure;         // station pressure (hPa) (mb)
  float seaLevelPressure;        // calculated SLP (hPa)
  float celsius;                 // temperature (°C)
  float fahrenheit;              // calculated temperature (°F)
  float humidity;                // relative humidity (%)
  float lightLevel;              // light intensity (lux)
  float cellVoltage;             // volts
  long  wifiRSSI;                // WiFi signal strength (dBm)
  bool  bme280Fail = false;      // BME280 sensor failure flag
  bool  bh1750Fail = false;      // BH1750 sensor failure flag
  bool  lowVcell = false;        // low Vcell alarm flag
  bool  lowRSSI = false;         // low WiFi signal alarm flag
} sensorData;

// The ESP8266 Real Time Clock memory is arranged into blocks of 4 bytes.
// The RTC data structure MUST be padded to a 4-byte multiple.
// Maximum 512 bytes available.
// https://arduino-esp8266.readthedocs.io/en/latest/libraries.html#esp-specific-apis
// Use fixed width types to avoid variations between devices, for example,
// int is two bytes in Arduino UNO and four bytes in ESP8266.
struct
{
  uint32_t  crc32;               // 4 bytes    4 total
  uint16_t  sequence;            // 2 byte,    6 total
  uint8_t   bme280Fail;          // 1 byte,    7 total
  uint8_t   bh1750Fail;          // 1 byte,    8 total
  uint8_t   lowVcell;            // 1 byte,    9 total
  uint8_t   lowRSSI;             // 1 byte,   10 total
  float     timeAwake;           // 4 bytes,  14 total
  float     adc_factor;          // 4 bytes,  18 total
  uint8_t   pad[2];              // 2 bytes,  20 total
} rtcData;

// *******************************************************
// ********* INSTANTIATE DEVICES *************************
// *******************************************************
BME280I2C myBME280;              // barometric pressure / temperature / humidity sensor
hp_BH1750 myBH1750;              // light level sensor
WiFiClient client;               // ThingSpeak
WiFiManager myWiFiMngr;          // captive portal

// drd is reset by stop() function in code, not by timeout
// drd uses rtc memory so must start after rtcData and OTA areas
const int DRD_ADDRESS = OTA_OFFSET + sizeof(rtcData); // OTA offset + rtcData size
DoubleResetDetector drd(10, DRD_ADDRESS);

// *******************************************************
// ******************** SETUP ****************************
// *******************************************************
void setup()
{
  Wire.begin();                  // required for BME280 library
  Serial.begin( 115200 );
  pinMode(LED_BUILTIN, OUTPUT);

  // erase FS config data - used ONLY for testing
  if ( RESET_WIFI )
  {
    LittleFS.format();
  }

  // read configuration from FS json
  Serial.println("\nMounting File System");
  openFS();

  // ***********************************************
  // define & add user parameters to config web page
  // ***********************************************

  //  parameter name (json id, Prompt, user input variable, length)
  myWiFiMngr.setCustomHeadElement("<h1 style=\"color:red; text-decoration: underline\">D1M-WX1 Weather</h1>");

  WiFiManagerParameter custom_elevation("elevation", "Elevation", wm_elevation, 10);
  myWiFiMngr.addParameter( &custom_elevation );

  WiFiManagerParameter custom_text_ts("<p style=\"color:red; font-weight: bold; text-decoration: underline\">Enter your ThingSpeak parameters:</p>");
  myWiFiMngr.addParameter( &custom_text_ts );

  WiFiManagerParameter custom_channel("channel", "Channel ID", wm_channel, 10);
  myWiFiMngr.addParameter( &custom_channel );

  WiFiManagerParameter custom_write_key("write_key", "Write Key", wm_write_key, 20);
  myWiFiMngr.addParameter( &custom_write_key );

  // WiFiManager Callback functions
  myWiFiMngr.setAPCallback( configModeCallback );
  myWiFiMngr.setSaveConfigCallback( saveConfigCallback );

  // reset settings - used for testing ONLY
  if ( RESET_WIFI )
  {
    myWiFiMngr.resetSettings();
  }

  myWiFiMngr.setTimeout( 240 );    // in seconds

  if ( drd.detectDoubleReset() )
  {
    Serial.println("Double Reset Detected");
    digitalWrite(LED_BUILTIN, LOW);
    myWiFiMngr.startConfigPortal( AP_PORTAL_NAME );
  }
  else
  {
    Serial.println("No Double Reset Detected");
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Attempt WiFi connection");
    myWiFiMngr.autoConnect( AP_PORTAL_NAME );
  }

  if ( WiFi.status() != WL_CONNECTED )
  {
    Serial.println("Failed to connect - restarting");
    delay( 3000 );
    ESP.reset();
  }

  // if you get here you have connected to the WiFi

  // get the user's parameters from WiFiManager
  wm_elevation = custom_elevation.getValue();
  wm_channel   = custom_channel.getValue();
  wm_write_key = custom_write_key.getValue();

  // cast char array as String
  write_key = wm_write_key;

  // cast char array as numeric
  elevation = atof( wm_elevation );

  // save the custom parameters to FS
  if ( saveConfigFlag )
  {
    Serial.println("Save new config");
    saveConfig();
    adcCalibrate();
    writeRTCmemory();
  }
  drd.stop();                    // prevent detection on reboot

  // *******************************************************
  // *********** Weather Station Program *******************
  // *******************************************************
  readRTCmemory();               // get APRS sequence number & sensor status
  rtcData.sequence++;            // increment sequence, rollover at 999
  if (rtcData.sequence > 999 )
  {
    rtcData.sequence = 0;
  }
  if ( rtcData.timeAwake > 25 || rtcData.timeAwake < 0 )
  {
    rtcData.timeAwake = 0;
  }
  readSensors();                 // read data into sensorData struct
  printToSerialPort();           // display data on local serial monitor
  // periodically check for an OTA
  // Note: if an OTA update is performed, program will be reset
  if ( rtcData.sequence % ( OTA_SPAN / SLEEP_INTERVAL ) == 0 )
  {
    unitStatus += "OTA check @ seq. " + String(rtcData.sequence);
    checkOTAupdate();            // check for OTA update
  }
  postToThingSpeak();            // send data to ThingSpeak
  writeRTCmemory();              // save sequence & sensor status
  enterSleep( SLEEP_INTERVAL );  // go to low power sleep mode
} //setup()

// *******************************************************
// ******************** LOOP *****************************
// *******************************************************
void loop()
{
  // everything is done in setup()
} // loop()

// *******************************************************
// ******************* readSensors ***********************
// *******************************************************
void readSensors()
{
  if ( myBME280.begin() == true )          // device is OK
  {
    // read pressure, temperature and humidity in one command
    myBME280.read( sensorData.stationPressure, sensorData.celsius,
                   sensorData.humidity, BME280::TempUnit_Celsius, BME280::PresUnit_hPa );

    // calculate fahrenheit
    sensorData.fahrenheit = 1.8 * sensorData.celsius + 32.0;

    // calculate the Sea Level Pressure from the station pressure and temperature
    sensorData.seaLevelPressure = calculateSeaLevelPressure( sensorData.celsius,
                                  sensorData.stationPressure, atof( wm_elevation ) );

    if ( rtcData.bme280Fail == true )     // device was failed
    {
      unitStatus += "BME280 cleared. ";
      rtcData.bme280Fail = false;         // now cleared
    }
  }
  else                                    // device is failed
  {
    if ( rtcData.bme280Fail == false )    // device was OK
    {
      rtcData.bme280Fail = true;
      unitStatus += "BME280 failed. ";    // now failed
    }
  }

  if ( myBH1750.begin( BH1750_TO_GROUND ) == true )
  {
    myBH1750.calibrateTiming();
    // read light level in lux
    myBH1750.start( BH1750_QUALITY_HIGH, 31 );        // max 121,557 lx, resolution 1.85 lx
    sensorData.lightLevel = myBH1750.getLux();
    if ( rtcData.bh1750Fail == true )     // device was failed
    {
      unitStatus += "BH1750 cleared. ";
      rtcData.bh1750Fail = false;         // now ok
    }
  }
  else
  {
    if ( rtcData.bh1750Fail == false )    // device was good
    {
      unitStatus += "BH1750 failed. ";
      rtcData.bh1750Fail = true;          // now failed
    }
  }

  // read analog voltage from the Analog to Digital Converter
  // on D1 Mini this is 0 - 1023 for voltages 0 to 3.2V
  // the D1M-WX1 has an external resistor to extend the range to 5.0 Volts
  // a fudgeFactor corrects for voltage divider component variation
  // as measured by the user in the calbration step

  sensorData.cellVoltage = 5.0 * analogRead( A0 ) * rtcData.adc_factor / 1023.0;
  if ( sensorData.cellVoltage > MIN_VCELL ) // unit is OK
  {
    if ( rtcData.lowVcell == true )         // was failed
    {
      unitStatus += "Vcell cleared. ";
      rtcData.lowVcell = false;             // now cleared
    }
  }
  else                                      // unit is bad
  {
    if ( rtcData.lowVcell == false )        // was good
    {
      unitStatus += "Vcell low. ";
      rtcData.lowVcell = true;              // now failed
    }
  }

  // read WiFi Received Signal Strength Indicator (RSSI)
  sensorData.wifiRSSI = WiFi.RSSI();
  if ( sensorData.wifiRSSI > MIN_RSSI )   // device is OK
  {
    if ( rtcData.lowRSSI == true )        // device was failed
    {
      unitStatus += "RSSI cleared. ";
      rtcData.lowRSSI = false;            // now cleared
    }
  }
  else                                    // device is failed
  {
    if ( rtcData.lowRSSI == false )       // device was good
    {
      unitStatus += "RSSI low. ";
      rtcData.lowRSSI = true;            // now failed
    }
  }
} // readSensors()

// RTC Memory Functions: The ESP8266 internal Real Time Clock has unused memory
// that remains active during the Deep Sleep mode. This sketch stores WiFi connection
// information in RTC memory to speed up connection time.
// *******************************************************
// ******************* Read RTC Memory *******************
// *******************************************************
bool readRTCmemory()
{
  bool rtcValid = false;
  // offset data 32 bytes to avoid OTA area
  if ( ESP.rtcUserMemoryRead( OTA_OFFSET, (uint32_t*)&rtcData, sizeof( rtcData ) ) )
  {
    // Calculate the CRC of what we just read from RTC memory,
    // but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32(((uint8_t*)&rtcData ) + 4, sizeof( rtcData ) - 4 );
    if ( crc == rtcData.crc32 )
    {
      rtcValid = true;
    }
  }
  return rtcValid;
} // readRTCmemory()

// *******************************************************
// ****************** Write RTC Memory *******************
// *******************************************************
void writeRTCmemory()
{
  // offset data 32 bytes to avoid OTA area
  rtcData.bme280Fail = sensorData.bme280Fail;
  rtcData.bh1750Fail = sensorData.bh1750Fail;
  rtcData.lowVcell   = sensorData.lowVcell;
  rtcData.lowRSSI    = sensorData.lowRSSI;
  rtcData.timeAwake  = ( millis() - startTime ) / 1000.0;  // total awake time in seconds
  //rtcData.adc_factor writen on config
  rtcData.crc32      = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );

  ESP.rtcUserMemoryWrite( OTA_OFFSET, (uint32_t*)&rtcData, sizeof( rtcData ) );
} // writeRTCmemory()

// *******************************************************
// ******************** Calculate CRC32 ******************
// *******************************************************
// Cribbed from Bakke. Originated by others.
uint32_t calculateCRC32( const uint8_t *data, size_t length )
{
  uint32_t crc = 0xffffffff;
  while ( length-- )
  {
    uint8_t c = *data++;
    for ( uint32_t i = 0x80; i > 0; i >>= 1 )
    {
      bool bit = crc & 0x80000000;
      if ( c & i )
      {
        bit = !bit;
      }
      crc <<= 1;
      if ( bit )
      {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
} // calculateCRC32()

// *******************************************************
// ******************** ADC Calibrate *******************
// *******************************************************
void adcCalibrate()
{
  // calibrate the ADC if the ADC voltage is within
  // +/- vBand of 4.2 volts
  float adcVal = 5.0 * analogRead( ADC_PIN ) / 1023.0;
  float vBand = 0.25;  // 6% of full charge voltage
  if ( adcVal > ( 4.2 - vBand ) && adcVal < ( 4.2 + vBand ) )
  {
    rtcData.adc_factor = 4.2 / adcVal;
  }
  else
  {
    if ( rtcData.adc_factor < 0.90 || rtcData.adc_factor > 1.10 )
    {
      rtcData.adc_factor = 1.0;
    }
  }
  Serial.print("ADC Val: "); Serial.println(adcVal);
  Serial.print("ADC Factor: "); Serial.println(rtcData.adc_factor);
} //

// *******************************************************
// **************** Check for OTA Updates ****************
// *******************************************************
void checkOTAupdate()
{
  const String FW_URL_BASE = "https://w4krl.com/fota/";
  const String FW_PATH = FW_FILENAME + "/";
  const String FW_VERSION_URL = FW_URL_BASE + FW_PATH + FW_FILENAME + ".version";
  const String FW_IMAGE_URL = FW_URL_BASE + FW_PATH + FW_FILENAME + ".ino.d1_mini.bin";

  std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
  client->setInsecure();  // doesn't need fingerprint!!!!

  HTTPClient https;

  if (https.begin(*client, FW_VERSION_URL))
  {
    // start connection and send HTTP header
    int httpCode = https.GET();
    Serial.printf("[HTTPS] GET code: %d\n", httpCode);
    if ( httpCode > 0 )
    {
      // HTTP header has been sent and Server response header has been handled
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
      {
        String newFWVersion = https.getString();
        int newVersion = newFWVersion.toInt();
        Serial.print(F("Using version: ")); Serial.println(FW_VERSION);
        Serial.print(F("Found version: ")); Serial.println(newVersion);
        if ( newVersion > FW_VERSION )
        {
          unitStatus += " Update to version " + newFWVersion + ".";
          postToThingSpeak();
          writeRTCmemory();
          // PROGRAM WILL RESTART AFTER UPDATE
          ESPhttpUpdate.update( *client, FW_IMAGE_URL );  // must be *client
        }
        else
        {
          unitStatus += " OK. ";
        }
      }
    }
    else
    {
      Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
    }
    https.end();
  }
  else
  {
    Serial.print(F("[HTTPS] Unable to connect\n"));
  }
}

// *******************************************************
// ********** Post data to ThingSpeak ********************
// *******************************************************
void postToThingSpeak()
{
  // assemble and post the data
  if ( client.connect( THINGSPEAK_SERVER, 80 ) == true )
  {
    Serial.println("ThingSpeak Server connected.");
    // declare dataString as a String and initialize with the API_WRITE_KEY
    String dataString = write_key;
    // cocatenate each field onto the end of dataString
    dataString += "&field1=" + String( sensorData.celsius );
    dataString += "&field2=" + String( sensorData.humidity );
    dataString += "&field3=" + String( rtcData.timeAwake );
    dataString += "&field4=" + String( sensorData.seaLevelPressure );
    dataString += "&field5=" + String( sensorData.lightLevel );
    dataString += "&field6=" + String( sensorData.cellVoltage );
    dataString += "&field7=" + String( sensorData.wifiRSSI );
    dataString += "&field8=" + String( sensorData.fahrenheit );
    dataString += "&status=" + unitStatus;
    Serial.println( dataString );   // show ThingSpeak payload on serial monitor

    // find the number of characters in dataString
    String dataStringLength = String(dataString.length());

    // post the data to ThingSpeak
    client.println("POST /update HTTP/1.1");
    client.println("Host: " + THINGSPEAK_SERVER);
    client.println("Connection: close");
    client.println("X-THINGSPEAKAPIKEY: " + write_key);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Content-Length: " + dataStringLength);
    client.println("");
    client.print(dataString);

    Serial.println("ThingSpeak data sent.");
  }
  client.stop();
} // postToThingSpeak()

// *******************************************************
// ************ Print data to the serial port ************
// *******************************************************
void printToSerialPort()
{
  char buf[60];  // must be >1 longer than the printed string
  // copy sensor data into short variables for readability
  float tc = sensorData.celsius;
  float tf = sensorData.fahrenheit;
  float rh = sensorData.humidity;
  float sl = sensorData.seaLevelPressure;
  float si = sl * HPA_TO_INHG;
  float lx = sensorData.lightLevel;
  float vc = sensorData.cellVoltage;
  // buffer to store formatted print string
  // header line
  Serial.println();
  Serial.println(F("   °C    (°F)   RH%  SLP mb    (in)     Lux  Volt"));

  // data line
  sprintf(buf, "%5.1f (%5.1f) %5.1f  %6.1f (%5.2f)  %6.0f  %4.2f", tc, tf, rh, sl, si, lx, vc);
  Serial.println( buf );

  Serial.println( unitStatus );
  Serial.println(F("----------------------------------------------------------------------------"));
} // printToSerialPort()

// *******************************************************
// ***************** Enter Sleep Mode ********************
// *******************************************************
void enterSleep(long sleep)
{
  Serial.print(F("Sleeping for "));
  Serial.print( sleep );
  Serial.println(F(" seconds."));
  delay( 2 );                       // delay to let things settle
  // WAKE_RF_DEFAULT wakes the ESP8266 with Wi-Fi enabled
  ESP.deepSleep(sleep * 1000000L, WAKE_RF_DEFAULT);
} // enterSleep()

// *******************************************************
// *********** WiFi Manager Functions ********************
// *******************************************************
void openFS()
{
  // from wifiManager example AutoConnectWithFSParameters
  // https://github.com/tzapu/WiFiManager/tree/master/examples/AutoConnectWithFSParameters
  if ( LittleFS.begin() )
  {
    if ( LittleFS.exists( CONFIG_FILENAME ) )
    {
      // file exists - read and copy to globals
      File configFile = LittleFS.open( CONFIG_FILENAME, "r" );
      if ( configFile )
      {
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        const size_t capacity = JSON_OBJECT_SIZE( USER_PARAMS ) + 300;
        DynamicJsonDocument doc( capacity );

        deserializeJson( doc, buf.get() );

        wm_elevation = doc["elevation"];
        wm_channel   = doc["channel"];
        wm_write_key = doc["write_key"];
      }
      else
      {
        Serial.println(F("Failed to load json config"));
      }
      configFile.close();
    }
  }
  else
  {
    Serial.println(F("Failed to mount FS"));
  }
  //end read
} // openFS()

void configModeCallback( WiFiManager * myWiFiManager )
{
  Serial.println(F("Entered config mode"));
  Serial.println( WiFi.softAPIP() );
  // add line using myWiFiManager
  drd.stop();
} // configModeCallback()

// callback when there is need to save config
void saveConfigCallback()
{
  Serial.println(F("Config save requested"));
  saveConfigFlag = true;
} // saveConfigCallback()

// saves user parameters in FS
void saveConfig()
{
  const size_t capacity = JSON_OBJECT_SIZE( USER_PARAMS );
  DynamicJsonDocument doc( capacity );
  // copy globals to json
  doc["elevation"]  = wm_elevation;
  doc["channel"]    = wm_channel;
  doc["write_key"]  = wm_write_key;

  File configFile = LittleFS.open( CONFIG_FILENAME, "w" );
  if ( !configFile )
  {
    Serial.println(F("Failed to write config"));
  }
  serializeJsonPretty( doc, Serial );
  Serial.println("");
  serializeJson( doc, configFile );
  configFile.close();
  //end save
} // saveConfig()

// *******************************************************
// Calculate relative sea level pressure from absolute station pressure in hPa
// temperature in °C, elevation in m
// http://www.sandhurstweather.org.uk/barometric.pdf
// http://keisan.casio.com/exec/system/1224575267
// *******************************************************
float calculateSeaLevelPressure(float celsius, float stationPressure, float elevation)
{
  float slP = stationPressure / pow(2.718281828, -(elevation / ((273.15 + celsius) * 29.263)));
  return slP;
} // calculateSeaLevelPressure()

// *******************************************************
// *********************** END ***************************
// *******************************************************
