#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <limits.h>

#include <NTPClient.h> // https://github.com/arduino-libraries/NTPClient
#include <TimeLib.h>   // https://github.com/PaulStoffregen/Time
#include <WiFiUdp.h>   // needed for ntp time

#include "Accounts.h"
#include "helper.h" // some helper classes

#include <SPI.h>                     // this is needed for BME280 library
#include <EnvironmentCalculations.h> // for dewPoint calculation
#include <BME280I2C.h>               // by Tyler Glenn https://github.com/finitespace/BME280
#include <Wire.h>                    // Needed for legacy versions of Arduino.

#define UPDATE_CHECK_INTERVAL 60 // FOTA update interval in seconds
#define AVERAGE_GUST_INTERVAL 3   // calc average gust every 3 seconds and uses the max of these measurements. https://www.wmo-sat.info/oscar/variables/view/205
#define REPORT_DATA_INTERVAL 600  // report every X seconds
//    #define REPORT_TIMES_PER_HOUR 6   //
//    #define CUP_CIRCUMFERENCE 1       // in meters. Experiment with this value (0.67 * 1.5)
//    #define READ_DIR_INTERVAL 1       // read wind direction sensor every 1 sec to calc average
#define CLOCK_SYNC_INTERVAL 3600  // update clock every X seconds

String projectName = "Weather Station";
String currentVersion = "1.1.10";
#define SENDTOSITE // if commented, will not post to sites.

bool sleeping = false;

const int httpsPort = 443;
const int httpPort = 80;

//thingspeak
const char *postSite1 = "api.thingspeak.com";
String postData1 = "/update?key=%ThingspeakKey&field1=%temp&field2=%hum&field3=%pres&field4=%cRain&field5=%heatIdx&field6=%wSpeed&field7=%wGust&field8=%wDir";
const char *ThingspeakSHA1 = "78:60:18:44:81:35:BF:DF:77:84:D4:0A:22:0D:9B:4E:6C:DC:57:2C";

// weatherundergound
const char *postSite2 = "weatherstation.wunderground.com";
String postData2 = "/weatherstation/updateweatherstation.php?ID=%WUndergroundID&PASSWORD=%WUndergroundPSW&dateutc=now&winddir=%wDir&windspeedmph=%wSpeed&windgustmph=%wGust&tempf=%temp&baromin=%pres&dewptf=%dew&humidity=%hum&rainin=%cRain&softwaretype=esp8266&action=updateraw";
const char *WUndergroundSHA1 = "12:DB:BB:24:8E:0F:6F:D4:63:EC:45:DD:5B:ED:37:D7:6F:B1:5F:E5";

//PWSweather.com
const char *postSite3 = "www.pwsweather.com";
String postData3 = "/pwsupdate/pwsupdate.php?ID=%PWSweatherID&PASSWORD=%PWSweatherPSW&dateutc=%now&winddir=%wDir&windspeedmph=%wSpeed&windgustmph=%wGust&tempf=%temp&baromin=%pres&dewptf=%dew&humidity=%hum&rainin=%cRain&softwaretype=esp8266&action=updateraw";
const char *PWSweatherSHA1 = "74:3C:4F:DD:3E:8E:51:40:B4:28:79:7D:6A:B4:30:FE:4A:56:10:BE";

const char *postSite4 = "www.studio19.gr"; // my private site
String postData4 = "/weather/post.php?id=%MySiteID&psw=%MySitePSW&temp=%temp&hum=%hum&pressure=%pres&heatindex=%heatIdx&dewpoint=%dew&windspeed=%wSpeed&windgust=%wGust&winddir=%wDir&windchill=%wChill&rain=%cRain&dayrain=%dRain";
//const char* postSite4 = "xxx.xxx.xxx.xxx";
// String postData4 = "/html/ws/post.php?id=%MySiteID&psw=%MySitePSW&temp=%temp&hum=%hum&pressure=%pres&heatindex=%heatIdx&dewpoint=%dew&windspeed=%wSpeed&windgust=%wGust&winddir=%wDir&windchill=%wChill&rain=%hRain&dayrain=%dRain";

// ===================================================================
// 						NTP
// ===================================================================
const int timeZone = 0; // Central European Time (UTC = 0)
WiFiUDP ntpUDP;
// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, CLOCK_SYNC_INTERVAL * 1000);

// ====================================================================

uint32_t nextFOTACheckTime = 0;
uint32_t nextReportTime = 30000; // 30 secs after boot, send first data
uint32_t nextWindVaneTime = 0;
uint32_t nextWindGustTime = 3000;

BME280I2C bme; // Default : forced mode, standby time = 1000 ms
               // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool BMESensorFound = false;

#define PIN_CLEAR 13

// ================ global variables ==================
bool metric = true;
float temp(NAN), hum(NAN), pres(NAN);
float dewPoint = 0.0f;;
float heatIndex = 0.0f;;
float windSpeed = 0.0f;;
float windGust = 0.0f;;
float windPeak = 0.0f;;
float windChill = 0.0f;
float windDir = 0.0f;;
float rainCurrent = 0.0f;;
float rainHour = 0.0f;;
float rainDay = 0.0f;;

//===========================================================================
// 					Rain Gauge
//===========================================================================

uint8_t RainPin = 12;
volatile uint16_t rainEventCount;
uint32_t lastRainEvent;
float RainScaleInches = 0.011f; // Each pulse is .011 inches of rain (or 0.2794 mm)

void handleRainEvent()
{
  // Count rain gauge bucket tips as they occur
  // Activated by the magnet and reed switch in the rain gauge, attached to input D2
  uint32_t timeRainEvent = millis(); // grab current time

  // ignore switch-bounce glitches less than 10mS after initial edge
  if (timeRainEvent - lastRainEvent < 10)
  {
    return;
  }

  rainEventCount++;              //Increase this minute's amount of rain
  lastRainEvent = timeRainEvent; // set up for next event
}

void initializeRainGauge()
{
  pinMode(RainPin, INPUT);
  rainEventCount = 0;
  lastRainEvent = 0;
  attachInterrupt(RainPin, handleRainEvent, FALLING);
}

float getAndResetRainInches()
{
  float result = RainScaleInches * float(rainEventCount);
  rainEventCount = 0;
  return result;
}

//===========================================================================
//                      Wind Speed (Anemometer)
//===========================================================================

// The Anemometer generates a frequency relative to the windspeed.  1Hz: 1.492MPH, 2Hz: 2.984MPH, etc.
// We measure the average period (elaspsed time between pulses), and calculate the average windspeed since the last recording.

uint8_t AnemometerPin = 14;
float AnemometerScaleMPH = 1.492f; // Windspeed if we got a pulse every second (i.e. 1Hz)
volatile uint32_t WindSpeedPeriodTotal = 0;
volatile uint16_t WindSpeedPeriodSamples = 0;
volatile uint32_t GustPeriod = UINT_MAX;
uint16_t GustAvgSamples = 0;
uint16_t GustAvgTotal = 0;
uint16_t GustTotalSamples = 0;
uint32_t GustTotal = 0;
uint32_t lastAnemoneterEvent = 0;

void handleAnemometerEvent()
{
  // Activated by the magnet in the anemometer (2 ticks per rotation)
  uint32_t currentMillis = millis(); // grab current time
  //If there's never been an event before (first time through), then just capture it
  if (lastAnemoneterEvent != 0)
  {
    // Calculate time since last event
    // todo: check overflow here
    uint32_t period = currentMillis - lastAnemoneterEvent;
    // ignore switch-bounce glitches less than 10mS after initial edge (which implies a max windspeed of 149mph)
    if (period > 10)
    {
      WindSpeedPeriodTotal += period;
      WindSpeedPeriodSamples++;
      // continuosly calc gust
      float tmpWindSpeed = AnemometerScaleMPH * 1000 / float(period);
      if (tmpWindSpeed > windPeak) windPeak = tmpWindSpeed;
      GustAvgSamples++;
      GustAvgTotal += tmpWindSpeed;
    }
  }
  lastAnemoneterEvent = currentMillis; // set up for next event
}

void initializeAnemometer()
{
  pinMode(AnemometerPin, INPUT);
  WindSpeedPeriodTotal = 0;
  WindSpeedPeriodSamples = 0;
  GustPeriod = UINT_MAX; //  The shortest period (and therefore fastest gust) observed
  GustTotal = 0;
  lastAnemoneterEvent = 0;
  attachInterrupt(AnemometerPin, handleAnemometerEvent, FALLING);
}

float getAndResetAnemometerMPH()
{
  if (WindSpeedPeriodSamples == 0)
  {
    return 0;
  }
  // Nonintuitive math:  We've collected the sum of the observed periods between pulses, and the number of observations.
  // Now, we calculate the average period (sum / number of readings), take the inverse and muliple by 1000 to give frequency, and then mulitply by our scale to get MPH.
  // The math below is transformed to maximize accuracy by doing all muliplications BEFORE dividing.
  float result = AnemometerScaleMPH * 1000.0 * float(WindSpeedPeriodSamples) / float(WindSpeedPeriodTotal);
  WindSpeedPeriodTotal = 0;
  WindSpeedPeriodSamples = 0;  
  return result;
}

float getAndResetWindGust()  // we call this every 3 secs in main loop
{
  float result = 0.0f;
  if (GustAvgSamples > 0)
    result = GustAvgTotal / GustAvgSamples;
  GustAvgTotal = 0.0f;
  GustAvgSamples = 0;  
  GustPeriod = UINT_MAX; // not needed
  return result;
}

//===========================================================
// 										Wind Vane
//===========================================================

void initializeWindVane()
{
  return;
}

// For the wind vane, we need to average the unit vector components (the sine and cosine of the angle)
uint8_t WindVanePin = A0;
float windVaneCosTotal = 0.0;
float windVaneSinTotal = 0.0;
uint16_t windVaneReadingCount = 0;
const float DEG2RAD = M_PI / 180.0; // convert degrees to radian

int voltage2degrees(uint16_t v)
{
  if (v > 1022) return 270;      // W            920
  else if (v > 1000) return 315; // NW           850
  else if (v > 913) return 0;   // N            780
  else if (v > 775) return 225; // SW           630
  else if (v > 606) return 45;  // NE           500
  else if (v > 398) return 180; // S            300
  else if (v > 265)  return 135; // SE          200
  else if (v > 140)  return 90;  // E            90
  else return 0;  
}

void captureWindVane()
{
  float windVaneRadians = voltage2degrees(analogRead(WindVanePin)) * DEG2RAD;
  if (windVaneRadians >= 0 && windVaneRadians <= 6.28318531f)  // 0 to 359.9999999 degrees
  {
    windVaneCosTotal += cos(windVaneRadians);
    windVaneSinTotal += sin(windVaneRadians);
    windVaneReadingCount++;
  }
}

float getAndResetWindVaneDegrees()
{
	if (windVaneReadingCount == 0) {
		return 0;
	}
	float avgCos = windVaneCosTotal/float(windVaneReadingCount);
	float avgSin = windVaneSinTotal/float(windVaneReadingCount);
	float result = atan(avgSin/avgCos) * 180.0f / 3.14159f;
	windVaneCosTotal = 0.0;
	windVaneSinTotal = 0.0;
	windVaneReadingCount = 0;
	// atan can only tell where the angle is within 180 degrees.  Need to look at cos to tell which half of circle we're in
	if(avgCos < 0) result += 180.0;
	// atan will return negative angles in the NW quadrant -- push those into positive space.
	if(result < 0) result += 360.0;
    
   return roundf(result * 100) / 100;
}

// ========================================================
// 					BME Sensor
// ========================================================

void initializeBMESensor()
{
  Serial.print(F("Checking for BME280 Sensor... "));
  Wire.begin(); // for BME280. Parameters are pinSDA, pinSCL Defaults are 4,5
  // try to initialize BME280 sensor
  BMESensorFound = bme.begin();

  if (BMESensorFound)
  {
    Serial.println(F("Found!"));
  }
  else
  {
    Serial.println(F("Not found!"));
  }
  delay(1000);
}

bool WiFi_Connected()
{
  bool b = false;
  b = ((WL_CONNECTED == WiFi.status()) && (static_cast<uint32_t>(WiFi.localIP()) != 0));
  if (b)
  {
    Serial.print("WiFi is connected with IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(" MAC: ");
    Serial.println(WiFi.BSSIDstr());
  }
  return b;
}

void restartIfNotConnected()
{
  if (!WiFi_Connected())
  {
    Serial.println("Not Connected to WiFi. Rebooting...");
    //reset and try again,
    ESP.restart();
    delay(1000);
  }
}
// check http://stackoverflow.com/questions/41371156/esp8266-and-post-request

void send2site(const char *host, String data)
{

  WiFiClient client;
  Serial.print("connecting to ");
  Serial.println(host);
  if (!client.connect(host, httpPort))
  {
    Serial.println("connection failed");
    return;
  }

  Serial.print("requesting URL: ");
  Serial.println(data);

  client.print(String("GET ") + data + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: ESP8266\r\n" +
               "Connection: close\r\n\r\n");

  Serial.println("request sent");

  // give host 5 seconds to reply
  unsigned long timeout = millis();
  while (client.available() == 0)
  {
    if (millis() - timeout > 5000)
    {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  while (client.connected())
  {
    String line = client.readStringUntil('\n');
    line += ("\r\n");
    Serial.print(line);
  }
  Serial.println("closing connection");
}

void send2siteSecure(const char *host, const char *fingerprint, String data)
{

  // Use WiFiClientSecure class to create TLS connection
  WiFiClientSecure client;
  Serial.print("connecting to ");
  Serial.println(host);
  if (!client.connect(host, httpsPort))
  {
    Serial.println("connection failed");
    return;
  }

  if (client.verify(fingerprint, host))
  {
    Serial.println("certificate matches");
  }
  else
  {
    Serial.println("certificate doesn't match");
  }

  Serial.print("requesting URL: ");
  Serial.println(data);

  client.print(String("GET ") + data + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: ESP8266\r\n" +
               "Connection: close\r\n\r\n");

  Serial.println("request sent");

  // give host 5 seconds to reply
  unsigned long timeout = millis();
  while (client.available() == 0)
  {
    if (millis() - timeout > 5000)
    {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  while (client.connected())
  {
    String line = client.readStringUntil('\n');
    line += ("\r\n");
    Serial.print(line);
  }
  Serial.println("closing connection");
}


void enableInterrupts()
{
  attachInterrupt(AnemometerPin, handleAnemometerEvent, FALLING);
  attachInterrupt(RainPin, handleRainEvent, FALLING);
}

void disableInterrupts()
{
  detachInterrupt(AnemometerPin);
  detachInterrupt(RainPin);
}

bool wakeUpNow()
{
  if (sleeping) {
    Serial.print("Waking Up: ");
    WiFi.forceSleepWake();
    delay(1);
    uint8_t i = 0;
    while (!WiFi_Connected() & (i++ < 20))
    {
      Serial.print("w");
      delay(1000);
    }
  }
  sleeping = !WiFi_Connected();
  Serial.println();
  return sleeping;
}

bool sleepNow()
{
  // ==================== turn off wifi to save battery  =====================
  Serial.print(F("Entering wifi sleep... "));
//  WiFi.disconnect();
//  delay(1);
//  WiFi.mode( WIFI_OFF );
//  delay(1);
  
  bool b = WiFi.forceSleepBegin();
  delay(1);
  Serial.print("Result: "); Serial.print(b);  
  Serial.println("ok!");
  sleeping = true;
}

bool updateCheck()
{

  bool updateFlag = false;
  String latestVersion = "";
  String LogStr = "FOTA: ";
  if (WiFi_Connected())
  {
    Serial.println(LogStr + "Checking for new firmware. Current version: " + currentVersion);
    HTTPClient upd;
    Serial.println(LogStr + "Connecting to update site: " + updateSite);
    // configure traged server and url
    String file2Load = updateSite + "version.txt";
    Serial.println(LogStr + "File to load: " + file2Load);
    upd.begin(file2Load); //HTTP
    // start connection and send HTTP header
    int httpCode = upd.GET();

    // httpCode will be negative on error
    if (httpCode > 0)
    {
      // HTTP header has been send and Server response header has been handled
      Serial.println(LogStr + "[HTTP] GET result code: " + httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK)
      {
        latestVersion = upd.getString();
        Serial.println(LogStr + "Found version No: <" + latestVersion + ">");
        if (latestVersion != currentVersion)
        {
          Serial.println(LogStr + "Modified version found");
          updateFlag = true;
        }
        else
          Serial.println(LogStr + "No new version found");
      }
    }
    else
    {
      Serial.println(LogStr + "[HTTP] GET... failed, error: " + httpCode);
    }

    upd.end();
  }

  if (updateFlag)
  {
    updateFlag = false;
    Serial.println(LogStr + "Trying to update version <" + currentVersion + "> to version: <" + latestVersion + ">");
    delay(1000);
    t_httpUpdate_return ret = ESPhttpUpdate.update(updateSite + latestVersion + ".bin");

    switch (ret)
    {
    case HTTP_UPDATE_FAILED:
      Serial.print(LogStr);
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(LogStr + "HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println(LogStr + "HTTP_UPDATE_OK");
      updateFlag = true;
      break;
    }
  }
  return updateFlag;
}

String TwoDigitNumber(int n)
{
  if (n >= 0 && n < 10)
    return '0' + String(n);
  else
    return String(n);
}

String Time2Str()
{
  //return = string.format("%04d/%02d/%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  return String(year()) + "-" + TwoDigitNumber(month()) + "-" + TwoDigitNumber(day()) + " " + TwoDigitNumber(hour()) + ":" + TwoDigitNumber(minute()) + ":" + TwoDigitNumber(second());
}

String URLEncode2(const char *msg)
{
  const char *hex = "0123456789abcdef";
  String encodedMsg = "";

  while (*msg != '\0')
  {
    if (('a' <= *msg && *msg <= 'z') || ('A' <= *msg && *msg <= 'Z') || ('0' <= *msg && *msg <= '9'))
    {
      encodedMsg += *msg;
    }
    else
    {
      encodedMsg += '%';
      encodedMsg += hex[*msg >> 4];
      encodedMsg += hex[*msg & 15];
    }
    msg++;
  }
  return encodedMsg;
}

String Value2String(int y)
{
  return !isnan(y) ? String(y) : "";
}

String Value2String(float y)
{
  return !isnan(y) ? String(y) : "";
}

String CreateURL(String postS)
{
  String s = postS;

  s.replace("%ThingspeakKey", String(ThingspeakKey));
  s.replace("%WUndergroundID", String(WUndergroundID));
  s.replace("%WUndergroundPSW", String(WUndergroundPSW));
  s.replace("%PWSweatherID", String(PWSweatherID));
  s.replace("%PWSweatherPSW", String(PWSweatherPSW));
  s.replace("%MySiteID", String(MySiteID));
  s.replace("%MySitePSW", String(MySitePSW));

  s.replace("%temp", Value2String(temp));
  s.replace("%hum", Value2String(hum));
  s.replace("%pres", Value2String(pres));
  s.replace("%dew", Value2String(dewPoint));
  s.replace("%heatIdx", Value2String(heatIndex));
  s.replace("%wSpeed", Value2String(windSpeed));
  s.replace("%wGust", Value2String(windGust));
  s.replace("%wChill", Value2String(windChill));
  s.replace("%wDir", Value2String(windDir));
  s.replace("%cRain", Value2String(rainCurrent));
  s.replace("%hRain", Value2String(rainHour));
  s.replace("%dRain", Value2String(rainDay));
  s.replace("%now", URLEncode2(Time2Str().c_str()));
  return s;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  Serial.println("\n\n");
  Serial.println("Starting " + projectName + " version: " + currentVersion);

  printBoardInfo();

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //wifiManager.setDebugOutput(false);
  //reset saved settings
  //wifiManager.resetSettings();
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());
  Serial.print(F("PASSWORD: "));
  Serial.println(WiFi.psk());
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setRemoveDuplicateAPs(false);

  Serial.println();

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect(projectName.c_str()))
  {
    Serial.println(F("Failed to connect to AP and hit timeout. Restarting now!"));
    //restart and try again,
    ESP.restart();
    delay(1000);
  }

  Serial.println();
  Serial.println(F("Connected to WiFi."));
  Serial.println();

  initializeAnemometer();
  initializeRainGauge();
  initializeWindVane();
  initializeBMESensor();
  pinMode(PIN_CLEAR, INPUT);

  timeClient.begin();
  // update system clock with ntp time
  if (timeClient.update())
    setTime(timeClient.getEpochTime());
  // check for FOTA
  updateCheck();
}

// =============================================================================
//                             main program loop 
// =============================================================================

void loop()
{
  uint32_t currentMillis = millis();

  // if Clear button is pressed, erase ssid and restart
  if (digitalRead(PIN_CLEAR) == LOW)
  {
    Serial.println(F("Clear wifi and restart."));
    WiFi.disconnect();
    ESP.restart();
    delay(1000);
  }

  // we need average gust of 3 seconds
  // and then the max of the result over report period

  // every 3 seconds sample average wind gust 
  if ((int32_t)(currentMillis - nextWindGustTime) >= 0) 
  {
    nextWindGustTime = currentMillis + 3000;
    float tmpGust = getAndResetWindGust();
    if (tmpGust > windGust) windGust = tmpGust;
  }

  // 20 secs before report time, check for wind vane every second;
  uint32_t startCheck = nextReportTime - 20000;
  if ((int32_t)(currentMillis - startCheck) >=0 )
  {
    if ((int32_t)(currentMillis - nextWindVaneTime) >= 0)
    {
      nextWindVaneTime = currentMillis + 1000;
      captureWindVane();

      Serial.println(Time2Str());
    }
  }
  

	  // @@@ update system clock with ntp time
	if (timeClient.update())
	  setTime(timeClient.getEpochTime());

	// check for FOTA updates every UPDATE_CHECK_INTERVAL minutes;
	if ((int32_t)(currentMillis - nextFOTACheckTime) >= 0)
	{
	  nextFOTACheckTime = currentMillis + UPDATE_CHECK_INTERVAL * 1000;
	  updateCheck();
	}
	// @@@
	
  // report every REPORT_DATA_INTERVAL minutes
  if ((int32_t)(currentMillis - nextReportTime) >= 0)
  {
    nextReportTime = currentMillis + REPORT_DATA_INTERVAL * 1000;

    //sleeping = wakeUpNow();
    if (!sleeping) {
      //restartIfNotConnected();

            // moved @@@ here because is waked up and we need internet connection

			// @@@


      // ----------- read BME sensor data ------------------------
      if (BMESensorFound )
      {
        BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
        BME280::PresUnit presUnit(BME280::PresUnit_hPa);
        bme.read(pres, temp, hum, tempUnit, presUnit); // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
        windChill = windChillFactor(temp, windSpeed, metric);
        dewPoint = EnvironmentCalculations::DewPoint(temp, hum, EnvironmentCalculations::TempUnit_Celsius);
        heatIndex = calculateHeatIndex(temp, hum, metric);
      }

      // read wind and rain data
      rainCurrent = getAndResetRainInches() * 25.4f;     // rain in mm
      windSpeed = getAndResetAnemometerMPH();

      windSpeed *= 1.609344f;  // convert to km/h
      windGust *= 1.609344f;
      windPeak *= 1.609344f;
      windDir = getAndResetWindVaneDegrees();

  //    disableInterrupts();

      Serial.println();
      Serial.print(F("Temp: "));
      Serial.print(temp);
      Serial.print(F(" Hum: "));
      Serial.print(hum);
      Serial.print(F(" Pressure: "));
      Serial.print(pres);
      Serial.print(F(" Speed: "));
      Serial.print(windSpeed);
      Serial.print(F(" Gust: "));
      Serial.print(windGust);
      Serial.print(F(" Peak: "));
      Serial.print(windPeak);
      Serial.print(F(" Dir: "));
      Serial.print(windDir);
      Serial.print(F(" Current Rain: "));
      Serial.print(rainCurrent);
      Serial.print(F(" Rain/hour: "));
      Serial.print(rainHour);
      Serial.print(F(" Rain/day: "));
      Serial.print(rainDay);
      Serial.print(F(" WindChill: "));
      Serial.print(windChill);
      Serial.print(F(" DewPoint: "));
      Serial.print(dewPoint);
      Serial.print(F(" HeatIndex: "));
      Serial.println(heatIndex);


      // ================= Create post data for thingspeak =======================
      String postData = CreateURL(postData1);
      Serial.println(F("\nThingspeak post data:"));
      Serial.println(postData);
      //#ifdef SENDTOSITE
      Serial.println(F("Sending..."));
      //send2siteSecure(postSite1, ThingspeakSHA1, postData);
      send2site(postSite1, postData);
      //#endif

      // ================= Post to my custom site ================================
      postData = CreateURL(postData4);
      Serial.println(F("\nMySite post data:"));
      Serial.println(postData);
      Serial.println(F("Sending..."));
      send2site(postSite4, postData);

      // ==========  convert metric to farenheight for wunderground ==============
      temp = convertCtoF(temp);
      pres = 0.0295299830714f * pres;    //hPa to inches
      windSpeed = 0.621371f * windSpeed; //mph
      windGust = 0.621371f * windGust;   //mph
      rainHour /= 25.40f;      // rain in inches

      windChill = windChillFactor(temp, windSpeed, false);
      dewPoint = EnvironmentCalculations::DewPoint(temp, hum, EnvironmentCalculations::TempUnit_Fahrenheit);
      heatIndex = calculateHeatIndex(temp, hum, false);

      // ------------------------- post to wunderground --------------------------
      postData = CreateURL(postData2);
      Serial.println("\nWUnderground post data:");
      Serial.println(postData);
  #ifdef SENDTOSITE
      Serial.println(F("Sending..."));
      send2siteSecure(postSite2, WUndergroundSHA1, postData);
  #endif

      // ------------------------- post to PWSweather.com --------------------------
      postData = CreateURL(postData3);
      Serial.println("\nPWSweather.com post data:");
      Serial.println(postData);
  #ifdef SENDTOSITE
      Serial.println(F("Sending..."));
      send2siteSecure(postSite3, PWSweatherSHA1, postData);
  #endif

      windPeak = 0.0f;
      windGust = 0.0f;
      
      //sleepNow();
      //enableInterrupts();
    }
    
  }

  yield();
}

/* todo: check this out: 
https://github.com/fractalxaos/weather/blob/master/Arduino/WeatherStation.ino
https://github.com/rpurser47/weatherstation/blob/master/weatherstation.ino
https://github.com/sparkfun/Wimp_Weather_Station/blob/master/Wimp_Weather_Station.ino
https://github.com/PaulRB/Wemos-Weather-Station
http://mile-end.co.uk/blog/?p=86
http://www.iwmi.cgiar.org/tools/mobile-weather-stations/Mobile%20Weather%20Stations%20-%20Manual.pdf
https://www.hackster.io/hliang/thingspeak-weather-station-data-analysis-2877b0
*/
