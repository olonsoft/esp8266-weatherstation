#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include "Accounts.h"

#include <SPI.h>                  // this is needed for BME280 library
#include <BME280I2C.h>            // by Tyler Glenn https://github.com/finitespace/BME280
#include <Wire.h>                 // Needed for legacy versions of Arduino.

#define UPDATE_CHECK_INTERVAL 300 // FOTA update interval
#define AVERAGE_GUST_INTERVAL 3   // calc average gust every 3 seconds and uses the max of these measurements. https://www.wmo-sat.info/oscar/variables/view/205
#define REPORT_DATA_INTERVAL 600  // report every X seconds
#define CUP_CIRCUMFERENCE 1       // in meters. Experiment with this value (0.67 * 1.5)
#define READ_DIR_INTERVAL 1       // read wind direction sensor every 1 sec to calc average
#define CLOCK_SYNC_INTERVAL 3600  // update clock every X seconds
#define USE_SERIAL Serial
#define DEBOUNCE_DELAY 10         // 10 ms

String projectName = "anemometer";
String currentVersion = "1.0.17";
#define SENDTOSITE // if commented, will not post to sites.

//thingspeak
String postSite1 = "http://api.thingspeak.com/update";
String postData1 = "key=%ThingspeakKey&field1=%temp&field2=%hum&field3=%pres&field4=%dew&field5=%heatIdx&field6=%wSpeed&field7=%wGust&field8=%wDir";
// weatherundergound
String postSite2 = "http://weatherstation.wunderground.com/weatherstation/updateweatherstation.php";
String postData2 = "ID=%WUndergroundID&PASSWORD=%WUndergroundPSW&dateutc=now&winddir=%wDir&windspeedmph=%wSpeed&windgustmph=%wGust&tempf=%temp&baromin=%pres&dewptf=%dew&humidity=%hum&rainin=%hRain&softwaretype=esp8266&action=updateraw";
//PWSweather.com
String postSite3 = "http://www.pwsweather.com/pwsupdate/pwsupdate.php";
String postData3 = "ID=%PWSweatherID&PASSWORD=%PWSweatherPSW&dateutc=%now&winddir=%wDir&windspeedmph=%wSpeed&windgustmph=%wGust&tempf=%temp&baromin=%pres&dewptf=%dew&humidity=%hum&rainin=%hRain&softwaretype=esp8266&action=updateraw";

// variables for ntp time
static const char ntpServerName[] = "pool.ntp.org";
const int timeZone = 0;     // Central European Time (UTC = 0)
WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
time_t getNtpTime();
void sendNTPpacket(IPAddress &address);

Ticker tickerUpdateCheck;
Ticker tickerGustTimer;
Ticker tickerReadDirection;
Ticker tickerClockSync;
Ticker tickerReportData;

bool doUpdateCheck = true;
bool doReadDirection = true;
bool doClockSync = true;
bool doCalcGustAverage = false;
bool doReportData = false;

BME280I2C bme;                // Default : forced mode, standby time = 1000 ms
                              // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool metric = true;
float temp(NAN), hum(NAN), pres(NAN);
uint8_t pressureUnit(1);    // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi

// wind variables
#define PIN_WINDSPEED 14
#define PIN_RAINPIN 12
#define PIN_CLEAR 13

float windSpeed = 0.0f;
float tmpWindSpeed = 0.0f;
volatile unsigned int windSpeedSamplesCount = 0;

float windGust = 0.0f;
float tmpWindGust = 0.0f;
volatile unsigned int windGustSamplesCount = 0;

float windPeak = 0.0f;
unsigned long windSpeedLastIntTime = 0;
unsigned long windSpeedStartSampleTime = 0;
unsigned int periodsCount = 0;

int windDir = 0;
float dewPoint = 0.0f;
float heatIndex = 0.0f;
float windChill = 0.0f;

volatile int currentRainCount = 0;
float currentRain = 0.0f;
int hourRainCount = 0;
int hourRainArray[60] = {};
float hourRain = 0.0f;
int dayRainCount = 0;
int dayRainArray[24] = {};
float dayRain = 0.0f;

unsigned long rainLastIntTime = 0 ;
unsigned long count1h = 0;
unsigned long count1d = 0;


void enableUpdateCheck() {
  doUpdateCheck = true;
}

void enableReadDirection(){
  doReadDirection = true;
}

void enableClockSync() {
  doClockSync = true;
}

void enableCalcGustAverage() {
  doCalcGustAverage = true;
}

void enableReportData() {
  doReportData = true;
}

float convertCtoF(float c) {
  return c * 1.8f + 32;
}

float convertFtoC(float f) {
  return (f - 32) * 0.55555f;
}

void restartIfNotConnected() {
  if (!WiFi.isConnected()) {
    USE_SERIAL.println("Not Connected to WiFi. Rebooting...");
    //reset and try again,
    ESP.restart();
    delay(1000);
    }
}

void send2site(String site, String data) {
  if (WiFi.isConnected()) {
	  HTTPClient http;
	  http.begin(site);
	  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
	  int httpCode = http.POST(data);
	  // httpCode will be negative on error
	  if (httpCode > 0) {
		// HTTP header has been send and Server response header has been handled
		USE_SERIAL.printf("[HTTP] POST/GET code: %d\n", httpCode);

		// file found at server
		if (httpCode == HTTP_CODE_OK) {
		  String payload = http.getString();
		  USE_SERIAL.println(payload);
		}
	  } else {
		USE_SERIAL.printf("[HTTP] POST/GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
	  }
	  http.writeToStream(&Serial);
	  http.end();
  }
}

void windSpeedPinInterrupt() {
  if (windSpeedLastIntTime == 0){
    windSpeedLastIntTime = windSpeedStartSampleTime = millis();
    return;
  }
  unsigned long _timeSpan = millis() - windSpeedLastIntTime;
  windSpeedLastIntTime = millis();
  //Serial.println(thisTime);
  if (_timeSpan > DEBOUNCE_DELAY) { //debouncing

    windSpeedSamplesCount++;
    windGustSamplesCount++;

    float _speed = (CUP_CIRCUMFERENCE / 1000.0f) / (_timeSpan / 3600000.0f);
    //Serial.print("<>Speed: "); Serial.println(_speed);
    tmpWindSpeed += _speed;
    tmpWindGust += _speed;

    if (windPeak < _speed) windPeak = _speed;
  } else Serial.println("Debounce ");
}

void rainPinInterrupt() {
  unsigned long thisTime =  micros() - rainLastIntTime;
  rainLastIntTime = micros();
  //Serial.println(thisTime);
  if (thisTime > 1000) { //debouncing
    currentRainCount++;
  }
}

float totalRain(float rainValue) {
  static float rainValues[REPORT_DATA_INTERVAL] = {};
  float total = 0;
  for (int i = 0; i < REPORT_DATA_INTERVAL-1; i++) {
    rainValues[i] = rainValues[i+1];
    total += rainValues[i];
  }
  rainValues[REPORT_DATA_INTERVAL-1]=rainValue;
  total += rainValue;
  return total;
}

void enableInterrupts() {
  attachInterrupt(PIN_WINDSPEED, windSpeedPinInterrupt, FALLING);
  attachInterrupt(PIN_RAINPIN, rainPinInterrupt, FALLING);
}

void disableInterrupts() {
  detachInterrupt(PIN_WINDSPEED);
  detachInterrupt(PIN_RAINPIN);
}

bool wakeUp() {
  USE_SERIAL.print("Waking Up: ");
  if (!WiFi.isConnected()) {
    //disableInterrupts();
    WiFi.forceSleepWake();
    int i = 0;
    while (!WiFi.isConnected() & (i++ < 20)) {
      USE_SERIAL.print("w");
      delay(1000);
    }
  }
  //enableInterrupts();
  USE_SERIAL.println();
  return WiFi.isConnected();
}

bool updateCheck() {
  bool updateFlag = false;
  String latestVersion = "";
  if (WiFi.isConnected()) {
	  USE_SERIAL.println("Update: Checking for new firmware. Current version: " + currentVersion);
    HTTPClient upd;
    USE_SERIAL.println("Update: Connecting to update site: " + updateSite );
    // configure traged server and url
    String file2Load = updateSite + projectName + ".txt";
    USE_SERIAL.println("Update: File to load: " + file2Load);
    upd.begin(file2Load); //HTTP
    // start connection and send HTTP header
    int httpCode = upd.GET();

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      USE_SERIAL.printf("Update: [HTTP] GET result code: %d\n", httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK) {
        latestVersion = upd.getString();
        USE_SERIAL.println("Update: Found version No: <" + latestVersion + ">");
        if (latestVersion != currentVersion) {
          USE_SERIAL.println("Update: Modified version found");
          updateFlag = true;
        } else USE_SERIAL.println("Update: No new version found");
      }
    } else {
      USE_SERIAL.printf("Update: [HTTP] GET... failed, error: %s\n", upd.errorToString(httpCode).c_str());
    }

    upd.end();
  }

  if (updateFlag) {
    updateFlag = false;
    USE_SERIAL.println("Update: Trying to update version <" + currentVersion + "> to version: <" + latestVersion + ">");
    delay(1000);
    t_httpUpdate_return ret = ESPhttpUpdate.update(updateSite + projectName + ".bin");

    switch(ret) {
        case HTTP_UPDATE_FAILED:
            USE_SERIAL.printf("Update: HTTP_UPDATE_FAILED Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
            break;

        case HTTP_UPDATE_NO_UPDATES:
            USE_SERIAL.println("Update: HTTP_UPDATE_NO_UPDATES");
            break;

        case HTTP_UPDATE_OK:
            USE_SERIAL.println("Update: HTTP_UPDATE_OK");
            updateFlag = true;
            break;
    }
  }
  return updateFlag;
}

int voltage2degrees(int v)
{
  // following values are working with esp8266
  if (v > 920) return 270;  // W
  if (v > 850) return 315;  // NW
  if (v > 780) return 0;    // N
  if (v > 630) return 225;  // SW
  if (v > 500) return 45;   // NE
  if (v > 300) return 180;  // S
  if (v > 200)  return 135; // SE
  if (v > 90)  return 90;   // E
  return 0;
}

int avgWindDir(int windDirection)
{
  const int ARYSIZE = 10;     // number of elements in arrays
  const double DEG2RAD = M_PI / 180.0; // convert degrees to radian
  static double dirNorthSouth[ARYSIZE];
  static double dirEastWest[ARYSIZE];
  static int c = 0; // array counter

  if ( c == ARYSIZE )
  { c = 0; }

  windDirection = windDirection + 1; // convert range from 0-359 to 1 to 360

  dirNorthSouth[c] = cos(windDirection * DEG2RAD);
  dirEastWest[c++] = sin(windDirection * DEG2RAD);

  // Get array totals
  double sumNorthSouth = 0.0;
  double sumEastWest =   0.0;
  int i;
  for (i = 0; i < ARYSIZE; i++)
  {
    sumNorthSouth += dirNorthSouth[i];
    sumEastWest   += dirEastWest[i];
  }
  sumEastWest = (sumEastWest/(double) ARYSIZE);
  sumNorthSouth = (sumNorthSouth/(double) ARYSIZE);
  // use atan2() with average vector
  double avgWindDir;
  avgWindDir = atan2(sumEastWest, sumNorthSouth);
  avgWindDir = avgWindDir / DEG2RAD;  // convert radians back to degrees

  if ( avgWindDir < 0 )
  { avgWindDir += 360; }

  int avgWindDirection = (int)avgWindDir % 360; // atan2() result can be > 360, so use modulus to just return remainder
  if (avgWindDirection > 0) {                   // just in case it's 0
    avgWindDirection = avgWindDirection - 1;    // convert range back to 0-359
  }
  return avgWindDirection;
}

float windChillFactor(float _temperature, float _windspeed, bool isCelcius){
  float _windChill =0.0f;
  if (isCelcius) {
    if ( (_temperature <= 10.0f) && (_windspeed > 4.8f) ) {    //celcius & km/h
      _windChill = 13.12f + 0.6215f * _temperature -
                   11.37f * pow(_windspeed, 0.16f) +
                  0.3965f * _temperature * pow(_windspeed, 0.16f);
    }
  }
  else if ((_temperature <= 50.0f) && (_windspeed > 3.0f)) {     // fahrenheit & miles/h
    _windChill = 35.74f + 0.6215f * _temperature -
                 35.75f * pow(_windspeed, 0.16f) +
                0.4275f * _temperature * pow(_windspeed, 0.16f);
  }
  return _windChill;
}

float calculateHeatIndex(float temperature, float percentHumidity, bool isCelcius) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi;

  if (isCelcius)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isCelcius ? convertFtoC(hi) : hi;
}

//------------- date and time functions

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  if (WiFi.isConnected()) {
    IPAddress ntpServerIP; // NTP server's ip address

    while (Udp.parsePacket() > 0) ; // discard any previously received packets
    USE_SERIAL.println("Transmit NTP Request");
    // get a random server from the pool
    WiFi.hostByName(ntpServerName, ntpServerIP);
    USE_SERIAL.print(ntpServerName);
    USE_SERIAL.print(": ");
    USE_SERIAL.println(ntpServerIP);
    sendNTPpacket(ntpServerIP);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500) {
      int size = Udp.parsePacket();
      if (size >= NTP_PACKET_SIZE) {
        USE_SERIAL.println("NTP Response Received");
        Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
        unsigned long secsSince1900;
        // convert four bytes starting at location 40 to a long integer
        secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
        secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
        secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
        secsSince1900 |= (unsigned long)packetBuffer[43];
        return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
      }
    }
    USE_SERIAL.println("No NTP Response :-(");
  }
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) {
  if (WiFi.isConnected()) {
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket(address, 123); //NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
  }
}

String Time2Str() {
   //return = string.format("%04d/%02d/%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
   return String(year()) + "-" + String(month()) + "-" + String(day())+" "+String(hour())+":"+String(minute())+":" + String(second());
}

String URLEncode2(const char* msg) {
    const char *hex = "0123456789abcdef";
    String encodedMsg = "";

    while (*msg!='\0'){
        if( ('a' <= *msg && *msg <= 'z')
                || ('A' <= *msg && *msg <= 'Z')
                || ('0' <= *msg && *msg <= '9') ) {
            encodedMsg += *msg;
        } else {
            encodedMsg += '%';
            encodedMsg += hex[*msg >> 4];
            encodedMsg += hex[*msg & 15];
        }
        msg++;
    }
    return encodedMsg;
}

String CreateURL(String postS) {
  String s = postS;

  s.replace("%ThingspeakKey", String(ThingspeakKey));
  s.replace("%WUndergroundID", String(WUndergroundID));
  s.replace("%WUndergroundPSW", String(WUndergroundPSW));
  s.replace("%PWSweatherID", String(PWSweatherID));
  s.replace("%PWSweatherPSW", String(PWSweatherPSW));

  s.replace("%temp", String(temp));
  s.replace("%hum", String(hum));
  s.replace("%pres", String(pres));
  s.replace("%dew", String(dewPoint));
  s.replace("%heatIdx", String(heatIndex));
  s.replace("%wSpeed", String(windSpeed));
  s.replace("%wGust", String(windGust));
  s.replace("%wDir", String(windDir));
  s.replace("%cRain", String(currentRain));
  s.replace("%hRain", String(hourRain));
  s.replace("%dRain", String(dayRain));
  s.replace("%now", URLEncode2(Time2Str().c_str()));
  return s;
}

void updateDateTime() {
  USE_SERIAL.print(F("Sync clock..."));
  Udp.begin(localPort);
  setTime(getNtpTime());
  USE_SERIAL.println(Time2Str());
}

void setup() {
  // put your setup code here, to run once:
  USE_SERIAL.begin(115200);
  USE_SERIAL.setDebugOutput(false);
  delay(1000);
  USE_SERIAL.println("\n\n");
  USE_SERIAL.println("Starting " + projectName + " version: " + currentVersion);

  USE_SERIAL.println();

  //USE_SERIAL.print("FlashChipSize: ");      USE_SERIAL.println(ESP.getFlashChipSize());
  USE_SERIAL.print(F("FlashChipRealSize: "));  USE_SERIAL.println(ESP.getFlashChipRealSize());
  //USE_SERIAL.print("FlashChipSizeByChipId: ");  USE_SERIAL.println(ESP.getFlashChipSizeByChipId());
  USE_SERIAL.print(F("Free Heap: "));  USE_SERIAL.println(ESP.getFreeHeap());
  USE_SERIAL.print(F("Free Sketch Size: "));  USE_SERIAL.println(ESP.getFreeSketchSpace());
  USE_SERIAL.print(F("Sketch Size: "));  USE_SERIAL.println(ESP.getSketchSize());

  USE_SERIAL.println();
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //wifiManager.setDebugOutput(false);
  //reset saved settings
  //wifiManager.resetSettings();
  USE_SERIAL.print(F("SSID: "));
  USE_SERIAL.println(WiFi.SSID());
  USE_SERIAL.print(F("PASSWORD: "));
  USE_SERIAL.println(WiFi.psk());
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);

  USE_SERIAL.println();

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect(projectName.c_str())) {
    USE_SERIAL.println(F("Failed to connect to AP and hit timeout"));
    //reset and try again,
    ESP.restart();
    delay(1000);
  }

  USE_SERIAL.println();
  USE_SERIAL.println(F("Connected to WiFi."));

  USE_SERIAL.println();

  // check for updates
  updateCheck();

  USE_SERIAL.println();

  //udate the time
  updateDateTime();

  USE_SERIAL.println();

  // ---------------- BME part -------------------
  USE_SERIAL.print(F("Checking for BME280 Sensor... "));
  Wire.begin(); // for BME280. Parameters are pinSDA, pinSCL Defaults are 4,5
  // try to initialize BME280 sensor
  if (bme.begin()) {
    USE_SERIAL.println(F("Found!"));
  }
  else {
    USE_SERIAL.println(F("Not found!"));
  }
  delay(1000);
  // ----------------------------------
  pinMode(PIN_WINDSPEED, INPUT);
  pinMode(PIN_RAINPIN, INPUT);
  pinMode(PIN_CLEAR, INPUT);

  enableInterrupts();

  // set updateFlag to update every
  tickerUpdateCheck.attach(UPDATE_CHECK_INTERVAL, enableUpdateCheck);
  //
  tickerReadDirection.attach(READ_DIR_INTERVAL, enableReadDirection);
  //
  tickerClockSync.attach(CLOCK_SYNC_INTERVAL, enableClockSync);

  tickerGustTimer.attach(AVERAGE_GUST_INTERVAL, enableCalcGustAverage);
  tickerReportData.attach(REPORT_DATA_INTERVAL, enableReportData);

  windSpeedStartSampleTime = millis();
  count1h = millis();
  count1d = millis();
}

// -----------------------------------------------------------------------------
// ----------------------------- main program loop -----------------------------
// -----------------------------------------------------------------------------
void loop() {

  // if Clear button is pressed, erase ssid and restart
  if (digitalRead(PIN_CLEAR) == LOW) {
    USE_SERIAL.println(F("Clear wifi and restart."));
    WiFi.disconnect();
    ESP.restart();
    delay(1000);
  }

  // --------- read direction every 1 sec to calculate average -------------
  if (doReadDirection) {
    windDir = avgWindDir(voltage2degrees(analogRead(A0)));
    doReadDirection = false;
    //Serial.println(ESP.getFreeHeap(), DEC);
    //USE_SERIAL.printf("%d-%02d-%02d %02d %02d %02d\n",
    //                  year(), month(), day(), hour(), minute(), second() );
  }

  float _speed = 0;

  if (doCalcGustAverage) {
    doCalcGustAverage = false;
    if (windGustSamplesCount > 0)
      _speed = tmpWindGust / windGustSamplesCount;
    else
      _speed = 0;
    if (windGust < _speed) windGust = _speed;
    USE_SERIAL.print("\tWindGust: "); Serial.println(_speed);
    tmpWindGust = 0;
    windGustSamplesCount = 0;
  }

  if (doReportData) {
    doReportData = false;
    USE_SERIAL.print("\tWindSamples: "); USE_SERIAL.print(windSpeedSamplesCount);
    if (windSpeedSamplesCount > 0)
      _speed = tmpWindSpeed / windSpeedSamplesCount;
    else
      _speed = 0;
    USE_SERIAL.print("\tWindSpeed: "); Serial.println(_speed);
    windSpeed = _speed;

    windSpeedSamplesCount = 0;
    tmpWindSpeed = 0;

    disableInterrupts();

    windSpeed /= 2; // we have 2 pulses / revolution   // --------<<<<<<<<<<<< don't forget this
    windGust /= 2;
    windPeak /= 2;

    currentRain = (currentRainCount * 0.2794f); // value in mm; There are 2 pulses per change <<<< test this
    hourRain = totalRain(currentRain);
    dayRain += currentRain;  // not used for the moment

    // ----------- read BME sensor data ------------------------
    bme.read(pres, temp, hum, metric, pressureUnit);        // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)

    windChill = windChillFactor(temp, windSpeed, metric);
    dewPoint = bme.dew(temp, hum, metric);
    heatIndex = calculateHeatIndex(temp, hum, metric);

    USE_SERIAL.println();
    USE_SERIAL.print(F("Temp: "));       USE_SERIAL.print(temp);
    USE_SERIAL.print(F(" Hum: "));       USE_SERIAL.print(hum);
    USE_SERIAL.print(F(" Pressure: "));  USE_SERIAL.print(pres);
    USE_SERIAL.print(F(" Speed: "));     USE_SERIAL.print(windSpeed);
    USE_SERIAL.print(F(" Gust: "));      USE_SERIAL.print(windGust);
    USE_SERIAL.print(F(" Peak: "));      USE_SERIAL.print(windPeak);
    USE_SERIAL.print(F(" Dir: "));       USE_SERIAL.print(windDir);
    USE_SERIAL.print(F(" Current Rain: ")); USE_SERIAL.print(currentRain);
    USE_SERIAL.print(F(" Rain/hour: ")); USE_SERIAL.print(hourRain);
    USE_SERIAL.print(F(" Rain/day: ")); USE_SERIAL.print(dayRain);
    USE_SERIAL.print(F(" WindChill: ")); USE_SERIAL.print(windChill);
    USE_SERIAL.print(F(" DewPoint: "));  USE_SERIAL.print(dewPoint);
    USE_SERIAL.print(F(" HeatIndex: ")); USE_SERIAL.println(heatIndex);


    wakeUp();
    restartIfNotConnected();

    // ----------- update the time  ------------------
    if (doClockSync) {
      //udate the time
      updateDateTime();
      doClockSync = false;
    }

    // ================= Create post data for thingspeak =======================
    String postData = CreateURL(postData1);
    USE_SERIAL.println(F("\nThingspeak post data:"));
    USE_SERIAL.println(postData);
    //#ifdef SENDTOSITE
      USE_SERIAL.println(F("Sending..."));
      send2site(postSite1, postData);
    //#endif

    // ==========  convert metric to farenheight for wunderground ==============
    temp = convertCtoF(temp);
    pres = 0.0295299830714f * pres; //hPa to inches
    windSpeed = 0.621371f * windSpeed; //mph
    windGust = 0.621371f * windGust; //mph
    hourRain = hourRain / 25.40f;  // rain in inches

    windChill = windChillFactor(temp, windSpeed, false);
    dewPoint = bme.dew(temp, hum, false);
    heatIndex = calculateHeatIndex(temp, hum, false);

    // ------------------------- post to wunderground --------------------------
    postData = CreateURL(postData2);
    USE_SERIAL.println("\nWUnderground post data:");
    USE_SERIAL.println(postData);
    #ifdef SENDTOSITE
      USE_SERIAL.println(F("Sending..."));
      send2site(postSite2, postData);
    #endif

    // ------------------------- post to PWSweather.com --------------------------
    postData = CreateURL(postData3);
    USE_SERIAL.println("\nPWSweather.com post data:");
    USE_SERIAL.println(postData);
    #ifdef SENDTOSITE
      USE_SERIAL.println(F("Sending..."));
      send2site(postSite3, postData);
    #endif

    // -------------------- check for firmware updates FOTA --------------------
    if (doUpdateCheck) {
      updateCheck();
      doUpdateCheck = false;
    }

    // ==================== turn off wifi to save battery  =====================
    USE_SERIAL.print(F("Entering wifi sleep... "));
    WiFi.forceSleepBegin();
    delay(1000);
    USE_SERIAL.println("ok!");

    windSpeed = 0.0f;
    windSpeedSamplesCount = 0;

    windGust = 0.0f;
    windGustSamplesCount = 0;

    windPeak = 0.0f;
    windSpeedLastIntTime = 0;
    currentRainCount = 0;
    enableInterrupts();
  }

  // do this every hour:
  /*
  if (((millis() - count1h) >= 3600000) && (hourRainCount > 0)) {
    hourRain = 0.0f;
    hourRainCount = 0;
    count1h = millis();
  }
  */

  // do this every day:
  if (((millis() - count1d) >= (3600000*24)) && (dayRainCount > 0)) {
    dayRain = 0.0f;
    dayRainCount = 0;
    count1d = millis();
  }
}
