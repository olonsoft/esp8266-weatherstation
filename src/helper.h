void printBoardInfo()
{
  Serial.println();
  //Serial.print("FlashChipSize: ");      Serial.println(ESP.getFlashChipSize());
  Serial.print(F("FlashChipRealSize: "));
  Serial.println(ESP.getFlashChipRealSize());
  //Serial.print("FlashChipSizeByChipId: ");  Serial.println(ESP.getFlashChipSizeByChipId());
  Serial.print(F("Free Heap: "));
  Serial.println(ESP.getFreeHeap());
  Serial.print(F("Free Sketch Size: "));
  Serial.println(ESP.getFreeSketchSpace());
  Serial.print(F("Sketch Size: "));
  Serial.println(ESP.getSketchSize());
  Serial.print(F("FlashChip Mode: "));
  Serial.println(ESP.getFlashChipMode());
  Serial.print(F("SDK version: "));
  Serial.println(ESP.getSdkVersion());
  Serial.print(F("Core version: "));
  Serial.println(ESP.getCoreVersion());
  Serial.println();
}

float convertCtoF(float c)
{
  return c * 1.8f + 32;
}

float convertFtoC(float f)
{
  return (f - 32) * 0.55555f;
}


float windChillFactor(float _temperature, float _windspeed, bool isCelcius)
{
  float _windChill = NAN;
  if (!isnan(_temperature))
  {
    if (isCelcius)
    {
      if ((_temperature <= 10.0f) && (_windspeed > 4.8f))
      { //celcius & km/h
        _windChill = 13.12f + 0.6215f * _temperature -
                     11.37f * pow(_windspeed, 0.16f) +
                     0.3965f * _temperature * pow(_windspeed, 0.16f);
      }
    }
    else if ((_temperature <= 50.0f) && (_windspeed > 3.0f))
    { // fahrenheit & miles/h
      _windChill = 35.74f + 0.6215f * _temperature -
                   35.75f * pow(_windspeed, 0.16f) +
                   0.4275f * _temperature * pow(_windspeed, 0.16f);
    }
  }
  return _windChill;
}

float calculateHeatIndex(float temperature, float percentHumidity, bool isCelcius)
{
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  if (isnan(temperature) || isnan(percentHumidity))
    return NAN;
  float hi;

  if (isCelcius)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79)
  {
    hi = -42.379 +
         2.04901523 * temperature +
         10.14333127 * percentHumidity +
         -0.22475541 * temperature * percentHumidity +
         -0.00683783 * pow(temperature, 2) +
         -0.05481717 * pow(percentHumidity, 2) +
         0.00122874 * pow(temperature, 2) * percentHumidity +
         0.00085282 * temperature * pow(percentHumidity, 2) +
         -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if ((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if ((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isCelcius ? convertFtoC(hi) : hi;
}
