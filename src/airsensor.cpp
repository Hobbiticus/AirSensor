#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
//#define HAS_SW_SERIAL
#include <PMserial.h> // Arduino library for PM sensors with serial interface
#include <DHT.h>
#include <DHT_U.h>
#include <ezTime.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include "RTClib.h"
#include "mhz19.h"
#include "logger.h"
#include "nextion.h"

//https://ww2.arb.ca.gov/resources/inhalable-particulate-matter-and-health
//PM2.5: <= 12 = healthy, >= 35 unhealthy
//PM10: <= 20 = healthy, >= 50-150 unhealthy

//https://www.health.state.mn.us/communities/environment/air/toxins/co2.html
//CO2: <1000 = healthy, < 5000 = high, 10000 = unhealthy, 30000 = asphyxiation in 15 minutes

enum MeasurementType
{
  MEAS_CO2 = 0,
  MEAS_PM10,
  MEAS_PM25,
  MEAS_MAX,
};

struct QualityThresholds
{
  int m_Thresholds[4];
};

QualityThresholds Qualities[MEAS_MAX] =
{
  {1000, 2500, 5000, 10000},
  {20, 35, 50, 65},
  {12, 19, 27, 35}
};

unsigned short QualityColors[5] = { 1160, 48832, 64613, 63746, 63859 };

unsigned short GetMeasurmentColor(MeasurementType type, int amount)
{
  const QualityThresholds& qualities = Qualities[type];
  for (int i = 0; i < 4; i++)
  {
    if (amount < qualities.m_Thresholds[i])
      return QualityColors[i];
  }
  return QualityColors[4];
}

#define WIFI_SSID "cjweiland"
#define WIFI_PASSWD "areallygoodkey"

Timezone myTZ;

//DHT temperature sensor
#define DHT_DATA_PIN 22
DHT_Unified dht(DHT_DATA_PIN, DHT22);

//DS3231 RTC
#define I2C_SDA 21
#define I2C_SCL 4
RTC_DS3231 rtc;

//CO2 sensor
#define CO2_PWM 27
#define MH_Z19_RX 14
#define MH_Z19_TX 21
MHZ19 co2(-1, -1, CO2_PWM);
//MHZ co2(0, 0, CO2_PWM, MHZ19B);

//PMS7003 PMS sensor
constexpr auto PMS_RX = 17;
constexpr auto PMS_TX = 16;
SerialPM pms(PMS7003, PMS_RX, PMS_TX); // PMSx003, RX, TX

//MICS-6814 gas sensor
#define CO_PIN 34
#define NH3_PIN 39
#define NO2_PIN 36

//Nextion display
#define NEXTION_RX 32
#define NEXTION_TX 33
SoftwareSerial DisplaySerial(NEXTION_RX, NEXTION_TX);
//HardwareSerial DisplaySerial(1); //doesn't work for some reason

//these must match the gdw/gdh of the waveform control
#define GRID_WIDTH 50
#define GRID_HEIGHT 26
//these must match the w/h of the waveform control
#define WAVEFORM_WIDTH 256
#define WAVEFORM_HEIGHT 188
const static int YAxisLines = (WAVEFORM_HEIGHT / GRID_HEIGHT) + 1;

#define HISTORY_LENGTH WAVEFORM_WIDTH
int dataArray[HISTORY_LENGTH] = {0};

enum DisplayPage
{
  PAGE_GRAPH_PM = 0,
  PAGE_GRAPH_WEATHER,
  PAGE_GRAPH_CO2,
  PAGE_GRAPH_TEST,
  PAGE_MAIN,
};
int CurrentPage = PAGE_MAIN;
enum TimeScale
{
  TIMESCALE_MINUTES = 0,
  TIMESCALE_10MINUTES,
  TIMESCALE_MAX,
};
int CurrentTimescale = TIMESCALE_MINUTES;


struct DataStream
{
  const static int InvalidValue = -1000;
  DataStream(const char* name)
  {
    char filename[32] = {0};
    snprintf(filename, sizeof(filename), "01%s.log", name);
    m_MinutesOver10 = new Logger(filename, 6, HISTORY_LENGTH);
    snprintf(filename, sizeof(filename), "%s.log", name);
    m_Minutes = new Logger(filename, 60, HISTORY_LENGTH);
    snprintf(filename, sizeof(filename), "10%s.log", name);
    m_10Minutes = new Logger(filename, 600, HISTORY_LENGTH);

    m_LastMaxValue = InvalidValue;
    m_LastValidValue = InvalidValue;
  }

  void TakeReading(int value)
  {
    m_LastValidValue = value;
    if (value > m_LastMaxValue)
      m_LastMaxValue = value;
  }
  void ResetMax()
  {
    m_LastMaxValue = InvalidValue;
  }

  Logger* m_MinutesOver10;
  Logger* m_Minutes;
  Logger* m_10Minutes;

  int m_LastMaxValue;
  int m_LastValidValue;
};


DataStream* StreamPM25;
DataStream* StreamPM10;
DataStream* StreamTemp;
DataStream* StreamHumidity;
DataStream* StreamCO2;

#define NUM_STREAMS 5
DataStream* AllStreams[NUM_STREAMS];

unsigned int LastLogMinuteTime;
unsigned int LastLog10MinuteTime;
unsigned int MillisecondsPerMinute = 1000 * 60;


struct ChannelParams
{
  int m_Start;
  double m_Step;
  double m_Scale;
};

struct GraphParams
{
  ChannelParams m_Channels[2];
};

GraphParams gGraphParams[PAGE_MAIN];

void SetAxisLabels()
{
  for (int i = 0; i < YAxisLines; i++)
  {
    DisplaySerial.printf("y%d.val=%d", i, (int)(gGraphParams[CurrentPage].m_Channels[0].m_Start + i * gGraphParams[CurrentPage].m_Channels[0].m_Step));
    DisplaySerial.write(EOS, 3);
  }  
  switch (CurrentPage)
  {
    case PAGE_GRAPH_PM:
      DisplaySerial.printf("cScope1.txt=\"PM2.5\"");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("cScope2.txt=\"PM10\"");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("vis cScope2,1");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("vis cScope3,0");
      DisplaySerial.write(EOS, 3);
      break;
    case PAGE_GRAPH_WEATHER:
    {
      for (int i = 0; i < YAxisLines; i++)
      {
        DisplaySerial.printf("y%d2.val=%d", i, (int)(gGraphParams[CurrentPage].m_Channels[1].m_Start + i * gGraphParams[CurrentPage].m_Channels[1].m_Step));
        DisplaySerial.write(EOS, 3);
      }
      DisplaySerial.printf("cScope1.txt=\"Temp\"");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("cScope2.txt=\"Humidity\"");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("vis cScope2,1");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("vis cScope3,0");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("y62.pco=0");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("y72.pco=0");
      DisplaySerial.write(EOS, 3);
    }
    break;
    case PAGE_GRAPH_CO2:
      DisplaySerial.printf("cScope1.txt=\"CO2\"");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("vis cScope2,0");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("vis cScope3,0");
      DisplaySerial.write(EOS, 3);
      break;
    case PAGE_GRAPH_TEST:
    {
      for (int i = 0; i < 6; i++)
      {
        DisplaySerial.printf("x%d.val=%d", i, i * GRID_WIDTH);
        DisplaySerial.write(EOS, 3);
      }
    }
    break;    
  }
}

uint8_t ValueToDisplayValue(int value, int start, double step, double valueScale)
{
  double displayValue = value;
  displayValue *= valueScale;
  displayValue -= start;
  displayValue *= GRID_HEIGHT;
  displayValue /= step;
  //clamp to valid values of uint8_t
  if (displayValue < 0)
    displayValue = 0;
  if (displayValue > 255)
    displayValue = 255;
  return (uint8_t)displayValue;
}

void SendValuesToWaveform(int channel, int numValues, uint8_t* values)
{
  //ctrl ID, channel, array length
  // Serial.printf("addt 1,0,%d", valuesUsed);
  // for (int i = 0; i < valuesUsed; i++)
  //   Serial.printf(" %hhu", displayValues[i]);
  // Serial.println("");
  DisplaySerial.printf("addt 1,%d,%d", channel, numValues); // we are sending an array of this size to the waveform
  DisplaySerial.write(EOS, 3);
  //wait for GO signal
  Serial.println("Waiting for GO");
  uint8_t reply[4];
  int waitStartTime = millis();
  while (DisplaySerial.available() < 4)
  {
    delay(1);
    //have had this stall here before
    if (millis() - waitStartTime > 200)
    {
      Serial.println("GO never received");
      DisplaySerial.end();
      delay(100);
      DisplaySerial.begin(19200);
      return;
    }
  }
  DisplaySerial.readBytes(reply, 4);
  int waitEndTime = millis();
  Serial.printf("Waited %d ms for GO signal\n", waitEndTime - waitStartTime);
  if (memcmp(reply, GO, 4) != 0)
  {
    Serial.printf("GO signal not received, got %02hhx %02hhx %02hhx %02hhx", reply[0], reply[1], reply[2], reply[3]);
    return;
  }

  //now send the values
  DisplaySerial.write(values, numValues);
  //wait for OK signal
  Serial.println("Waiting for OK");
  waitStartTime = millis();
  while (DisplaySerial.available() < 4)
    delay(1);
  DisplaySerial.readBytes(reply, 4);
  waitEndTime = millis();
  Serial.printf("Waited %d ms for AOK signal\n", waitEndTime - waitStartTime);
  if (memcmp(reply, AOK, 4) != 0)
  {
    Serial.printf("OK signal not received, got %02hhx %02hhx %02hhx %02hhx", reply[0], reply[1], reply[2], reply[3]);
    return;
  }
}

void LogOne10Minute(time_t now_time, DataStream* stream)
{
    VALUE values[HISTORY_LENGTH];
    int valuesUsed;
    time_t startTime;
    stream->m_Minutes->GetData(values, &valuesUsed, &startTime);
    //log the highest value
    VALUE highestValue = INVALID_VALUE;
    for (int i = 0; i < (valuesUsed < 10 ? valuesUsed : 10); i++)
    {
      VALUE value = values[valuesUsed - i - 1];
      if (value == INVALID_VALUE)
        continue;
      if (value > highestValue)
        highestValue = value;
    }
    stream->m_10Minutes->AddData(now_time, highestValue);
}

void Log10Minutes(time_t now_time)
{
  LogOne10Minute(now_time, StreamPM25);
  LogOne10Minute(now_time, StreamPM10);
  LogOne10Minute(now_time, StreamTemp);
  LogOne10Minute(now_time, StreamHumidity);
  LogOne10Minute(now_time, StreamCO2);
}

void UpdateGraph()
{
  if (CurrentPage == PAGE_MAIN)
    return;

  DisplaySerial.printf("cle 1,255");
  DisplaySerial.write(EOS, 3);

  uint8_t displayValues[HISTORY_LENGTH];
  int valuesUsed = 0;
  time_t startTime;

  //max batch is allegedly 128 but 256 seems to have worked

  // get the values!
  VALUE values[HISTORY_LENGTH];
  int valuesStart = gGraphParams[CurrentPage].m_Channels[0].m_Start;
  double valuesStep = gGraphParams[CurrentPage].m_Channels[0].m_Step;
  double valuesScale = gGraphParams[CurrentPage].m_Channels[0].m_Scale;
  if (CurrentPage == PAGE_GRAPH_TEST)
  {
    valuesUsed = HISTORY_LENGTH;
    for (int i = 0; i < HISTORY_LENGTH; i++)
      values[i] = i;
  }
  else
  {
    DataStream* stream;
    switch (CurrentPage)
    {
      case PAGE_GRAPH_PM:
        stream = StreamPM25; break;
      case PAGE_GRAPH_WEATHER:
        stream = StreamTemp; break;
      case PAGE_GRAPH_CO2:
        stream = StreamCO2; break;
      default:
        return;
    }
    Logger* logger;
    if (CurrentTimescale == TIMESCALE_MINUTES)
      logger = stream->m_Minutes;
    else
      logger = stream->m_10Minutes;
    logger->GetData(values, &valuesUsed, &startTime);
  }

  for (int i = 0; i < valuesUsed; i++)
  {
    int value = values[i];
    //if (value == INVALID_VALUE)
    if (value * valuesScale < valuesStart || value * valuesScale > valuesStart + valuesStep * 10)
    {
      //go back and see if we can't find something appropriate
      //for (int k = i-1; k >= 0 && value == INVALID_VALUE; k--)
      for (int k = i-1; k >= 0 && (value * valuesScale < valuesStart || value * valuesScale > valuesStart + valuesStep * 10); k--)
        value = values[k];
      Serial.printf("Replaced invalid value with %d\n", value);
    }
    if (value != INVALID_VALUE)
      displayValues[i] = ValueToDisplayValue(value, valuesStart, valuesStep, valuesScale);
    else
      displayValues[i] = 0;
  }
  Serial.println("");

  SendValuesToWaveform(0, valuesUsed, displayValues);

  DataStream* stream;
  switch (CurrentPage)
  {
    case PAGE_GRAPH_PM:
      stream = StreamPM10; break;
    case PAGE_GRAPH_WEATHER:
      stream = StreamHumidity; break;
    default:
      return;
  }
  Logger* logger;
  if (CurrentTimescale == TIMESCALE_MINUTES)
    logger = stream->m_Minutes;
  else
    logger = stream->m_10Minutes;
  logger->GetData(values, &valuesUsed, &startTime);

  valuesStart = gGraphParams[CurrentPage].m_Channels[1].m_Start;
  valuesStep = gGraphParams[CurrentPage].m_Channels[1].m_Step;
  valuesScale = gGraphParams[CurrentPage].m_Channels[1].m_Scale;
  for (int i = 0; i < valuesUsed; i++)
  {
    int value = values[i];
    //if (value == INVALID_VALUE)
    if (value * valuesScale < valuesStart || value * valuesScale > valuesStart + valuesStep * 10)
    {
      //go back and see if we can't find something appropriate
      //for (int k = i-1; k >= 0 && value == INVALID_VALUE; k--)
      for (int k = i-1; k >= 0 && (value * valuesScale < valuesStart || value * valuesScale > valuesStart + valuesStep * 10); k--)
        value = values[k];
      Serial.printf("Replaced invalid value with %d\n", value);
    }
    if (value != INVALID_VALUE)
      displayValues[i] = ValueToDisplayValue(value, valuesStart, valuesStep, valuesScale);
    else
      displayValues[i] = 0;
  }

  SendValuesToWaveform(1, valuesUsed, displayValues);
}

bool DidSetTime = false;

void SetRTCTime()
{
  if (DidSetTime)
    return;
  DidSetTime = true;
  //sync to NTP
  waitForSync();
  Serial.println("Synced to NTP");
  myTZ.setLocation("America/New_York");
  Serial.println("eztime says it is " + myTZ.dateTime());
  rtc.adjust(DateTime(myTZ.now()));
  Serial.println("Adjusted RTC");
  char timeBuff[64] = {'\0'};
  strcpy(timeBuff, "YYYY-MM-DD hh:mm:ss");
  DateTime now = rtc.now();
  Serial.println("got RTC now");
  now.toString(timeBuff);
  Serial.printf("Set the time to %s\n", timeBuff);
}

//#define USE_TASKS

#ifdef USE_TASKS

void SensorTask(void* arg)
{
  int ppm_pwm = -1;
  for (;;)
  {
    if (!co2.isPreHeating())
      ppm_pwm = co2.readCO2PWM();

    float temperature = 0;
    float humidity = 0;
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (!isnan(event.temperature))
      temperature = event.temperature * 9 / 5 + 32;
    dht.humidity().getEvent(&event);
    if (!isnan(event.relative_humidity))
      humidity = event.relative_humidity;

    unsigned short co_reading = analogRead(CO_PIN);
    unsigned short nh3_reading = analogRead(NH3_PIN);
    unsigned short no2_reading = analogRead(NO2_PIN);

    pms.read();

    //...
  }
}

#endif

void setup()
{
  Serial.begin(115200);

  gGraphParams[PAGE_GRAPH_PM].m_Channels[0].m_Start = 0;
  gGraphParams[PAGE_GRAPH_PM].m_Channels[0].m_Step = 10;
  gGraphParams[PAGE_GRAPH_PM].m_Channels[0].m_Scale = 1;
  gGraphParams[PAGE_GRAPH_PM].m_Channels[1].m_Start = 0;
  gGraphParams[PAGE_GRAPH_PM].m_Channels[1].m_Step = 10;
  gGraphParams[PAGE_GRAPH_PM].m_Channels[1].m_Scale = 1;
  gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[0].m_Start = 30;
  gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[0].m_Step = 10;
  gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[0].m_Scale = 0.1;
  gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[1].m_Start = 0;
  gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[1].m_Step = 20;
  gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[1].m_Scale = 0.1;
  gGraphParams[PAGE_GRAPH_CO2].m_Channels[0].m_Start = 0;
  gGraphParams[PAGE_GRAPH_CO2].m_Channels[0].m_Step = 500;
  gGraphParams[PAGE_GRAPH_CO2].m_Channels[0].m_Scale = 1;
  gGraphParams[PAGE_GRAPH_TEST].m_Channels[0].m_Start = 0;
  gGraphParams[PAGE_GRAPH_TEST].m_Channels[0].m_Step = 255;
  gGraphParams[PAGE_GRAPH_TEST].m_Channels[0].m_Scale = 1;

  DisplaySerial.begin(19200);
  //DisplaySerial.begin(19200, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
  DisplaySerial.printf("page %d", 0);
  DisplaySerial.write(EOS, 3);

  //start connecting to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);

  Serial.println("Opening SD card...");
  if (!Logger::begin(5)) //default is 5
  {
    Serial.println("Failed to start SD");
    //this doesn't work for some reason - not sure why
    // DisplaySerial.printf("xstr 37, 68, 228, 30, 1, RED, BLACK, 1, 1, 1, \"Failed to open SD card\"");
    // DisplaySerial.write(EOS, 3);
    //this doesn't work either
    // DisplaySerial.printf("err.txt=\"Failed to open SD card\"");
    // DisplaySerial.write(EOS, 3);
    // DisplaySerial.printf("err.sta=1"); //show
    // DisplaySerial.write(EOS, 3);
    // DisplaySerial.printf("err.bco=0");
    // DisplaySerial.write(EOS, 3);
    // DisplaySerial.printf("err.pco=63488"); //RED
    // DisplaySerial.write(EOS, 3);

    while (1)
    {
      delay(1000);
    }
    return;
  }
  Serial.println("SD card opened!");

  //start the RTC
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!rtc.begin(&Wire))
  {
    Serial.println("Failed to find RTC");
    while (1) delay(1000);
  }
  if (rtc.lostPower())
  {
    Serial.println("RTC lost power!");
    while (WiFi.status() != WL_CONNECTED)
      delay(10);
    Serial.println("Connected to WiFi");

    //set the time on the RTC
    SetRTCTime();
  }

  //NOTE: keep the filenames short (not sure the max, but adding "_minutes" was too long)
  AllStreams[0] = StreamPM25 = new DataStream("pm25");
  AllStreams[1] = StreamPM10 = new DataStream("pm10");
  AllStreams[2] = StreamTemp = new DataStream("temp");
  AllStreams[3] = StreamHumidity = new DataStream("humid");
  AllStreams[4] = StreamCO2 = new DataStream("co2");
  
  dht.begin();

  //no need to set pin modes for analog reads? (on MICS-6814 sensor)

  //I2CBME.begin(I2C_SDA, I2C_SCL, 100000);

  //unsigned status;

  // default settings
  // status = bme.begin(0x76, &I2CBME);
  // if (!status) {
  //     Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  //     Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
  //     Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  //     Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //     Serial.print("        ID of 0x60 represents a BME 280.\n");
  //     Serial.print("        ID of 0x61 represents a BME 680.\n");
  //     while (1) delay(10);
  // }

  pms.init();
  LastLogMinuteTime = millis();
  LastLog10MinuteTime = millis();
#ifdef USE_TASKS
  //func, name, stack size, parameter, priority, out_task_handle, core
  xTaskCreatePinnedToCore(SensorTask, "Sensor", 1024, NULL, 1, NULL, 1);
#endif
}

#define SEALEVELPRESSURE_HPA (1013.25)


void loop()
{
  // if (!DidSetTime && WiFi.status() == WL_CONNECTED)
  // {
  //   SetRTCTime();
  // }

  DateTime dtNow = rtc.now();
  unsigned int now = millis();

  Serial.print("\n----- Time from start: ");
  Serial.print(now / 1000);
  Serial.println(" s");

  //read CO2 concentration
  if (!co2.IsPreheated())
    Serial.println("CO2 is preheating...");
  else
  {
    int co2_ppm = co2.GetCO2();
    Serial.printf("CO2 ppm: %d\n", co2_ppm);
    if (co2_ppm >= 0)
      StreamCO2->TakeReading(co2_ppm);
  }

  //read temperature/humidity
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
    Serial.println(F("Error reading temperature!"));
  else
  {
    StreamTemp->TakeReading((int)((event.temperature * 9.0 / 5 + 32) / gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[0].m_Scale));
    Serial.printf("Temperature = %.1f °F\n", StreamTemp->m_LastValidValue * gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[0].m_Scale);
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
    Serial.println(F("Error reading humidity!"));
  else
  {
    StreamHumidity->TakeReading((int)(event.relative_humidity / gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[1].m_Scale));
    Serial.printf("Humidity = %.1f %%\n", StreamHumidity->m_LastValidValue * gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[1].m_Scale);
  }

  //VOC sensor readings
  //0-4095 (never seen anything other than 0)
  unsigned short co_reading = analogRead(CO_PIN);
  unsigned short nh3_reading = analogRead(NH3_PIN);
  unsigned short no2_reading = analogRead(NO2_PIN);
  Serial.printf("VOC says CO: %hu, NH3: %hu, NO2: %hu\n", co_reading, nh3_reading, no2_reading);

  //read PM concentrations
  pms.read();
  if (pms)
  {
    // print the results
    Serial.printf("PM1.0 %hu, PM2.5 %hu, PM10 %hu [ug/m3]\n", pms.pm01, pms.pm25, pms.pm10);
    StreamPM10->TakeReading(pms.pm10);
    StreamPM25->TakeReading(pms.pm25);
  }
  else
  { // something went wrong
    switch (pms.status)
    {
    case pms.OK: // should never come here
      break;     // included to compile without warnings
    case pms.ERROR_TIMEOUT:
      Serial.println(F(PMS_ERROR_TIMEOUT));
      break;
    case pms.ERROR_MSG_UNKNOWN:
      Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
      break;
    case pms.ERROR_MSG_HEADER:
      Serial.println(F(PMS_ERROR_MSG_HEADER));
      break;
    case pms.ERROR_MSG_BODY:
      Serial.println(F(PMS_ERROR_MSG_BODY));
      break;
    case pms.ERROR_MSG_START:
      Serial.println(F(PMS_ERROR_MSG_START));
      break;
    case pms.ERROR_MSG_LENGTH:
      Serial.println(F(PMS_ERROR_MSG_LENGTH));
      break;
    case pms.ERROR_MSG_CKSUM:
      Serial.println(F(PMS_ERROR_MSG_CKSUM));
      break;
    case pms.ERROR_PMS_TYPE:
      Serial.println(F(PMS_ERROR_PMS_TYPE));
      break;
    }
  }


  Serial.println("\n------------------------------");

  int waitStart = millis();
  bool doUpdateGraph = false;
  char readBytes[1024];
  while (millis() - waitStart < 500)
  {
    if (DisplaySerial.available())
    {
      size_t numBytes = DisplaySerial.readBytes(readBytes, DisplaySerial.available());
      readBytes[numBytes] = '\0';
      Serial.print("RECEIVED:");
      for (size_t i = 0; i < numBytes; i++)
      {
        Serial.printf(" %02hhx", readBytes[i]);
      }
      Serial.printf(" or %s\n", readBytes);
      //search for things we care about
      for (size_t i = 0; i < numBytes; i++)
      {
        if (numBytes - i >= 4 && memcmp(readBytes + i, "TEMP", 4) == 0)
        {
          DisplaySerial.printf("page %d", 1);
          DisplaySerial.write(EOS, 3);
          CurrentPage = PAGE_GRAPH_WEATHER;
          CurrentTimescale = TIMESCALE_MINUTES;
          doUpdateGraph = true;
          SetAxisLabels();
        }
        else if (numBytes - i >= 2 && memcmp(readBytes + i, "PM", 2) == 0)
        {
          DisplaySerial.printf("page %d", 1);
          DisplaySerial.write(EOS, 3);
          CurrentPage = PAGE_GRAPH_PM;
          CurrentTimescale = TIMESCALE_MINUTES;
          doUpdateGraph = true;
          SetAxisLabels();
        }
        else if (numBytes - i >= 3 && memcmp(readBytes + i, "CO2", 3) == 0)
        {
          DisplaySerial.printf("page %d", 1);
          DisplaySerial.write(EOS, 3);
          CurrentPage = PAGE_GRAPH_CO2;
          CurrentTimescale = TIMESCALE_MINUTES;
          doUpdateGraph = true;
          SetAxisLabels();
        }
        else if (numBytes - i >= 4 && memcmp(readBytes + i, "TEST", 4) == 0)
        {
          DisplaySerial.printf("page %d", 1);
          DisplaySerial.write(EOS, 3);
          CurrentPage = PAGE_GRAPH_TEST;
          CurrentTimescale = TIMESCALE_MINUTES;
          doUpdateGraph = true;
          SetAxisLabels();
        }
        else if (numBytes - i >= 4 && memcmp(readBytes + i, "BACK", 4) == 0)
        {
          DisplaySerial.printf("page %d", 0);
          DisplaySerial.write(EOS, 3);
          CurrentPage = PAGE_MAIN;
        }
        else if (numBytes - i >= 4 && memcmp(readBytes + i, "TIME", 5) == 0)
        {
          if (CurrentTimescale == TIMESCALE_MINUTES)
          {
            CurrentTimescale = TIMESCALE_10MINUTES;
            DisplaySerial.printf("x1.txt=\"10h\"");
            DisplaySerial.write(EOS, 3);
            DisplaySerial.printf("x2.txt=\"20h\"");
            DisplaySerial.write(EOS, 3);
            DisplaySerial.printf("x3.txt=\"30h\"");
            DisplaySerial.write(EOS, 3);
            DisplaySerial.printf("x4.txt=\"40h\"");
            DisplaySerial.write(EOS, 3);
          }
          else
          {
            CurrentTimescale = TIMESCALE_MINUTES;
            DisplaySerial.printf("x1.txt=\"1h\"");
            DisplaySerial.write(EOS, 3);
            DisplaySerial.printf("x2.txt=\"2h\"");
            DisplaySerial.write(EOS, 3);
            DisplaySerial.printf("x3.txt=\"3h\"");
            DisplaySerial.write(EOS, 3);
            DisplaySerial.printf("x4.txt=\"4h\"");
            DisplaySerial.write(EOS, 3);
          }
          doUpdateGraph = true;
        }
      }
    }
    delay(100);
  }
  if (doUpdateGraph)
    //this means we just switched pages
    delay(60); // load bearing sleep so the display actually gets all of the values

  if (now - LastLogMinuteTime >= MillisecondsPerMinute)
  {
    LastLogMinuteTime += MillisecondsPerMinute;
    Serial.println("UPDATE!");
    time_t now_time = dtNow.unixtime();
    Serial.printf("pm = %p, stream0 = %p\n", StreamPM25, AllStreams[0]);
    for (int i = 0; i < NUM_STREAMS; i++)
    {
      Serial.printf("Update %d\n", i);
      VALUE value = AllStreams[i]->m_LastMaxValue;
      Serial.printf("Got value: %d", value);
      if (value == DataStream::InvalidValue)
        value = AllStreams[i]->m_LastValidValue;
      Serial.println("adding data");
      AllStreams[i]->m_Minutes->AddData(now_time, value);
      Serial.println("resetting max");
      AllStreams[i]->ResetMax();
      Serial.println("all done");
    }
    
    doUpdateGraph = true;
  }
  if (now - LastLog10MinuteTime >= MillisecondsPerMinute * 10)
  {
    LastLog10MinuteTime += MillisecondsPerMinute * 10;
    time_t now_time = dtNow.unixtime();
    Log10Minutes(now_time);
  }

  if (doUpdateGraph)
  {
    Serial.println("UPDATE GRAPH");
    UpdateGraph();
    Serial.println("UPDATE COMPLETE!");
  }

  char timeBuff[64] = {'\0'};
  strcpy(timeBuff, "hh:mm:ss");
  dtNow.toString(timeBuff);
  if (CurrentPage == PAGE_MAIN)
  {
    DisplaySerial.printf("time.txt=\"%s\"", timeBuff);
    DisplaySerial.write(EOS, 3);

    if (StreamCO2->m_LastValidValue >= 0)
    {
      DisplaySerial.printf("cCO2.txt=\"%d\"", StreamCO2->m_LastValidValue);
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("cCO2.pco=%hu", GetMeasurmentColor(MEAS_CO2, StreamCO2->m_LastValidValue));
      DisplaySerial.write(EOS, 3);
    }
    else
    {
      DisplaySerial.printf("cCO2.txt=\"WARM\"");
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("cCO2.pco=%hu", 65535);
      DisplaySerial.write(EOS, 3);
    }

    DisplaySerial.printf("cTemp.val=%d", (int)(StreamTemp->m_LastValidValue * gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[0].m_Scale));
    DisplaySerial.write(EOS, 3);
    DisplaySerial.printf("cHumidity.val=%d", (int)(StreamHumidity->m_LastValidValue * gGraphParams[PAGE_GRAPH_WEATHER].m_Channels[1].m_Scale));
    DisplaySerial.write(EOS, 3);

    DisplaySerial.printf("cCO.val=%hu", co_reading);
    DisplaySerial.write(EOS, 3);
    DisplaySerial.printf("cNH3.val=%hu", nh3_reading);
    DisplaySerial.write(EOS, 3);
    DisplaySerial.printf("cNO2.val=%hu", no2_reading);
    DisplaySerial.write(EOS, 3);

    if (pms)
    {
      DisplaySerial.printf("cPM10.val=%hu", pms.pm10);
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("cPM10.pco=%hu", GetMeasurmentColor(MEAS_PM10, pms.pm10));
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("cPM25.val=%hu", pms.pm25);
      DisplaySerial.write(EOS, 3);
      DisplaySerial.printf("cPM25.pco=%hu", GetMeasurmentColor(MEAS_PM25, pms.pm25));
      DisplaySerial.write(EOS, 3);
    }
  }
}
