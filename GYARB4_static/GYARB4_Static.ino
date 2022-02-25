#include "config.h"
#include <Adafruit_INA219.h>
#include "Adafruit_LC709203F.h"

Adafruit_LC709203F lc;
Adafruit_INA219 ina219;

#define uS_TO_S_FACTOR 1000000ULL
#define NormalSleepTime 600     /* Time ESP32 will go to sleep (in seconds) */
#define NightSleepTime 1800

RTC_DATA_ATTR long time2sleep;
RTC_DATA_ATTR int WifiFailures;
int retries;
int timeHour;
int timeMin;
int timeDay;
int timeMonth;
int timeYear;
float fGaugeVoltage;
float fGaugePercentage;
float current_sum = 0;
float INA219_current_avg = 0;

AdafruitIO_Group *group = io.group("gyarb4-heliostat");
AdafruitIO_Time *timeH = io.time(AIO_TIME_ISO);

void setup() 
{
  Serial.begin(115200);
  timeHour = 123;
  io.connect();
  retries = 0;
  while(io.status() < AIO_CONNECTED) {
    retries++;
    if(retries < 9) {
      Serial.println(io.statusText());
      delay(1000);
    }
    else  {
      Serial.println("to many tries going to sleep");
      switch (WifiFailures) {
      case 0: time2sleep = 60; break;
      case 1: time2sleep = 180; break;
      case 2: time2sleep = 600; break;
      case 3: time2sleep = 1200; break;
      default: time2sleep = 3600; break;
      }
      Serial.println("Going to sleep now");
      WifiFailures++;
      esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
      Serial.println(time2sleep);
      esp_deep_sleep_start();
    }
  }
  Serial.println(io.statusText());
  timeH->onMessage(handleISO);
  
  io.run();
  
  // INA219 code start
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  Serial.println("ina connected");
  
  for(int i = 0; i < 1000; i++) {
    current_sum = current_sum + ina219.getCurrent_mA();   //Calculate the total sum
    Serial.println(i);
    if(i==999) {
      break;
    }
    }
  
  INA219_current_avg = current_sum / 1000;//Calculate the current average
  Serial.println("ina average calculated");
  // INA219 code end

  //Fuel Gauge code start
  if (!lc.begin()) {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
  Serial.println("fg connected");
  lc.setPackSize(LC709203F_APA_2000MAH);
  lc.setAlarmVoltage(3.8);

  fGaugeVoltage = lc.cellVoltage();
  fGaugePercentage = lc.cellPercent();
  //Fuel Gauge code end
  if (fGaugePercentage < 3) {
    time2sleep = 1800;
  } 
  else if (fGaugePercentage < 4) {
    time2sleep = 1200;
  } 
  else if (fGaugePercentage < 6) {
    time2sleep = 900;
  } 
  else {
    time2sleep = NormalSleepTime;
  }

  long startTime = millis();
  while (timeHour == 123) {
    if (millis()-startTime > 10000) {
      group->set("Current_static", INA219_current_avg);
      group->set("Voltage_static", fGaugeVoltage);
      group->set("Percentage_static", fGaugePercentage);
      group->save();
      Serial.println("sent data");
      
      esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
      Serial.println("Going to sleep...");
      esp_deep_sleep_start();
      break;
      }
    else {
      io.run();
      }
  } 
  if (timeHour < 6 || timeHour >= 18) {
    time2sleep = NightSleepTime;
  }
  
  // Send data and go too sleep
  group->set("Current_static", INA219_current_avg);
  group->set("Voltage_static", fGaugeVoltage);
  group->set("Percentage_static", fGaugePercentage);
  group->save();
  Serial.println("sent data");
  
  esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
  Serial.println("Going to sleep...");
  esp_deep_sleep_start();
}

void handleISO(char *data, uint16_t len) {
  struct tm tm = {0};
  // Convert to tm struct
  strptime(data, "%Y-%m-%dT%H:%M:%SZ", &tm);
  timeMin = tm.tm_min;
  timeHour = tm.tm_hour;
  timeHour++;
  if(timeHour>=24) {
    timeHour = timeHour-24;
  }
  timeDay = tm.tm_mday;
  timeMonth = tm.tm_mon;
  timeYear = (tm.tm_year)%100;
}

void loop() {
}
