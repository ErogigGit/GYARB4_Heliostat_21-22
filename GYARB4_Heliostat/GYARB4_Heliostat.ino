#include "config.h"
#include <Adafruit_INA219.h>
#include <Adafruit_PWMServoDriver.h>
#include "Adafruit_LC709203F.h"

Adafruit_LC709203F lc;
Adafruit_INA219 ina219;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41); // make sure too solder together the A0 pads on the bottom of the featherwing


#define SERVOMIN  590 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2800 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 300 //Servo frequency for digital servos
#define uS_TO_S_FACTOR 1000000ULL
#define NormalSleepTime 600     /* Time ESP32 will go to sleep (in seconds) */
#define NightSleepTime 1800

int pulseleng;
uint8_t servoUpDown = 4;
uint8_t servoSide = 7;
int azipulse;
int elepulse;
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASS;
float Lon = 13.38 * DEG_TO_RAD,
      Lat = 55.34 * DEG_TO_RAD,
      elevation,
      azimuth;
int sun_azimuth;
int sun_elevation;
//String time_str, current_hour, current_minute, current_day, current_month, current_year;

//static RTC_DATA_ATTR struct timeval sleep_enter_time;
RTC_DATA_ATTR long time2sleep;
RTC_DATA_ATTR int WifiFailures;
//int solarChargerClicks;
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
  /*
  pinMode(wakeUpIntPin, INPUT_PULLUP); 
  esp_sleep_enable_ext0_wakeup(wakeUpIntPin,0);
  struct timeval now;
  
  gettimeofday(&now, NULL);
  long sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  
  switch(wakeup_reason) {
    case soughtWakeupReason: 
      solarChargerClicks += 1;
      time2sleep = time2sleep - sleep_time_ms;
      gettimeofday(&sleep_enter_time, NULL);
      esp_deep_sleep_start();
      break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    default: Serial.printf("Wakeup reason unknown: %d\n",wakeup_reason); break;
  }
  */
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
  /*
  if(current_hour.toInt()>20){//// Dont upload values if its too late
    time2sleep = 60 * 30 * 1000000; //reset sleep time
    esp_sleep_enable_timer_wakeup(time2sleep);//30 minutes between send readings wake ups
    gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
    Serial.println("Going to sleep...");
    esp_deep_sleep_start();
  }
  if(current_hour.toInt()<6){//// Dont upload values if its too early
    time2sleep = 60 * 30 * 1000000; //reset sleep time
    esp_sleep_enable_timer_wakeup(time2sleep);//30 minutes between send readings wake ups
    gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
    Serial.println("Going to sleep...");
    esp_deep_sleep_start();
  }
  else{
    time2sleep = wakeup_time_usec; //15 minutes sleep time
  }
  */
  // INA219 code start
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  Serial.println("ina connected");
  
  for(int i = 0; i < 1000; i++) {
    current_sum = current_sum + ina219.getCurrent_mA();   //Calculate the total sum
    Serial.println(i);
    if(i==999) { // here due to weird error
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
      //group->set("solarCharger", solarChargerClicks);
      group->save();
      Serial.println("sent data");
      
      esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
      //gettimeofday(&sleep_enter_time, NULL);
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
  // Servo code begin
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
/*
  StartTime();
  UpdateLocalTime();
  Serial.print("Update time:");
  Serial.println(Update_DateTime());
*/
  Calculate_Sun_Position(timeHour, timeMin, 0, timeDay, timeMonth, timeYear); // parameters are HH:MM:SS DD:MM:YY start from midnight and work out all 24 hour positions.

  azipulse=map(sun_azimuth, 90, 270, 180, 0);//Map the sun position to degrees
  pulseleng=map(azipulse, 0, 180, SERVOMIN, SERVOMAX);// Map the degrees to PWM
  pwm.setPWM(servoSide, 0, pulseleng);//Move the servo to correct position
  delay(1000);
  pwm.setPWM(servoSide, 0, 4096);//servo "sleep"
  Serial.println("side moved");
  if (sun_elevation < 0) {
    sun_elevation = 0; // Point at horizon if less than horizon
  }
  sun_elevation = 145 - sun_elevation;
  sun_elevation = map(sun_elevation, 0, 180, SERVOMIN, SERVOMAX);// Map the degrees to PWM
  pwm.setPWM(servoUpDown, 0, sun_elevation);  // Move the servo to correct position
  delay(1000);
  pwm.setPWM(servoUpDown, 0, 4096);//servo "sleep"
  Serial.println("updown moved");
  // Servo code end
  
  // Send data and go too sleep
  group->set("Current_static", INA219_current_avg);
  group->set("Voltage_static", fGaugeVoltage);
  group->set("Percentage_static", fGaugePercentage);
  //group->set("solarCharger", solarChargerClicks);
  group->save();
  Serial.println("sent data");
  
  esp_sleep_enable_timer_wakeup(time2sleep*uS_TO_S_FACTOR);
  //gettimeofday(&sleep_enter_time, NULL);
  Serial.println("Going to sleep...");
  esp_deep_sleep_start();
}

void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
  float T, JD_frac, L0, M, e, C, L_true, f, R, GrHrAngle, Obl, RA, Decl, HrAngle;
  long JD, JDx;
  int   zone = 0;  //Unused variable but retained for continuity 
  JD      = JulianDate(year, month, day);
  JD_frac = (hour + minute / 60. + second / 3600.0) / 24.0 - 0.5;
  T          = JD - 2451545; T = (T + JD_frac) / 36525.0;
  L0         = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * T, 360);
  M          = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * T, 360);
  e          = 0.016708617 - 0.000042037 * T;
  C          = DEG_TO_RAD * ((1.9146 - 0.004847 * T) * sin(M) + (0.019993 - 0.000101 * T) * sin(2 * M) + 0.00029 * sin(3 * M));
  f          = M + C;
  Obl        = DEG_TO_RAD * (23 + 26 / 60.0 + 21.448 / 3600. - 46.815 / 3600 * T);
  JDx        = JD - 2451545;
  GrHrAngle  = 280.46061837 + (360 * JDx) % 360 + 0.98564736629 * JDx + 360.98564736629 * JD_frac;
  GrHrAngle  = fmod(GrHrAngle, 360.0);
  L_true     = fmod(C + L0, 2 * PI);
  R          = 1.000001018 * (1 - e * e) / (1 + e * cos(f));
  RA         = atan2(sin(L_true) * cos(Obl), cos(L_true));
  Decl       = asin(sin(Obl) * sin(L_true));
  HrAngle    = DEG_TO_RAD * GrHrAngle + Lon - RA;
  elevation  = asin(sin(Lat) * sin(Decl) + cos(Lat) * (cos(Decl) * cos(HrAngle)));
  azimuth    = PI + atan2(sin(HrAngle), cos(HrAngle) * sin(Lat) - tan(Decl) * cos(Lat)); // Azimuth measured east from north, so 0 degrees is North
  sun_azimuth   = azimuth   / DEG_TO_RAD;
  sun_elevation = elevation / DEG_TO_RAD;
  Serial.println("Longitude and latitude " + String(Lon / DEG_TO_RAD, 3) + " " + String(Lat / DEG_TO_RAD, 3));
  Serial.println("Year\tMonth\tDay\tHour\tMinute\tSecond\tElevation\tAzimuth");
  Serial.print(String(year) + "\t" + String(month) + "\t" + String(day) + "\t" + String(hour - zone) + "\t" + String(minute) + "\t" + String(second) + "\t");
  Serial.println(String(elevation / DEG_TO_RAD, 0) + "\t\t" + String(azimuth / DEG_TO_RAD, 0));
}
long JulianDate(int year, int month, int day) {
  long JD;
  int A, B;
  if (month <= 2) {year--; month += 12;}
  A = year / 100; B = 2 - A + A / 4;
  JD = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
  return JD;
}
/*
void StartTime() {
  configTime(0, 0, "0.uk.pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1); // Change for your location
  UpdateLocalTime();
}

void UpdateLocalTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }
  //See http://www.cplusplus.com/reference/ctime/strftime/
  Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S"); // Displays: Saturday, June 24 2017 14:05:49
  char output[50];
  strftime(output, 50, "%a %d-%b-%y  (%H:%M:%S)", &timeinfo);
  time_str = output;
}

String Update_DateTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time - trying again");
  }
  //See http://www.cplusplus.com/reference/ctime/strftime/
  char output[50];
  strftime(output, 50, "%H", &timeinfo);
  current_hour   = output;
  strftime(output, 50, "%M", &timeinfo);
  current_minute = output;
  strftime(output, 50, "%d", &timeinfo);
  current_day    = output;
  strftime(output, 50, "%m", &timeinfo);
  current_month  = output;
  strftime(output, 50, "%Y", &timeinfo);
  current_year   = output;
  Serial.println(time_str);
  return time_str; // returns date-time formatted like this "11/12/17 22:01:00"
}
*/
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
