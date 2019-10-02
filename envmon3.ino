/****************************************************************

   Environmental monitor

   Track changes in temperature, barometric pressure and humidity
   and store the reading every two minutes to an SD card.
   Live data including date and time are displayed on a 20 x 4
   LCD module.
 ****************************************************************
   LCD interface based on:
   https://www.instructables.com/id/Using-PCF8574-backpacks-with-LCD-modules-and-Ardui/

   See also:
   https://docs.labs.mediatek.com/resource/linkit7697-arduino/en/tutorial/driving-1602-lcd-with-pcf8574-pcf8574a

   LCD Library: 
   https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads

   Usage:
   https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home   
****************************************************************
   Hardware Setup

   Backpack     Arduino
   ========     =======
      SDA          A4
      SCL          A5

****************************************************************
   RTC Support
   https://github.com/adafruit/RTClib
   
   Hardware Setup

    DS3231      Arduino
   ========     =======
      SDA          A4
      SCL          A5

****************************************************************
    DHT22 Support
    https://github.com/adafruit/DHT-sensor-library
    
    Hardware Setup
    
    DHT         Arduino
    ===         =======
     1            Vcc
     2             2
     3            N/C
     4            Gnd
    
****************************************************************/

#include <Wire.h>                // I2C Communications
#include <LiquidCrystal_I2C.h>   // LCD Interface (I2C Bus)
#include "RTClib.h"              // DS3231 (I2C Bus)
#include <Adafruit_Sensor.h>     // Sensor abstraction - may not need
#include <Adafruit_BMP280.h>     // BMP280 Interface (I2C Bus)
#include <DHT.h>                 // DHT22 (AM2302) Temperature and Humidity Sensor
#include <DHT_U.h>

/* For the DHT sensor                                         */
#define DHTPIN 2                 // The pin we are using
#define DHTTYPE DHT22            // DHT22 (AM2302)

/* Set the I2C address of the LCD controller
      0x3F for PCF8574A
      0x27 for PCF8574                                        */
#define  LCDADDR 0x27            // LCD backpack i2c address in hex

// Set the LCD I2C address
LiquidCrystal_I2C lcd(LCDADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
LCD *myLCD = &lcd;               // How we talk to the LCD

// define the custom bitmaps
// up to 8 bitmaps are supported
const uint8_t my_bitmap[][8] =
{
  // degree symbol
  {0x00, 0x0C, 0x12, 0x0C, 0x00, 0x00, 0x00, 0x00},
  // up arrow
  {0x04, 0x0E, 0x1F, 0x04, 0x04, 0x04, 0x00, 0x00},
  // down arrow
  {0x00, 0x00, 0x04, 0x04, 0x04, 0x1F, 0x0E, 0x04},
  // rectangle
  {0x00, 0x1F, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x00},
  // up-left arrow
  {0x1F, 0x1E, 0x1C, 0x1A, 0x11, 0x00, 0x00, 0x00},
  // up-right arrow
  {0x1F, 0x0F, 0x07, 0x0B, 0x11, 0x00, 0x00, 0x00},
  // down-left arrow
  {0x00, 0x00, 0x00, 0x11, 0x1A, 0x1C, 0x1E, 0x1F},
  // down-right arrow
  {0x00, 0x00, 0x00, 0x11, 0x0B, 0x07, 0x0F, 0x1F},
};

RTC_DS3231 rtc;                  // How we talk to the RTC

/* Useful names for days of the week and months               */
char DoW[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
char Mon[13][4] = {"", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                   "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
                  };

Adafruit_BMP280 bme;             // How we talk to the BMP280

DHT dht(DHTPIN, DHTTYPE);        // How we talk to the DHT22

/* Some macros for delays in the program                      */
#define LONG_DELAY()  delay(3000)
#define SHORT_DELAY() delay(500)
#define ANIM_DELAY()  delay(400)

/* Globals that the various functions use                     */
float T, P, H;                   // Temperature, Pressure, Humidity
unsigned long delayTime = 0;     // Actual value set later
unsigned long lastm = millis();  // Time since startup - do early

/* Setup is called at reset before the main Loop is started   */
void setup()
{
  Serial.begin(9600);            // Setup serial port for diagnostics
  
  // Activate LCD module
  myLCD->begin(20, 4);           // 4 rows, 20 columns
  myLCD->backlight();            // Turn on backlight

  // register the custom bitmaps
  int i;
  int bitmap_size = sizeof(my_bitmap) / sizeof(my_bitmap[0]);
  
  for (i = 0; i < bitmap_size; i++)
  {
    myLCD->createChar(i, (uint8_t *)my_bitmap[i]);
  }

  // Activate the RTC
  if (! rtc.begin()) {
    myLCD->clear();
    myLCD->setCursor(0, 0);
    myLCD->print("   RTC Not Found    ");
    while (1);
  } else {
    Serial.println("RTC found ...");
  }
  // remove comment and adjust to set time
  //                    yyyy  m  d  HH mm ss
  //rtc.adjust(DateTime(2019, 3, 2, 20,13,0));

  // Activate the BMP280 sensor
  if (!bme.begin()) {
    myLCD->clear();
    myLCD->setCursor(0, 0);
    myLCD->print(" BMP Sensor Error  ");
    Serial.println("BMP sensor not found");
    while (1);
  } else {
    Serial.println("BMP found ...");
  }
  // Configure BMP sensor parameters
  /*
    bme.setSampling(Adafruit_BME280::MODE_FORCED,     // sleep until required
                      Adafruit_BME280::SAMPLING_X1,   // temperature
                      Adafruit_BME280::SAMPLING_X1,   // pressure
                      Adafruit_BME280::SAMPLING_X1,   // humidity
                      Adafruit_BME280::FILTER_OFF ); */

  // Activate the DHT22 sensor
  dht.begin();
  Serial.println("DHT activated ...");

  delayTime = 6000;             // minimum ms between samples
  lastm = millis();

  showIntro();
  LONG_DELAY();

  takeReadings();                // Perform an initial set of readings
  showBack();                    // Draw static text
  showTemp(T);                   // Display temperature
  showPres(P);                   // Display pressure
  showHum(H);                    // Display humidity
  Star(0);                       // Turn off activity indicator
}

void loop()
{
  // Get the date and time
  DateTime now = rtc.now();

  drawTime(now);                 // Show the time
  drawDate(now);                 // Show the date

  if ((millis() - lastm) >= delayTime) {   // Take readings every minute
    lastm = millis();
    Star(1);                     // Turn on the activity indicator
    takeReadings();              // Read all the sensors
    showTemp(T);                 // Show temperature
    showPres(P);                 // Show pressure
    showHum(H);                  // Show humidity
    SHORT_DELAY();               // A little delay for the activity indicator
	// logSD();                  // Save data to SD card
    Star(0);                     // Turn off activity indicator
  }
}

void showIntro(void) {
  // move the cursor to 0
  myLCD->clear();
  myLCD->setCursor(0, 0);
  myLCD->print("   Environmental    ");
  myLCD->setCursor(0, 1);
  myLCD->print("      Monitor       ");
  myLCD->setCursor(0, 2);
  myLCD->print("     Version 3      ");
  myLCD->setCursor(0, 3);
  myLCD->print(" Blackfire Software ");
}

void showBack(void) {
  // clear the LCD and set the string direction to left-to-right
  myLCD->clear();
  myLCD->backlight();
  myLCD->leftToRight();
  myLCD->setCursor(0, 0);
  myLCD->print("Temp:-XX C         *");   // The * appears when data written to SD
  myLCD->setCursor(8, 0); //in column 8 of row 0
  myLCD->print(char(0));  //display degree symbol

  myLCD->setCursor(0, 1);
  myLCD->print("Pres:XXXX mB        ");
  myLCD->setCursor(13, 1); // in column 13, row 1
  myLCD->print(char(1));   // display up arrow for rising pressure trend
  myLCD->setCursor(14, 1); // in column 14, row 1
  myLCD->print(char(2));   // display down arrow for falling pressure trend

  myLCD->setCursor(0, 2);
  myLCD->print("Hum: XX%            ");
  myLCD->setCursor(9, 2); // in column 10, row 2
  myLCD->print(char(1));   // display up arrow for rising humidity trend
  myLCD->setCursor(10, 2); // in column 11, row 2
  myLCD->print(char(2));   // display down arrow for falling humidity trend

  myLCD->setCursor(0, 3);
  myLCD->print("DDD, MMM DD HH:MM:SS");

}

String two(String s) {
  String r;
  String t;
  String z = "0";
  t = z + s;
  r = t.substring(t.length() -2, t.length());
  return r;
}

void drawTime(DateTime tm) {
  String s;
  myLCD->setCursor(12, 3);       // Position cursor
  s = makeTimestr(tm);           // Get time as String
  myLCD->print(s);               // Print the time
}

String makeTimestr(DateTime tm) {
  String t;
  // Add a leading 0 to the hours (if necessary)
  t = two("0" + String(tm.hour()));
  // Separator
  t = t + String(":");
  // Add a leading 0 to the minutes (if necessary)
  t = t + two("0" + String(tm.minute()));
  // Separator
  t = t + String(":");
  // Add a leading 0 to the seconds (if necessary)
  t = t + two("0" + String(tm.second()));
  return t;
}

void drawDate(DateTime dt) {
  String s;
  myLCD->setCursor(0, 3);
  myLCD->print(DoW[dt.dayOfTheWeek()]);
  myLCD->setCursor(5, 3);
  myLCD->print(Mon[dt.month()]);
  myLCD->setCursor(9, 3);
  s = two(String(dt.day()));
  myLCD->print(s);
  myLCD->print(" ");
}

/* Date is YYYY-MM-DD
String makeDatestr(DateTime dt) {
  String t;
  // Start with the day of the year
  t = String();
  // Separator
  t = t + String("-");
  // The month as two digits with leading zero
  t = t + String(two(dt.month()));
  // Separator
  t = t + String("-");
  // The day of the month
  t = t + String(two(dt.day()));
  return t;
}

/* Take sensor readings - can add other sensors easily        */
void takeReadings() {
  T = readTemp();
  P = readPres();
  H = readHum();
}

float readTemp(void) {
  return (float)bme.readTemperature();
}

float readPres(void) {
  return (float)bme.readPressure() / 100.0;
}

float readHum(void) {
  return (float)dht.readHumidity();
}

void showTemp(float temp) {
  String s;
  s = String(temp, 1);

  myLCD->setCursor(5, 0);
  myLCD->print(s);
  myLCD->print(" ");
  myLCD->print(char(0));
  myLCD->print("C");
  //Serial.print("Temperature: ");
  //Serial.println(String(temp));
}

void showPres(float pres) {
  String s;
  s = String(pres / 10.0, 1);

  myLCD->setCursor(5, 1);
  myLCD->print(s);
  myLCD->print(" kPa");
  //Serial.print("Pressure: ");
  //Serial.println(String(pres));
}

void showHum(float Humidity) {
  String s;
  s = String(Humidity, 1);
  
  myLCD->setCursor(5, 2);
  myLCD->print(s);
  myLCD->print(" % ");
  Serial.print("Humidity: ");
  Serial.println(String(Humidity));
}

void Star(int st) {
  myLCD->setCursor(19, 0);
  if (st == 0)
    myLCD->print(" ");
  else if (st == 1)
    myLCD->print("*");    
}
