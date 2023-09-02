#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

// define bmp pin numbers for SCK, CS, MISO, & MOSI; set sea level pressure in HPA
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

// create a bmp strand
Adafruit_BMP3XX bmp;
// create a pixel strand with 1 pixel on PIN_NEOPIXEL
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

// set default vars
float altitude = 0;
float pressure = 0;

char TEAM_ID[] = "Grissom";
char MISSION_TIME[12];
long PACKET_COUNT = 0;
char SW_STATE[] = "Idle";
char CAM_STATE[] = "Idle";
long ACC_X = 0;
long ACC_Y = 0;
long ACC_Z = 0;
long GYRO_X = 0;
long GYRO_Y = 0;
long GYRO_Z = 0;
long GPS_LAT = 0;
long GPS_LONG = 0;
long GPS_ALT = 0;
char vectorArt[] =
    "                   ___           ___                       ___           ___     \n"
    "      ___         /  /\\         /  /\\          ___        /  /\\         /  /\\    \n"
    "     /__/\\       /  /:/_       /  /:/         /  /\\      /  /::\\       /  /::\\   \n"
    "     \\  \\:\\     /  /:/ /\\     /  /:/         /  /:/     /  /:/\\:\\     /  /:/\\:\\  \n"
    "      \\  \\:\\   /  /:/ /:/_   /  /:/  ___    /  /:/     /  /:/  \\:\\   /  /:/~/:/  \n"
    "  ___  \\__\\:\\ /__/:/ /:/ /\\ /__/:/  /  /\\  /  /::\\    /__/:/ \\__\\:\\ /__/:/ /:/___\n"
    " /__/\\ |  |:| \\  \\:\\/:/ /:/ \\  \\:\\ /  /:/ /__/:/\\:\\   \\  \\:\\ /  /:/ \\  \\:\\/:::::/\n"
    " \\  \\:\\|  |:|  \\  \\::/ /:/   \\  \\:\\  /:/  \\__\\/  \\:\\   \\  \\:\\  /:/   \\  \\::/~~~~ \n"
    "  \\  \\:\\__|:|   \\  \\:\\/:/     \\  \\:\\/:/        \\  \\:\\   \\  \\:\\/:/     \\  \\:\\     \n"
    "   \\__\\::::/     \\  \\::/       \\  \\::/          \\__\\/    \\  \\::/       \\  \\:\\    \n"
    "       ~~~~       \\__\\/         \\__\\/                     \\__\\/         \\__\\/    \n";

void ledCheck()
{
  pixels.begin();

  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 255, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 255, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 255, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 255, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 255, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 255, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 255, 0));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 255, 255));
  pixels.show();
  delay(125);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();
  delay(125);

  pixels.clear();
  pixels.show();
}

void setup()
{
  delay(5000);
  Serial.begin(115200);
  while (!Serial)
    ;

  // Print startup message
  Serial.println("VECTOR FLIGHT COMPUTER STARTUP");
  Serial.println("-------------------------------");
  Serial.println();

  // Display VECTOR ASCII art
  Serial.println(vectorArt);

  // Print startup message
  Serial.println("VECTOR SYSTEM CHECK");
  Serial.println("-------------------------------");
  Serial.println();

  // Check LEDs
  Serial.println("LED CHECK");
  ledCheck();
  Serial.println("LEDs: OK\n");

  // Check BMP390 sensor
  Serial.println("BMP390 SENSOR CHECK");
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  if (!bmp.begin_I2C())
  {
    Serial.println("Could not find a valid sensor. Check connections.");
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    while (1)
      ;
  }
  else
  {
    pixels.clear();
    pixels.show();
    delay(2000);
    Serial.println("Sensor: OK\n");
  }

  // Set up BMP390 sensor
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  // Print header for data
  // Print startup message
  Serial.println("VECTOR ENTERING STARTUP!!!");
  Serial.println("-------------------------------");
  Serial.println();
  Serial.println("TEAM_ID, MISSION_TIME, PACKET_COUNT, SW_STATE, CAM_STATE, ALTITUDE, TEMP, ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z, GPS_LAT, GPS_LONG, GPS_ALT");
  Serial.println();
}

void loop()
{
  // setting up mission timer
  unsigned long currentTimeMillis = millis();
  unsigned int hours = currentTimeMillis / 3600000;
  unsigned int minutes = (currentTimeMillis % 3600000) / 60000;
  unsigned int seconds = (currentTimeMillis % 60000) / 1000;
  unsigned int hundredths = (currentTimeMillis % 1000) / 10;
  // Format the time string and store it in MISSION_TIME
  sprintf(MISSION_TIME, "%02d:%02d:%02d.%02d", hours, minutes, seconds, hundredths);

  // double check for the BMP390
  if (!bmp.performReading())
  {
    Serial.println("Failed BMP Reading!!!");
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    return;
  }
  // turn led red
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();

  // increment packet count, get altitude [meters] & temp [celsius]
  PACKET_COUNT++;
  long ALTITUDE = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  long TEMP = bmp.temperature;

  // print telem to serial
  Serial.println((String)TEAM_ID + ", " + MISSION_TIME + ", " + PACKET_COUNT + ", " + SW_STATE + ", " + CAM_STATE + ", " + ALTITUDE + ", " + TEMP + ", " + ACC_X + ", " + ACC_Y + ", " + ACC_Z + ", " + GYRO_X + ", " + GYRO_Y + ", " + GYRO_Z + ", " + GPS_LAT + ", " + GPS_LONG + ", " + GPS_ALT);

  // clear led with delays causing loop to last 500millisec (1/2 a second)
  delay(250);
  pixels.clear();
  pixels.show();
  delay(250);
}