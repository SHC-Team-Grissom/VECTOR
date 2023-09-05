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

// set software states
enum FlightState
{
  STARTUP,
  IDLE,
  ASCENT,
  RCS_GO,
  RCS_LOCK,
  ANOMALY_DETECTED,
  RECOVERY,
  SHUTDOWN,
  ERROR_STATE,
  NUM_STATES
};
FlightState SW_STATE = STARTUP;
// Define colors for each state
uint32_t stateColors[] = {
    pixels.Color(255, 0, 0),    // Red for STARTUP
    pixels.Color(0, 255, 0),    // Green for IDLE
    pixels.Color(0, 0, 255),    // Blue for ASCENT
    pixels.Color(255, 255, 0),  // Yellow for RCS_GO
    pixels.Color(255, 0, 255),  // Magenta for RCS_LOCK
    pixels.Color(0, 255, 255),  // Cyan for ANOMALY_DETECTED
    pixels.Color(255, 127, 0),  // Orange for RECOVERY
    pixels.Color(128, 0, 128),  // Purple for SHUTDOWN
    pixels.Color(255, 255, 255) // White for ERROR_STATE
};
// Define state names
const char *stateNames[NUM_STATES] = {
    "STARTUP",
    "IDLE",
    "ASCENT",
    "RCS_GO",
    "RCS_LOCK",
    "ANOMALY_DETECTED",
    "RECOVERY",
    "SHUTDOWN",
    "ERROR_STATE"};

// set default vars
float altitude = 0;
float pressure = 0;

char TEAM_ID[] = "GRISSOM";
char MISSION_TIME[12];
long PACKET_COUNT = 0;
char CAM_STATE[] = "IDLE";
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
  pixels.setPixelColor(0, stateColors[SW_STATE]);
  pixels.show();
  if (!bmp.begin_I2C())
  {
    Serial.println("Could not find a valid sensor. Check connections.");
    pixels.setPixelColor(0, pixels.Color(255, 255, 255));
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

bool startupComplete = false;
bool startupMessagePrinted = false;
bool idleMessagePrinted = false;
bool liftoffDetected = false;
bool ascentMessagePrinted = false;
bool rcsGo = false;
bool rcsgoMessagePrinted = false;
bool rcsLock = false;
bool rcslockMessagePrinted = false;
bool anomalyDetected = false;
bool anomalyMessagePrinted = false;
bool recoverable = false;
bool touchdownComplete = false;
bool recoveryComplete = false;
bool recoveryMessagePrinted = false;
bool shutdownMessagePrinted = false;


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
  // increment packet count, get altitude [meters] & temp [celsius]
  PACKET_COUNT++;
  long ALTITUDE = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  long TEMP = bmp.temperature;

  // print telem to serial
  Serial.println((String)TEAM_ID + ", " + MISSION_TIME + ", " + PACKET_COUNT + ", " + stateNames[SW_STATE] + ", " + CAM_STATE + ", " + ALTITUDE + ", " + TEMP + ", " + ACC_X + ", " + ACC_Y + ", " + ACC_Z + ", " + GYRO_X + ", " + GYRO_Y + ", " + GYRO_Z + ", " + GPS_LAT + ", " + GPS_LONG + ", " + GPS_ALT);

  switch (SW_STATE)
  {
  case STARTUP:
    // Perform actions for the STARTUP state
    if (!startupMessagePrinted)
    {
      Serial.println("VECTOR ENTERING STARTUP!!!");
      Serial.println("-------------------------------");
      Serial.println();
      startupMessagePrinted = true;
    }
    // double check for the BMP390
    if (!bmp.performReading())
    {
      Serial.println("Failed BMP Reading!!!");
      pixels.setPixelColor(0, pixels.Color(255, 255, 255));
      pixels.show();
      return;
    }
    pixels.setPixelColor(0, stateColors[SW_STATE]);
    pixels.show();
    delay(5000);
    startupComplete = true;

    // Transition to the next state when ready
    if (startupComplete)
    {
      SW_STATE = IDLE;
    }
    break;

  case IDLE:
    // Perform actions for the IDLE state
    if (!idleMessagePrinted)
    {
      Serial.println("VECTOR ENTERING IDLE!!!");
      Serial.println("-------------------------------");
      Serial.println();
      idleMessagePrinted = true;
    }
    // Check for conditions to transition to other states
    pixels.setPixelColor(0, stateColors[SW_STATE]);
    pixels.show();
    delay(5000);
    liftoffDetected = true;

    if (liftoffDetected)
    {
      SW_STATE = ASCENT;
    }
    break;

  case ASCENT:
    // Perform actions for the ASCENT state
    if (!ascentMessagePrinted)
    {
      Serial.println("VECTOR ENTERING ASCENT!!!");
      Serial.println("-------------------------------");
      Serial.println();
      ascentMessagePrinted = true;
    }
    // Check for conditions to transition to other states (e.g., anomaly detected)
    pixels.setPixelColor(0, stateColors[SW_STATE]);
    pixels.show();
    delay(5000);
    rcsGo = true;
    anomalyDetected = false;

    if (rcsGo)
    {
      SW_STATE = RCS_GO;
    }
    if (anomalyDetected)
    {
      SW_STATE = ANOMALY_DETECTED;
    }
    break;

  case RCS_GO:
    // Perform actions for the RCS state
    if (!rcsgoMessagePrinted)
    {
      Serial.println("VECTOR ENTERING RCS GO!!!");
      Serial.println("-------------------------------");
      Serial.println();
      rcsgoMessagePrinted = true;
    }
    // Check for conditions to transition to other states
    pixels.setPixelColor(0, stateColors[SW_STATE]);
    pixels.show();
    delay(5000);
    rcsLock = true;
    anomalyDetected = false;

    if (rcsLock)
    {
      SW_STATE = RCS_LOCK;
    }
    if (anomalyDetected)
    {
      SW_STATE = ANOMALY_DETECTED;
    }
    break;

  case ANOMALY_DETECTED:
    // Perform actions for the ANOMALY_DETECTED state
    if (!anomalyMessagePrinted)
    {
      Serial.println("VECTOR ENTERING ANOMALY DETECTED!!!");
      Serial.println("-------------------------------");
      Serial.println();
      anomalyMessagePrinted = true;
    }
    // Transition to the recovery state or another appropriate state
    pixels.setPixelColor(0, stateColors[SW_STATE]);
    pixels.show();
    delay(5000);
    recoverable = true;

    if (recoverable)
    {
      SW_STATE = RECOVERY;
    }
    else
    {
      SW_STATE = RCS_LOCK; // to rcs lock to just ride the flight out
    }
    break;

  case RCS_LOCK:
    // Perform actions for the RCS_LOCK state
    if (!rcslockMessagePrinted)
    {
      Serial.println("VECTOR ENTERING RCS LOCKOUT!!!");
      Serial.println("-------------------------------");
      Serial.println();
      rcslockMessagePrinted = true;
    }
    // Transition to shutdown after touchdown
    pixels.setPixelColor(0, stateColors[SW_STATE]);
    pixels.show();
    delay(5000);
    touchdownComplete = true;

    if (touchdownComplete)
    {
      SW_STATE = SHUTDOWN;
    }
    break;

  case RECOVERY:
    // Perform actions for the RECOVERY state
    if (!recoveryMessagePrinted)
    {
      Serial.println("VECTOR ENTERING RECOVERY!!!");
      Serial.println("-------------------------------");
      Serial.println();
      recoveryMessagePrinted = true;
    }
    // Transition back to normal flight when recovery is complete
    pixels.setPixelColor(0, stateColors[SW_STATE]);
    pixels.show();
    delay(5000);
    recoveryComplete = true;

    if (recoveryComplete)
    {
      SW_STATE = SHUTDOWN;
    }
    break;

  case SHUTDOWN:
    // Perform actions for the SHUTDOWN state
    if (!shutdownMessagePrinted)
    {
      Serial.println("VECTOR ENTERING SHUTDOWN!!!");
      Serial.println("-------------------------------");
      Serial.println();
      shutdownMessagePrinted = true;
    }
    // Transition back to normal flight when recovery is complete
    pixels.setPixelColor(0, stateColors[SW_STATE]);
    pixels.show();
    delay(5000);
    break;
  }
}