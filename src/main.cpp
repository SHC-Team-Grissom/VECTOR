#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BMP3XX.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
#include <MicroNMEA.h>
#include <Adafruit_NeoPixel.h>

// define bmp pin numbers for SCK, CS, MISO, & MOSI; set sea level pressure in HPA
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
/* Set the delay between fresh samples of bno */
#define BNO055_SAMPLERATE_DELAY_MS (100)

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// create strands for bmp, bno, gps, & pixels
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
SFE_UBLOX_GNSS gps;
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

// default values
long lastTime = 0;
boolean pixelOn = false;
float PRESSURE = 0;
char TEAM_ID[] = "GRISSOM";
char MISSION_TIME[12];
long PACKET_COUNT = 0;
char CAM_STATE[] = "IDLE";
float ALTITUDE = 0;
float TEMP = 0;
long ACC_X = 0;
long ACC_Y = 0;
long ACC_Z = 0;
long GYRO_X = 0;
long GYRO_Y = 0;
long GYRO_Z = 0;
long ROLL = 0;
long PITCH = 0;
long YAW = 0;
byte SIV = 0;
long GPS_LAT = 0;
long GPS_LONG = 0;
long GPS_ALT = 0;

// set software states
enum FlightState
{
    STARTUP,
    CALIBRATION,
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
// Define state names
const char *stateNames[NUM_STATES] = {
    "STARTUP",
    "CALIBRATION",
    "IDLE",
    "ASCENT",
    "RCS_GO",
    "RCS_LOCK",
    "ANOMALY_DETECTED",
    "RECOVERY",
    "SHUTDOWN",
    "ERROR_STATE"};
// Define colors for each state
uint32_t stateColors[] = {
    pixels.Color(255, 0, 0),    // Red for STARTUP
    pixels.Color(255, 127, 0),  // Orange for CALIBRATION
    pixels.Color(0, 255, 0),    // Green for IDLE
    pixels.Color(0, 0, 255),    // Blue for ASCENT
    pixels.Color(255, 255, 0),  // Yellow for RCS_GO
    pixels.Color(255, 0, 255),  // Magenta for RCS_LOCK
    pixels.Color(0, 255, 255),  // Cyan for ANOMALY_DETECTED
    pixels.Color(255, 127, 0),  // Orange for RECOVERY
    pixels.Color(128, 0, 128),  // Purple for SHUTDOWN
    pixels.Color(255, 255, 255) // White for ERROR_STATE
};
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

void setup(void)
{
    Serial.begin(115200);

    while (!Serial)
        delay(10); // wait for serial port to open!

    // Print startup message
    Serial.println("VECTOR FLIGHT COMPUTER STARTUP");
    Serial.println("-------------------------------");
    Serial.println();

    // Display VECTOR ASCII art
    Serial.println(vectorArt);

    // Print startup message
    Serial.println("VECTOR ENTERING STARTUP!!!");
    Serial.println("-------------------------------");
    Serial.println();

    // Check LEDs
    Serial.println("LED CHECK");
    ledCheck();
    Serial.println("LEDs: OK\n");

    pixels.setPixelColor(0, stateColors[SW_STATE]);
    pixels.show();

    // Check BMP390 sensor
    Serial.println("BMP390 SENSOR CHECK");
    if (!bmp.begin_I2C())
    {
        Serial.println("Could not find a valid sensor. Check connections.");
        SW_STATE = ANOMALY_DETECTED;
        pixels.setPixelColor(0, stateColors[SW_STATE]);
        pixels.show();
        while (1)
            ;
    }
    else
    {
        delay(2000);
        Serial.println("Sensor: OK\n");
    }

    // Check BNO055 sensor
    Serial.println("BNO055 SENSOR CHECK");
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Could not find a valid sensor. Check connections.");
        SW_STATE = ANOMALY_DETECTED;
        pixels.setPixelColor(0, stateColors[SW_STATE]);
        pixels.show();
        while (1)
            ;
    }
    else
    {
        delay(2000);
        Serial.println("Sensor: OK\n");
    }

    // Check GPS
    Serial.println("NEO-M9N GPS CHECK");
    if (!gps.begin())
    {
        /* There was a problem detecting the GPS ... check your connections */
        Serial.print("Could not find a valid gps. Check connections.");
        SW_STATE = ANOMALY_DETECTED;
        pixels.setPixelColor(0, stateColors[SW_STATE]);
        pixels.show();
        while (1)
            ;
    }
    else
    {
        delay(2000);
        Serial.println("GPS: OK\n");
    }

    // Set up BMP390 sensor
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_200_HZ);

    // Set up BNO055 sensor
    bno.setExtCrystalUse(true);

    delay(1000);
    pixels.clear();
    pixels.show();
    Serial.println("VECTOR ENTERING CALIBRATION!!!");
    Serial.println("-------------------------------");
    Serial.println();
    delay(1000);
    SW_STATE = CALIBRATION;
    Serial.println("TEAM_ID, MISSION_TIME, PACKET_COUNT, SW_STATE, CAM_STATE, ALTITUDE, TEMP, ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z, ROLL, PITCH, YAW, SIV, GPS_LAT, GPS_LONG, GPS_ALT");
}

void loop(void)
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

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Quaternion quat = bno.getQuat();

    ACC_X = linearaccel.x();
    ACC_Y = linearaccel.y();
    ACC_Z = linearaccel.z();
    GYRO_X = gyro.x();
    GYRO_Y = gyro.y();
    GYRO_Z = gyro.z();
    ROLL = quat.x();
    PITCH = quat.y();
    YAW = quat.z();
    SIV = gps.getSIV();
    GPS_LAT = gps.getLatitude();
    GPS_LONG = gps.getLongitude();
    GPS_ALT = gps.getAltitudeMSL();

    // print telem to serial
    Serial.println((String)TEAM_ID + ", " + MISSION_TIME + ", " + PACKET_COUNT + ", " + stateNames[SW_STATE] + ", " + CAM_STATE + ", " + ALTITUDE + ", " + TEMP + ", " + ACC_X + ", " + ACC_Y + ", " + ACC_Z + ", " + GYRO_X + ", " + GYRO_Y + ", " + GYRO_Z + ", " + quat.x() + ", " + quat.y() + ", " + quat.z() + ", " + SIV + ", " + GPS_LAT + ", " + GPS_LONG + ", " + GPS_ALT);

    switch (SW_STATE)
    {
    case CALIBRATION:

        if (millis() - lastTime >= 1000)
        {
            lastTime = millis();
            if (pixelOn)
            {
                pixels.setPixelColor(0, pixels.Color(0, 0, 0));
            }
            else
            {
                pixels.setPixelColor(0, stateColors[SW_STATE]);
            }
            pixelOn = !pixelOn;
            pixels.show();
        }
        delay(250);

        // Transition to the next state when ready
        if (SIV > 3)
        {
            SW_STATE = IDLE;
        }
        break;

    case IDLE:
        // Perform actions for the IDLE state
        // Check for conditions to transition to other states
        pixels.setPixelColor(0, stateColors[SW_STATE]);
        pixels.show();
        delay(5000);

        break;
    }
}