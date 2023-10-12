#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BMP3XX.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include <MicroNMEA.h>

// define bmp pin numbers for SCK, CS, MISO, & MOSI; set sea level pressure in HPA
#define I2C_SDA       26  // Use GP2 as I2C1 SDA
#define I2C_SCL       27  // Use GP3 as I2C1 SCL
arduino::MbedI2C Wire1(I2C_SDA, I2C_SCL);
extern TwoWire Wire1;
#define SEALEVELPRESSURE_HPA (1013.25)
/* Set the delay between fresh samples of bno */
#define BNO055_SAMPLERATE_DELAY_MS (100)

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// create strands for bmp, bno, & gps
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);
SFE_UBLOX_GNSS gps;

// default values
long lastTime = 0;
boolean pixelOn = false;
float PRESSURE = 0;
char TEAM_ID[] = "GRISSOM";
char POWER_TIME[12];
char MISSION_TIME[12];
int MissionTimeStart = 0;
int currentMissionTime = 0;
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
const int LEDS = 14;
const int CW_SOLENOID = 11;
const int CCW_SOLENOID = 12;
long ASCENT_CHECK = 0;

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
// Defined colors for each state
// uint32_t stateColors[] = {
//     pixels.Color(255, 0, 0),    // Red for STARTUP
//     pixels.Color(255, 127, 0),  // Orange for CALIBRATION
//     pixels.Color(0, 255, 0),    // Green for IDLE
//     pixels.Color(0, 0, 255),    // Blue for ASCENT
//     pixels.Color(255, 255, 0),  // Yellow for RCS_GO
//     pixels.Color(255, 0, 255),  // Magenta for RCS_LOCK
//     pixels.Color(0, 255, 255),  // Cyan for ANOMALY_DETECTED
//     pixels.Color(255, 127, 0),  // Orange for RECOVERY
//     pixels.Color(128, 0, 128),  // Purple for SHUTDOWN
//     pixels.Color(255, 255, 255) // White for ERROR_STATE
// };

// void ledCheck()
// {
//     pixels.begin();
//     pixels.setPixelColor(0, pixels.Color(0, 0, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 0, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 255, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 255, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 255, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 0, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 0, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 0, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 255, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 255, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 255, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 0, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 0, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 0, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 255, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 255, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 255, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 0, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 0, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 0, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 255, 0));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(255, 255, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 255, 255));
//     pixels.show();
//     delay(125);
//     pixels.setPixelColor(0, pixels.Color(0, 0, 255));
//     pixels.show();
//     delay(125);
//     pixels.clear();
//     pixels.show();
// }

void ledCheck(){
    digitalWrite(LEDS, LOW);
    delay(500);
    digitalWrite(LEDS, HIGH);
    delay(1000);
    digitalWrite(LEDS, LOW);
    delay(500);
    digitalWrite(LEDS, HIGH);
    delay(1000);
    digitalWrite(LEDS, LOW);
    delay(500);
}

void powerTimer(){
    // setting up power timer
    unsigned long currentTimeMillis = millis();
    unsigned int hours = currentTimeMillis / 3600000;
    unsigned int minutes = (currentTimeMillis % 3600000) / 60000;
    unsigned int seconds = (currentTimeMillis % 60000) / 1000;
    unsigned int hundredths = (currentTimeMillis % 1000) / 10;
    // Format the time string and store it in POWER_TIME
    sprintf(POWER_TIME, "%02d:%02d:%02d.%02d", hours, minutes, seconds, hundredths);
}

void missionTimer(){
    // setting up mission timer
    unsigned long currentMissionMillis = millis();
    unsigned int MissionHours = currentMissionMillis / 3600000;
    unsigned int MissionMinutes = (currentMissionMillis % 3600000) / 60000;
    unsigned int MissionSeconds = (currentMissionMillis % 60000) / 1000;
    unsigned int MissionHundredths = (currentMissionMillis % 1000) / 10;
    // Format the time string and store it in POWER_TIME
    sprintf(MISSION_TIME, "%02d:%02d:%02d.%02d", MissionHours, MissionMinutes, MissionSeconds, MissionHundredths);
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
        delay(10);
    //delay(10);

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
    SW_STATE = STARTUP;

    // Check LEDs
    Serial.println("LED CHECK");
    pinMode(LEDS, OUTPUT);
    ledCheck();
    Serial.println("LEDs: OK\n");

    // Check BMP390 sensor
    Serial.println("BMP390 SENSOR CHECK");
    if (!bmp.begin_I2C(0x77, &Wire1))
    {
        Serial.println("Could not find a valid sensor. Check connections.");
        SW_STATE = ANOMALY_DETECTED;
        while (1);
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
        Serial.print("Could not find a valid sensor. Check connections.");
        SW_STATE = ANOMALY_DETECTED;
        while (1);
    }
    else
    {
        delay(2000);
        Serial.println("Sensor: OK\n");
    }

    // Check GPS
    Serial.println("NEO-M9N GPS CHECK");
    if (!gps.begin(Wire1))
    {
        Serial.print("Could not find a valid gps. Check connections.");
        SW_STATE = ANOMALY_DETECTED;
        while (1);
    }
    else
    {
        delay(2000);
        Serial.println("GPS: OK\n");
    }

    // Check Solenoids
    Serial.println("SOLENOID CHECK");
    pinMode(CW_SOLENOID, OUTPUT);
    digitalWrite(CW_SOLENOID, LOW);
    pinMode(CCW_SOLENOID, OUTPUT);
    digitalWrite(CCW_SOLENOID, LOW);
    delay(500);
    digitalWrite(CW_SOLENOID, HIGH);
    delay(500);
    digitalWrite(CW_SOLENOID, LOW);
    delay(1000);
    digitalWrite(CCW_SOLENOID, HIGH);
    delay(500);
    digitalWrite(CCW_SOLENOID, LOW);
    delay(2000);
    Serial.println("SOLENOID: OK\n");

    // Set up BMP390 sensor
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_200_HZ);

    // Set up BNO055 sensor
    bno.setExtCrystalUse(true);

    delay(1000);
    Serial.println("VECTOR ENTERING CALIBRATION!!!");
    Serial.println("-------------------------------");
    Serial.println();
    delay(1000);
    SW_STATE = CALIBRATION;
    Serial.println("TEAM_ID, POWER_TIME, MISSION_TIME, PACKET_COUNT, SW_STATE, CAM_STATE, ALTITUDE, TEMP, ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z, ROLL, PITCH, YAW, SIV, GPS_LAT, GPS_LONG, GPS_ALT");
}

void loop(void)
{
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
    Serial.println((String)TEAM_ID + ", " + POWER_TIME + ", " + MISSION_TIME + ", " + PACKET_COUNT + ", " + stateNames[SW_STATE] + ", " + CAM_STATE + ", " + ALTITUDE + ", " + TEMP + ", " + ACC_X + ", " + ACC_Y + ", " + ACC_Z + ", " + GYRO_X + ", " + GYRO_Y + ", " + GYRO_Z + ", " + quat.x() + ", " + quat.y() + ", " + quat.z() + ", " + SIV + ", " + GPS_LAT + ", " + GPS_LONG + ", " + GPS_ALT);

    switch (SW_STATE)
    {
    case CALIBRATION:

        // power timer
        powerTimer();

        if (millis() - lastTime >= 500)
        {
            lastTime = millis();
            if (pixelOn)
            {
                digitalWrite(LEDS, HIGH);
            }
            else
            {
                digitalWrite(LEDS, LOW);
            }
            pixelOn = !pixelOn;
        }
        delay(250);

        // Transition to the IDLE once calibrated
        if (SIV > 3)
        {
            SW_STATE = IDLE;
        }
        break;

    case IDLE:

        // power timer
        powerTimer();

        // Perform actions for the IDLE state
        // Check for conditions to transition to other states
        if (millis() - lastTime >= 5000)
        {
            lastTime = millis();
            if (pixelOn)
            {
                digitalWrite(LEDS, HIGH);
            }
            else
            {
                digitalWrite(LEDS, LOW);
            }
            pixelOn = !pixelOn;
        }
        delay(5000);
        digitalWrite(CW_SOLENOID, HIGH);
        delay(500);
        digitalWrite(CW_SOLENOID, LOW);
        delay(1000);
        digitalWrite(CCW_SOLENOID, HIGH);
        delay(500);
        digitalWrite(CCW_SOLENOID, LOW);

        // Transition to the ASCENT once calibrated
        if (ACC_Z > 3)
        {
            ASCENT_CHECK++;
            if (ASCENT_CHECK > 4)
            {
                SW_STATE = ASCENT;
                MissionTimeStart = millis();
            }
        }
        break;

    case ASCENT:

        // power & mission timer
        powerTimer();
        missionTimer();

        if (millis() - lastTime >= 1000)
        {
            lastTime = millis();
            if (pixelOn)
            {
                digitalWrite(LEDS, HIGH);
            }
            else
            {
                digitalWrite(LEDS, LOW);
            }
            pixelOn = !pixelOn;
        }
        delay(250);

        // Transition to the RCS_GO once calibrated
        if (ALTITUDE > 18000 && GPS_ALT > 18000 && millis() - MissionTimeStart > 5880000) // 5880000 milli is 98 minutes
        {
            SW_STATE = RCS_GO;
        }
        break;

    case RCS_GO:

        // power & mission timer
        powerTimer();
        missionTimer();

        if (millis() - lastTime >= 1000)
        {
            lastTime = millis();
            if (pixelOn)
            {
                digitalWrite(LEDS, HIGH);
            }
            else
            {
                digitalWrite(LEDS, LOW);
            }
            pixelOn = !pixelOn;
        }
        delay(250);

        // Transition to the RCS_GO once calibrated
        if (ALTITUDE < 18000 && GPS_ALT < 18000)
        {
            SW_STATE = RCS_LOCK;
        }
        break;
    
    case RCS_LOCK:

        // power & mission timer
        powerTimer();
        missionTimer();

        if (millis() - lastTime >= 1000)
        {
            lastTime = millis();
            if (pixelOn)
            {
                digitalWrite(LEDS, HIGH);
            }
            else
            {
                digitalWrite(LEDS, LOW);
            }
            pixelOn = !pixelOn;
        }
        delay(250);

        break;

    case ANOMALY_DETECTED:

        // power & mission timer
        powerTimer();
        missionTimer();

        if (millis() - lastTime >= 1000)
        {
            lastTime = millis();
            if (pixelOn)
            {
                digitalWrite(LEDS, HIGH);
            }
            else
            {
                digitalWrite(LEDS, LOW);
            }
            pixelOn = !pixelOn;
        }
        delay(250);

        Serial.println("VECTOR: ANOMALY_DETECTED!!!");

        break;

    case RECOVERY:

        // power & mission timer
        powerTimer();
        missionTimer();

        if (millis() - lastTime >= 1000)
        {
            lastTime = millis();
            if (pixelOn)
            {
                digitalWrite(LEDS, HIGH);
            }
            else
            {
                digitalWrite(LEDS, LOW);
            }
            pixelOn = !pixelOn;
        }
        delay(250);

        break;

    case SHUTDOWN:

        // power & mission timer
        powerTimer();
        missionTimer();

        if (millis() - lastTime >= 1000)
        {
            lastTime = millis();
            if (pixelOn)
            {
                digitalWrite(LEDS, HIGH);
            }
            else
            {
                digitalWrite(LEDS, LOW);
            }
            pixelOn = !pixelOn;
        }
        delay(250);

        break;
    
    case ERROR_STATE:

        // power & mission timer
        powerTimer();
        missionTimer();

        if (millis() - lastTime >= 1000)
        {
            lastTime = millis();
            if (pixelOn)
            {
                digitalWrite(LEDS, HIGH);
            }
            else
            {
                digitalWrite(LEDS, LOW);
            }
            pixelOn = !pixelOn;
        }
        delay(250);

        break;
    }
}