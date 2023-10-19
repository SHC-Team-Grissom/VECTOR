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
#define I2C_SDA 26 // Use GP2 as I2C1 SDA
#define I2C_SCL 27 // Use GP3 as I2C1 SCL
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
const int CW_SOLENOID = 6;
const int CCW_SOLENOID = 7;
long ASCENT_CHECK = 0;
int TARGET_MODE = 1;
float target_angle = 1;
float actual_angle = 0;

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

void ledCheck()
{
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

void powerTimer()
{
    // setting up power timer
    unsigned long currentTimeMillis = millis();
    unsigned int hours = currentTimeMillis / 3600000;
    unsigned int minutes = (currentTimeMillis % 3600000) / 60000;
    unsigned int seconds = (currentTimeMillis % 60000) / 1000;
    unsigned int hundredths = (currentTimeMillis % 1000) / 10;
    // Format the time string and store it in POWER_TIME
    sprintf(POWER_TIME, "%02d:%02d:%02d.%02d", hours, minutes, seconds, hundredths);
}

void missionTimer()
{
    // setting up mission timer
    unsigned long currentMissionMillis = millis();
    unsigned int MissionHours = currentMissionMillis / 3600000;
    unsigned int MissionMinutes = (currentMissionMillis % 3600000) / 60000;
    unsigned int MissionSeconds = (currentMissionMillis % 60000) / 1000;
    unsigned int MissionHundredths = (currentMissionMillis % 1000) / 10;
    // Format the time string and store it in POWER_TIME
    sprintf(MISSION_TIME, "%02d:%02d:%02d.%02d", MissionHours, MissionMinutes, MissionSeconds, MissionHundredths);
}

void rcs_correct(float target_angle, float actual_angle)
{
    int deadband = 15;
    //float opp_point = target_angle + 180;

    float lower = target_angle - deadband;
    float upper = target_angle + deadband;

    float target_distance = abs(target_angle - actual_angle);

    if (lower < 0){lower = 360 + lower;}
    if (upper > 360){upper = upper - 360;}

    float left_distance = lower - actual_angle;
    float right_distance = upper - actual_angle;

    float left_distance_alt = lower + (360-actual_angle);
    float right_distance_alt = actual_angle + (360-upper);

    if (left_distance_alt < abs(left_distance)){left_distance = left_distance_alt;}
    if (right_distance_alt < abs(right_distance)){right_distance = right_distance_alt;}

    //if (opp_point > 360){opp_point = opp_point - 360;}

    if (target_distance > deadband)
    {
        if (left_distance < right_distance)
        {
            digitalWrite(CW_SOLENOID, HIGH);
            delay(250);
            digitalWrite(CW_SOLENOID, LOW);
        }

        else if (right_distance < left_distance)
        {
            digitalWrite(CCW_SOLENOID, HIGH);
            delay(250);
            digitalWrite(CCW_SOLENOID, LOW);
        }

        else
        {
            digitalWrite(CCW_SOLENOID, LOW);
            digitalWrite(CW_SOLENOID, LOW);
        }
    }

    else
    {
        digitalWrite(CCW_SOLENOID, LOW);
        digitalWrite(CW_SOLENOID, LOW);
    }
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
    // Serial1.begin(115200);
    Serial.begin(9600);
    Serial1.begin(9600);

    pinMode(LEDS, OUTPUT);
    digitalWrite(LEDS, LOW);
    pinMode(CW_SOLENOID, OUTPUT);
    digitalWrite(CW_SOLENOID, LOW);
    pinMode(CCW_SOLENOID, OUTPUT);
    digitalWrite(CCW_SOLENOID, LOW);

    while (!Serial)
        delay(1000);
    // delay(1000);

    // Print startup message
    Serial.println("VECTOR FLIGHT COMPUTER STARTUP");
    Serial.println("-------------------------------");
    Serial.println();
    Serial1.println("VECTOR FLIGHT COMPUTER STARTUP");
    Serial1.println("-------------------------------");
    Serial1.println();

    // Display VECTOR ASCII art
    Serial.println(vectorArt);
    Serial1.println(vectorArt);

    // Print startup message
    Serial.println("VECTOR ENTERING STARTUP!!!");
    Serial.println("-------------------------------");
    Serial.println();
    Serial1.println("VECTOR ENTERING STARTUP!!!");
    Serial1.println("-------------------------------");
    Serial1.println();
    SW_STATE = STARTUP;

    // Check LEDs
    Serial.println("LED CHECK");
    Serial1.println("LED CHECK");
    ledCheck();
    Serial.println("LEDs: OK\n");
    Serial1.println("LEDs: OK\n");

    // Check BMP390 sensor
    Serial.println("BMP390 SENSOR CHECK");
    Serial1.println("BMP390 SENSOR CHECK");
    if (!bmp.begin_I2C(0x77, &Wire1))
    {
        Serial.println("Could not find a valid sensor. Check connections.");
        Serial1.println("Could not find a valid sensor. Check connections.");
        SW_STATE = ANOMALY_DETECTED;
        while (1)
            ;
    }
    else
    {
        delay(2000);
        Serial.println("Sensor: OK\n");
        Serial1.println("Sensor: OK\n");
    }

    // Check BNO055 sensor
    Serial.println("BNO055 SENSOR CHECK");
    Serial1.println("BNO055 SENSOR CHECK");
    if (!bno.begin())
    {
        Serial.print("Could not find a valid sensor. Check connections.");
        Serial1.print("Could not find a valid sensor. Check connections.");
        SW_STATE = ANOMALY_DETECTED;
        while (1)
            ;
    }
    else
    {
        delay(2000);
        Serial.println("Sensor: OK\n");
        Serial1.println("Sensor: OK\n");
    }

    // Check GPS
    Serial.println("NEO-M9N GPS CHECK");
    Serial1.println("NEO-M9N GPS CHECK");
    if (!gps.begin(Wire1))
    {
        Serial.print("Could not find a valid gps. Check connections.");
        Serial1.print("Could not find a valid gps. Check connections.");
        SW_STATE = ANOMALY_DETECTED;
        while (1)
            ;
    }
    else
    {
        delay(2000);
        Serial.println("GPS: OK\n");
        Serial1.println("GPS: OK\n");
    }

    // Check Solenoids
    Serial.println("SOLENOID CHECK");
    Serial1.println("SOLENOID CHECK");
    pinMode(CW_SOLENOID, OUTPUT);
    digitalWrite(CW_SOLENOID, LOW);
    pinMode(CCW_SOLENOID, OUTPUT);
    digitalWrite(CCW_SOLENOID, LOW);
    delay(1000);
    digitalWrite(CW_SOLENOID, HIGH);
    delay(500);
    digitalWrite(CW_SOLENOID, LOW);
    delay(1000);
    digitalWrite(CCW_SOLENOID, HIGH);
    delay(500);
    digitalWrite(CCW_SOLENOID, LOW);
    delay(2000);
    Serial.println("SOLENOID: OK\n");
    Serial1.println("SOLENOID: OK\n");

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
    Serial1.println("VECTOR ENTERING CALIBRATION!!!");
    Serial1.println("-------------------------------");
    Serial1.println();
    delay(1000);
    SW_STATE = CALIBRATION;
    Serial.println("TEAM_ID, POWER_TIME, MISSION_TIME, PACKET_COUNT, SW_STATE, CAM_STATE, ALTITUDE, TEMP, ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z, ROLL, PITCH, YAW, SIV, GPS_LAT, GPS_LONG, GPS_ALT");
    Serial1.println("TEAM_ID, POWER_TIME, MISSION_TIME, PACKET_COUNT, SW_STATE, CAM_STATE, ALTITUDE, TEMP, ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z, ROLL, PITCH, YAW, SIV, GPS_LAT, GPS_LONG, GPS_ALT");
}

void loop(void)
{
    PACKET_COUNT++;
    long ALTITUDE = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    long TEMP = bmp.temperature;

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    ACC_X = linearaccel.x();
    ACC_Y = linearaccel.y();
    ACC_Z = linearaccel.z();
    GYRO_X = gyro.x();
    GYRO_Y = gyro.y();
    GYRO_Z = gyro.z();
    ROLL = euler.x();
    PITCH = euler.y();
    YAW = euler.z();
    SIV = gps.getSIV();
    GPS_LAT = gps.getLatitude();
    GPS_LONG = gps.getLongitude();
    GPS_ALT = gps.getAltitudeMSL();

    // print telem to serial
    Serial.println((String)TEAM_ID + ", " + POWER_TIME + ", " + MISSION_TIME + ", " + PACKET_COUNT + ", " + stateNames[SW_STATE] + ", " + CAM_STATE + ", " + ALTITUDE + ", " + TEMP + ", " + ACC_X + ", " + ACC_Y + ", " + ACC_Z + ", " + GYRO_X + ", " + GYRO_Y + ", " + GYRO_Z + ", " + ROLL + ", " + PITCH + ", " + YAW + ", " + SIV + ", " + GPS_LAT + ", " + GPS_LONG + ", " + GPS_ALT);
    Serial1.println((String)TEAM_ID + ", " + POWER_TIME + ", " + MISSION_TIME + ", " + PACKET_COUNT + ", " + stateNames[SW_STATE] + ", " + CAM_STATE + ", " + ALTITUDE + ", " + TEMP + ", " + ACC_X + ", " + ACC_Y + ", " + ACC_Z + ", " + GYRO_X + ", " + GYRO_Y + ", " + GYRO_Z + ", " + ROLL + ", " + PITCH + ", " + YAW + ", " + SIV + ", " + GPS_LAT + ", " + GPS_LONG + ", " + GPS_ALT);

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
        delay(5000);

        // Transition to the IDLE once calibrated
        // if (SIV > 3)
        // {
        //     SW_STATE = IDLE;
        // }
        SW_STATE = RCS_GO;
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
        delay(250);

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

        switch (TARGET_MODE)
        {
        case 1:
            rcs_correct(90, ROLL); // True North
            break;

        case 2:
            // rcs_correct(0, ROLL); // Sun's Azimuth
            break;

        case 3:
            // rcs_correct(0, ROLL); // Direction of Travel
            break;

        case 4:
            // int angle = atan(abs(gps - UAH_W_CO)/abs((gps-UAH_N_CO)));
            // rcs_correct(angle, ROLL); // UAH
            break;
        }

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
        // if (ALTITUDE < 18000 && GPS_ALT < 18000)
        // {
        //     SW_STATE = RCS_LOCK;
        // }
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
        Serial1.println("VECTOR: ANOMALY_DETECTED!!!");

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