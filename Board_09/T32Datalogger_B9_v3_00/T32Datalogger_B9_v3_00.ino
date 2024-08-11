/*
  ---------------------------------------------------------
  PROGRAM INFORMATION
  ---------------------------------------------------------
  Bob Jones Patriot Racing Car Datalogger
  Copyright 2016-2024, All Rights reserved
  This code is property of Patriot Racing and Kris Kasprzak
  This code cannot be used outside of Bob Jones High School
  Code for Teensy 3.2 / 4.0 (with change to RX definitin)
  ---------------------------------------------------------
  COMPILE INSTRUCTIONS
  ---------------------------------------------------------
  Compile Speed:  72MHz
  Optimize:       Smallest code


  WARNINGS
  make sure ILI9341_Menu.h has #define MAX_OPT 16 (or max number of menu items)

  ---------------------------------------------------------
  CODE PURPOSE
  ---------------------------------------------------------
  1. MEASURE: Volts, Amps, Motor Temperature (internal and external), Wheel RPM, G-Force, and GPS location
  2. COMPUTE: Speed, Power, Energy, Averages, Driver Statistics
  3. OUTPUT:  Volts, Amps, Power, etc.
  4. WRITE:   Data to a flash chip, and later download to an SD card for analysis
  5. SEND:    Data Wirelessly to RECEIVER module (see Receiver Code)

*/


/*---------------------------------------------------------*/
//INITIALIZATION
/*---------------------------------------------------------*/

// #define DO_DEBUG

/*-------------------*/
// Code Version
/*-------------------*/

#define CODE_VERSION "B9_v3.00"

/*-------------------*/
// Libraries
/*-------------------*/

#include <avr/io.h>                   // standard library that ships with Teensy
#include <SPI.h>                      // standard lib
#include <PatriotRacing_Utilities.h>  // custom utilities definition
#include <PatriotRacing_Icons.h>      // lib of dedicated defines, struct, etc.
#include <ILI9341_t3_Menu.h>          // Menu library      https://github.com/KrisKasprzak/ILI9341_t3_Menu
#include <ILI9341_t3_Controls.h>      // Controls library  https://github.com/KrisKasprzak/ILI9341_t3_controls
#include <ILI9341_t3.h>               // Display library   https://github.com/PaulStoffregen/ILI9341_t3
#include <SdFat.h>                    // SD card           https://github.com/greiman/SdFat
#include <EEPROM.h>                   // standard library that ships with Teensy
#include <EasyTransfer.h>             // manages struct compression for sending data https://github.com/madsci1016/Arduino-EasyTransfer
#include <TinyGPSPlus.h>              // GPS module lib    https://github.com/mikalhart/TinyGPSPlus
#include <TimeLib.h>                  // time libs         https://github.com/PaulStoffregen/Time https://github.com/PaulStoffregen/Time/blob/master/TimeLib.h
#include <EBYTE.h>                    // transceiver lib   https://github.com/KrisKasprzak/EBYTE
#include <FlickerFreePrint.h>         // eliminates text drawing flicker https://github.com/KrisKasprzak/FlickerFreePrint/blob/master/FlickerFreePrint.h
#include <Arial_100BINO.h>            // special font file that is numbers only. stored in PatriotRacing_Fonts, recreate use https://spooksoft.pl/download/, https://spooksoft.pl/en/download-3/
#include <font_ArialBold.h>           // comes with display or go here https://github.com/PaulStoffregen/ILI9341_fonts
#include <font_Arial.h>               // comes with display or go here https://github.com/PaulStoffregen/ILI9341_fonts
#include <font_ArialBoldItalic.h>     // https://github.com/PaulStoffregen/ILI9341_fonts
#include <BulletDB.h>                 // flash chip database driver https://github.com/KrisKasprzak/BulletDB
#include <Wire.h>
#include <Adafruit_Sensor.h>   // common sensor lib  https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BME280.h>   // altimiter lib      https://github.com/adafruit/Adafruit_BME280_Library
#include "MPU6050.h"           // accelerometer lib  https://github.com/ElectronicCats/mpu6050
#include <FreqMeasureMulti.h>  // lib for speed sensor https://github.com/PaulStoffregen/FreqMeasureMulti

/*-------------------*/
//Constant Definitions
/*-------------------*/

#define SEALEVELPRESSURE_HPA 1013.25
#define GRAPH_X 45
#define GRAPH_Y 210
#define GRAPH_W 255
#define GRAPH_H 160

//Global Fonts and Locations
#define FONT_100BINO Arial_100BINO      // font for the large data
#define FONT_24BI Arial_24_Bold_Italic  // font for the small data
#define FONT_16B Arial_16_Bold          // font for all headings
#define FONT_14 Arial_14                // font for menus

// location for large data font--need to change if different font is used
#define DATA_Y 75
#define DATA_X 5

#define L_LONG 1
#define L_SHORT 2
#define R_LONG 3
#define R_SHORT 4

// error codes for data file re-reading
#define RR_ERROR -1
#define RT_HEADER 0x01
#define RT_DATA 0x02

// menu colors
#define MENU_TEXT C_WHITE
#define MENU_BACK C_BLACK
#define MENU_HIGHTEXT C_WHITE
#define MENU_HIGHLIGHT C_MDBLUE

#define MENU_HIGHBORDER C_DKBLUE
#define MENU_SELECTTEXT C_WHITE
#define MENU_SELECT C_RED
#define MENU_TITLETEXT C_WHITE
#define MENU_TITLEBACK C_DKBLUE

#define DISABLE_WDT 0
#define ENABLE_WDT 1
#define RESET_WDT 2

#define DATA_DELIMITER 44
#define RACE_EXTENSION 300  // seconds we add to the 90 min mark to account for posible race time stop (very rare)
#define RACE_TIME_SECONDS 5400

//Timers for button presses
#define NO_PRESS 0
#define SHORT_PRESS 60
#define LONG_PRESS 1000
#define DEBOUNCE 200
#define DUMMY_MAX 9999
#define STATUS_TYPE 5
#define STATUS_RESULT 145
#define SPEEDMENU_LIMIT 3000
#define SPEEDMENU_DELAY 50

// UART ports
#define ESerial Serial1    // setup serial port for Exx-TTL-100
#define GPSSerial Serial3  // setup serial port for GPS Teensy 3.2
// #define GPSSerial Serial2                     // setup serial port for GPS Teensy 4.0

#define AX_PIN 14   // A0 aux pin for EBYTE
#define THX_PIN A1  // thermisto measurement pin motor internal temp
#define THM_PIN A2  // thermisto measurement pin motor external temp
#define AM_PIN A3   // amp sensor pin
#define KEY_PIN A6  // key status pin
#define M1_PIN A7   // state pin for EBYTE
#define M0_PIN A7   // state pin for EBYTE
#define RPM_PIN 22  // A8 pin for the RPM
#define VM_PIN A9   // voltage divider input

#define UPDATE_LIMIT 500

#define DRS_PIN 2     // display DC/RS pin
#define R_PIN 3       // pin for the down display mode button (uint8_ts to address display upside down)
#define L_PIN 4       // pin for the up display mode button (uint8_ts to address display upside down)C
#define CD_PIN 5      // card detect pin
#define SSD_PIN 6     // chip select for the flash memory chip
#define GPSLED_PIN 8  // we're using this pin to light up an LED when trigger point is crossed
#define DCS_PIN 9     // display chip select
#define SDCS_PIN 10   // CS for SD card

#define RACE_NOTSTARTED 0
#define RACE_INPROGRESS 1
#define RACE_FINISHED 2

#define TIME_HEADER "T"

#define METERS_TO_FEET 3.28084

#define BIT_CONVERSION 4096

#define SD_SPI_SPEED 60

/*-------------------*/
// Program Variables
/*-------------------*/

bool DrawGraph = true;
uint16_t EnergyPoints[100];

uint16_t BLEnergy[93] = { 0, 8, 16, 22, 28, 36, 42, 48, 54, 62, 68, 74, 80, 88, 94, 102, 108, 116, 124, 130, 138, 144, 150, 158,
                          164, 172, 178, 186, 192, 200, 206, 214, 220, 226, 234, 240, 248, 254, 260, 268, 274, 282, 288, 294, 
                          302, 308, 314, 322, 328, 334, 340, 348, 354, 360, 368, 374, 380, 386, 394, 400, 406, 412, 418, 426, 
                          432, 438, 444, 450, 456, 464, 470, 476, 482, 488, 494, 500, 506, 512, 518, 524, 530, 536, 542, 548, 
                          554, 560, 566, 570, 576, 582, 588, 592, 600 };



/*
uint16_t BLEnergy[93] = { 0, 1, 5, 8, 12, 16, 19, 23, 26, 30, 33, 37, 40, 44, 47, 51, 54, 58, 62, 65, 69, 72, 75, 79, 82, 86, 89, 93, 96, 100, 103, 107, 110, 113,
117, 120, 124, 127, 130, 134, 137, 141, 144, 147, 151, 154, 157, 161, 164, 167, 170, 174, 177, 180, 184, 187, 190, 193, 197, 200, 203, 206, 209, 213, 216,
  219, 222, 225, 228, 232, 235, 238, 241, 244, 247, 250, 253, 256, 259, 262, 265, 268, 271, 274, 277, 280, 283, 285, 288, 291, 294, 296, 300
};
*/

uint32_t epoint;
uint16_t EnergyID, bEnergyID, GForceXID, GForceYID, GForceZID, GForceID;

// sensors
bool RPMStatus = false;
bool RadioStatus = false;
bool AltimiterStatus = false;
bool GForceStatus = false;
bool SDCardStatus = false;
bool GPSStatus = false;
bool SSDStatus = false;
bool HasASensor = false;
bool KeySwitchedOn = false;
bool oldKeySwitchedOn = false;
bool KeyJustSwitchedOn = false;
bool DriverKeySwitchedOn = false;
bool ResetAltitude = false;
bool UseGPSForSpeedDisplay = false;
float AltitudeOffset = 0.0f;
float StartAltitude = 0.0f;
uint8_t PressCount = 0;
bool AutoCurrentCal = true;
uint8_t Battery1 = 0;
uint8_t Battery2 = 0;

uint32_t SpeedTimer = 500;

bool AllowCalibrations = false;

bool ResetGPS = false;
bool RestoreData = true;
bool StartGPSFound = false;
uint16_t banner_back = C_YELLOW;
int Tyear = 0;
int Tmonth = 0;
int Tday = 0;
int Thour = 0;
int Tminute = 0;
int GForceXPoint;
uint8_t GForceRange = 4;
uint8_t AccelLPFilter = 0;
uint8_t AccelHPFilter = 0;

bool RestartDisplayAlways = false;
bool AddLapInPit = true;
uint16_t RecordSETID = 0;
uint16_t Record = 0;
uint8_t RecordType = RT_HEADER;
uint32_t LastRecord = 0;
float ASensorBits = 4096.0f;
int16_t AccelCalX = 0;
int16_t AccelCalY = 0;
int16_t AccelCalZ = 0;
uint32_t Duration = 0;

//Setup Variables
unsigned long RadioUpdate = 1;  // index to char array
uint16_t CarID = 0;             // 0 = red, 1 = blue, 3 = white
uint8_t MotorSprocket = 15;     // number of teeth for motor sprocket
uint8_t WheelSprocket = 70;     // number of teeth for driven sprocket
float GearRatio = 4.66666f;     // large, small sprocket teeth
uint8_t TireID = 0;             // tire id for tiretext and tirerad arrays
uint8_t MotorID = 0;            // motor id for tiretext and tirerad arrays
float TireRad = 9.0f;           // will get set later by tire ID
uint16_t TotalEnergy = 600;     // sum of both batteries at 10.5 volt mark
float VoltageSlope = 11.0f;     // Vin is comming through diodes before vvoltage divider--can't use standard equation.
float VoltageOffset = 0.299f;   // Vin is comming through diodes before vvoltage divider--can't use standard equation.
uint8_t Pickups = 4;            // default pickups
uint16_t Invert = 0;            // track white / black display background
uint16_t Orientation = 0;       // track display orientation
uint16_t GPSTolerance = 4;      // default to 8 meter for GPS tolerance

// note the ACS-770 U200 curent sensor has sensitivity of 20 mV/Amp, 0.5 volts offset at 0 amps,
// unit is powered with 5.0 Vcc
float VMid = 0.5f;            // offset for current sensor
float mVPerAmp = 20.0f;       // sensitivity for current sensor
float BatWarning = 21.0f;     // default voltage for batter warning
float TempWarning = 140.0f;   // default temp for motor warning
float ThermResMotor = 10000;  // voltage divider resistor for external thermistor sensor
float ThermResAux = 10000;    // voltage divider resistor for external thermistor sensor
uint8_t LapThreashold = 30;   // time in seconds required to elapse before another lap is allowed to be counted
uint16_t PacketSize = 0;      // transceiver packet size
uint8_t LPin = L_PIN;         // pin for the up display mode button (uint8_ts to address display upside down)
uint8_t RPin = R_PIN;         // pin for the down button
float ax = 0.0, ay = 0.0, az = 0.0;
int16_t gx = 0.0, gy = 0.0, gz = 0.0;
float GForceX = 0.0, GForceY = 0.0, GForceZ = 0.0, GForce = 0.0;
uint32_t amt = 0;
int16_t ByteWidth = 0;
uint8_t sByte = 0, b = 0;
char c;
char str[50];  // char for time buffers
char buf[50];  // generic buffer for various uses

//Time Variables (RTCTime is a time_t object; others are used to set RTCTime)
time_t RTCTime;
int16_t hours = 0, minutes = 0, seconds = 0, days = 0, months = 0, years = 0;
int16_t RaceDay = 0, RaceHour = 0, RaceMinute = 0, RaceSecond = 0, RaceMonth = 0;

// accelerometer calibration variables
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int buffersize = 1000;  //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;  //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;

uint8_t RaceStatus = RACE_NOTSTARTED;

float TriggerAmps = 30.0f;  // Minimum amp required to trigger a race start or dirver change

// Speed and Distance Variables
uint32_t RPMSum = 0.0;
uint32_t RPMCount = 0;

uint32_t MinPulses = 3;
float Revolutions = 0.0;        // counts total to compute Distance
volatile uint32_t Counter = 0;  // the number of measurements between each display
uint32_t AverageCounter = 0;

//Transceiver Variables
float TargetAmps = 0.0f;
float ERem = 100.0f, TRem = 100.0f;  // Energy and Time remaining

uint32_t StartPage = 0;

//Driver Variables
uint8_t Driver = 0;  // variable for current driver 0-2

uint8_t DriverID[3] = { 0, 0, 0 };     // Array for Driver ID
uint8_t DriverLaps[3] = { 0, 0, 0 };   // Array for number of laps per driver
uint32_t DriverTime[3] = { 0, 0, 0 };  // Array for driver time

//Transceivers Variables
uint16_t AirDataRate = 0, oAirDataRate = 0;
uint16_t RadioChannel = 0, oRadioChannel = 0;

//Car Variables
uint16_t WRPM = 0;  // wheel rpm (measured)
uint16_t mRPM = 0;  // motor rpm (calculated)
uint16_t val = NO_PRESS;
float vVolts = 0.0f, Volts = 0.0f, MinVolts = DUMMY_MAX;                                                 // computed Volts
float thmVolts = 0.0f, thxVolts = 0.0f, AmbTemp = 0.0f, MotorTemp = 0.0f, AuxTemp = 0.0f, TempK = 0.0f;  // computed temp values
float AmbTempCF = 0.0f;
uint8_t RedrawHeader = false;
float aVolts = 0.0f, Amps = 0.0f, MaxAmps = 0.0f;  // computed Amps
float tr2 = 0.0f;                                  // computed thermistor resistance
float Power = 0.0f;                                // computed Power
float Energy = 0.0f;                               // computed Energy basically Power x time slice
float CarSpeed = 0.0f, CarMaxSpeed = 0.0f;         // computed speed
float Distance = 0.0f;                             // computed distance
uint16_t h = 0, m = 0, s = 0;                      // for formatting min and sec
uint16_t Point = 0;                                // Counter for the data Point

//Buttons
uint8_t DisplayID = 0, OldDisplayID = 0;  // right button tracker
uint32_t ButtonDebounce = 200;            // may sound long but drivers have gloves
uint8_t Reset = 0;

//Variables for Average Calculations
uint8_t LapCount = 0;
uint32_t AverageCount = 0;
float LapAmps = 0.0, LapVolts = 0.0f, AverageAmps = 0.0f;
float AverageVolts = 0.0f, LapEnergy = 0.0f, StartLapEnergy = 0.0f;
float iTemp = 0.0f, iPressure = 0.0f, Altitude = 0.0f, Humidity = 0.0f;

uint8_t StartGPSDelayID = 0;
uint32_t StartGPSDelay = 0;

// GPS and Channel Variables
float GPSLat = 0.0, GPSLon = 0.0, GPSSpeed = 0.0, GPSStartLat = 0.0, GPSStartLon = 0.0, GPSDistance = 0.0;

uint16_t LapTime = 0, LastLapTime = 0, TimeSplit = 0;
uint16_t GPSSatellites = 0;
uint16_t fore_color = 0, back_color = 0;  // remember the foreground and background colors
uint16_t i = 0, j = 0, NewMCU = 0;        // just some storage variables
uint8_t RestoreType = STATUS_OK;

//Warning Global Variable
uint16_t Warnings = 0;

// menu ID variables
uint8_t MainMenuOption = 0, MainMenuOption1 = 0, MainMenuOption2 = 0, MainMenuOption3 = 0;
uint8_t MainMenuOption4 = 0, MainMenuOption5 = 0, MainMenuOption6 = 0, MainMenuOption7 = 0;
uint8_t MenuOption = 0;

uint8_t RaceMenuOption1 = 0, RaceMenuOption2 = 0, RaceMenuOption3 = 0, RaceMenuOption4 = 0;
uint8_t RaceMenuOption5 = 0, RaceMenuOption6 = 0, RaceMenuOption7 = 0, RaceMenuOption8 = 0, RaceMenuOption9 = 0, RaceMenuOption10 = 0;

uint8_t TelemetryMenuOption1 = 0, TelemetryMenuOption3 = 0, TelemetryMenuOption4 = 0, TelemetryMenuOption5 = 0, TelemetryMenuOption6 = 0;
uint8_t TelemetryMenuOption7 = 0, TelemetryMenuOption8 = 0, TelemetryMenuOption9 = 0;

uint8_t WirelessMenuOption1 = 0, WirelessMenuOption2 = 0, WirelessMenuOption3 = 0, WirelessMenuOption4 = 0;
uint8_t WirelessMenuOption5 = 0, WirelessMenuOption7 = 0, WirelessMenuOption8 = 0;

uint8_t SensorMenuOption1 = 0, SensorMenuOption2 = 0, SensorMenuOption3 = 0, SensorMenuOption4 = 0, SensorMenuOption5 = 0;
uint8_t SensorMenuOption6 = 0, SensorMenuOption7 = 0, SensorMenuOption8 = 0, SensorMenuOption9 = 0, SensorMenuOption10 = 0, SensorMenuOption11 = 0;
uint8_t SensorMenuOption12 = 0, SensorMenuOption13 = 0, SensorMenuOption14 = 0, WirelessMenuOption6 = 0;

uint8_t GForceMenuOption1 = 0, GForceMenuOption2 = 0, GForceMenuOption3 = 0, GForceMenuOption4 = 0, GForceMenuOption5 = 0;
uint8_t GForceMenuOption6 = 0, GForceMenuOption7 = 0, GForceMenuOption8 = 0;
uint8_t ClockMenuOption1 = 0, ClockMenuOption2 = 0, ClockMenuOption3 = 0, ClockMenuOption4 = 0, ClockMenuOption5 = 0, ClockMenuOption6 = 0, ClockMenuOption7 = 0;
uint8_t SSDMenuOption1 = 0, SSDMenuOption2 = 0, SSDMenuOption3 = 0, SSDMenuOption4 = 0, SSDMenuOption5 = 0;

// field ID variables
uint8_t frType = 0, frID = 0, frPoint = 0, frLap = 0, frDriver = 0, frVolts = 0, frAmps = 0, frMotorTemp = 0, frAuxTemp = 0, frEnergy = 0, frRPM = 0;
uint8_t frSpeed = 0, frDist = 0, frRT = 0, frLon = 0, frLat = 0, frAlt = 0, frGSpeed = 0, frRestoreType = 0, frMax = 0, frMay = 0, frMaz = 0;
uint8_t frHumidity = 0, frAmbTemp = 0;

// header ID variables
uint8_t hrType = 0, hrID = 0, hrYear = 0, hrMonth = 0, hrDay = 0, hrHour = 0, hrMinute = 0, hrMSprocket = 0, hrWSprocket = 0, hrTireID = 0;
uint8_t hrD0ID = 0, hrD1ID = 0, hrD2ID = 0, hrCarID = 0, hrMotorID = 0, hrTemp = 0, hrPressure = 0, hrHumidity = 0, hrAltitude = 0, hrAmbTemp = 0;
uint8_t hrEnergy = 0, hrCounter = 0, hrBattery1 = 0, hrBattery2 = 0, hrLon = 0, hrLat = 0;

uint16_t ReturnCode = 0;
uint32_t UsedSpace = 0;
uint32_t RealClockTime = 0;
uint32_t fc = 0;

/*---------------------------------------------------------*/
//OBJECT INITIALIZATION
/*---------------------------------------------------------*/

ILI9341_t3 Display(DCS_PIN, DRS_PIN);  //Display object

TinyGPSPlus GPS;

SdFat SDCARD;
SdFile SDDataFile;
SdFile SDSetupFile;
Transceiver Data;  //Transceiver data structure

// make sure AUX and TX pins have 4K7 pullups
EBYTE Radio(&ESerial, M0_PIN, M1_PIN, AX_PIN);  //Transceiver object

EasyTransfer DataPacket;

FlickerFreePrint<ILI9341_t3> ffVolts(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffLapVolts(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffMinVolts(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffAmps(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffSpeed(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffLapSpeed(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffMaxSpeed(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffWRPM(&Display, C_BLUE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffEnergy(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffLapAmps(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffLapEnergy(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffMaxAmps(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffLaps(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffDriver(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffCarRaceTimer(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffDriverTime(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffThisLap(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffLastLap(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffBestLap(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffSplitLap(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffTime(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffDate(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffACalX(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffACalY(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffACalZ(&Display, C_WHITE, C_BLACK);

ItemMenu TopMainMenu(&Display);
ItemMenu SSDMenu(&Display);

EditMenu RaceMenu(&Display);
EditMenu TelemetryMenu(&Display);
EditMenu WirelessMenu(&Display);
EditMenu GForceMenu(&Display);
EditMenu SensorMenu(&Display);
EditMenu ClockMenu(&Display);

elapsedMillis GraphDrawTimer = 0;
elapsedMillis RadioUpdateTimer = 0;
elapsedMillis SaveTimer = 0;
elapsedMillis GraphStoreTimer = 0;
elapsedMillis LapLEDTimer = 0;
elapsedMillis DisplayUpdateTimer = 0;
elapsedMillis GPSUpdateTimer = 0;
elapsedMillis GPSLapTimer = 0;
elapsedMillis CarRaceTimer = 0;
elapsedMillis DriverTimer = 0;
elapsedMillis GPSMaxReadTimer = 0;
elapsedMillis LapTimer = 0;
elapsedMillis SpeedUpdateTimer = 0;
elapsedMillis PressTimer = 0;
elapsedMillis DriverChangeScreen = 0;
elapsedMillis StartGPSDelayTimer = 0;
Adafruit_BME280 BMEsensor;

MPU6050 ASensor;

BulletDB SSD(SSD_PIN);

CGraph EnergyG(&Display, GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, 0, 90, 15, 0, 700, 100);
CGraph GForceG(&Display, 45, 210, 270, 160, 0, 40, 40, -3, 3, 1);

BarChartV MotorTempG(&Display);
BarChartV AuxTempG(&Display);
BarChartV AmbTempG(&Display);

FreqMeasureMulti RPM;

/*---------------------------------------------------------*/
// PROGRAM FUNCTIONS
/*---------------------------------------------------------*/

/*
   PURPOSE : Setup Datalogger device upon turning on power
   PARAMS :  -
   RETURNS : None
   NOTES : Runs at the beginning of turning on power
*/

void setup() {

  Serial.begin(115200);

  // while (!Serial) {}
  Serial.println("Starting");

  // setup pin modes
  pinMode(RPM_PIN, INPUT);  // if using LED beam breaker, CANNOT have pullup (software or hardware)
  pinMode(VM_PIN, INPUT_DISABLE);

  // note the current sensor has a pull-down resistor through a voltage divider
  // where key off = 0 volts (plus some potential noise fluctuation), key on = 1000/11000 * battery voltage
  pinMode(KEY_PIN, INPUT_PULLDOWN);

  pinMode(AM_PIN, INPUT_DISABLE);
  pinMode(L_PIN, INPUT_PULLUP);
  pinMode(R_PIN, INPUT_PULLUP);
  pinMode(CD_PIN, INPUT_PULLUP);

  Display.begin();  // start the display

  StartRTC();

  Wire.begin();

  SSDStatus = SSD.init();

  ////////////////////////////////////
  // this is the magic trick for printf to support float
  asm(".global _printf_float");

  // this is the magic trick for scanf to support float
  asm(".global _scanf_float");

  GetParameters();

  Display.fillScreen(C_BLACK);

  SetScreenParameters();

  Display.setFont(FONT_24BI);

  if (CarID == 0) {
    Display.fillRect(0, 0, 319, 35, C_BLUE);
    Display.setTextColor(C_WHITE);
  } else if (CarID == 1) {
    Display.fillRect(0, 0, 319, 35, C_RED);
    Display.setTextColor(C_WHITE);
  } else {
    Display.fillRect(0, 0, 319, 35, C_WHITE);
    Display.setTextColor(C_BLACK);
  }

  Display.setCursor(10, 5);
  Display.print(F("PATRIOT RACING"));

  Display.setFont(FONT_14);
  Display.setTextColor(C_WHITE);
  Display.setCursor(STATUS_TYPE, 40);
  Display.print(F("G-Force: X,Y,Z"));
  Display.setCursor(STATUS_TYPE, 60);
  Display.print(F("Volts / Amps"));
  Display.setCursor(STATUS_TYPE, 80);
  Display.print(F("Alt / Temp / RH"));
  Display.setCursor(STATUS_TYPE, 100);
  Display.print(F("Temp M / X / A"));
  Display.setCursor(STATUS_TYPE, 120);
  Display.print(F("GPS"));
  Display.setCursor(STATUS_TYPE, 140);
  Display.print(F("Speed sensor"));
  Display.setCursor(STATUS_TYPE, 160);
  Display.print(F("Wireless"));
  Display.setCursor(STATUS_TYPE, 180);
  Display.print(F("Flash memory"));
  Display.setCursor(STATUS_TYPE, 200);
  Display.print(F("Status"));
  Display.setCursor(STATUS_TYPE, 220);
  Display.print(F("Records: "));

  ESerial.begin(9600);

  GPSSerial.begin(9600);  // GPS

  delay(10);

  // this line must come after the GPSSerial as we are using the MCU Tx line for
  // led indicator
  pinMode(GPSLED_PIN, OUTPUT);

  BuildFieldList();

  LastRecord = SSD.findLastRecord();

  SSD.gotoRecord(LastRecord);
  RecordSETID = SSD.getField(RecordSETID, frID);

  ///////////////////////////////////////
  if (RecordSETID == 0xFFFF) {
    // this will happen on new card
    RecordSETID = 0;
  }
  if (RecordSETID == CHIP_FORCE_RESTART) {
    SaveStartGPS(false);
    ResetRaceDate();
  }

#ifdef DO_DEBUG
  Serial.print("Chip JEDEC: ");
  Serial.println(SSD.getChipJEDEC());
  Serial.print("Last Record: ");
  Serial.println(SSD.getLastRecord());
  Serial.print("Current Record: ");
  Serial.println(SSD.getCurrentRecord());
  Serial.print("Last RecordSetID: ");
  Serial.println(RecordSETID);
  Serial.print("used space (b): ");
  Serial.println(SSD.getUsedSpace());
  SSD.dumpBytes(0, 1500);
  SSD.listFields();
  SSD.listHeaderFields();
#endif

  if (SSD.getUsedSpace() > 6000000000) {

    Warnings = Warnings | SSD_FAIL;
  }
  RaceStatus = RACE_NOTSTARTED;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // need > 90 min hours between race1 start and race2 start to consider a new race
  // and test if current race could be restored
  // duration is in seconds

  Duration = ((hour() * 3600) + (minute() * 60) + second()) - ((RaceHour * 3600) + (RaceMinute * 60) + RaceSecond);
  if ((RestoreData) && (RaceMonth == month()) && (RaceDay == day()) && (Duration < (95 * 60))) {
    RaceStatus = RACE_INPROGRESS;
  }

  CreateUserInterface();

  //Get the structure size for the transceiver
  PacketSize = sizeof(Data);

  //Configure up/down
  ConfigureButtons();

  InitializeSensors();

  // JenniferMonitor();


  UsedSpace = SSD.getUsedSpace();

  if (!SSDStatus) {
    Display.setTextColor(C_RED);
    Display.setCursor(STATUS_RESULT, 180);
    Display.print(F("FAIL"));
    Warnings = Warnings | SSD_FAIL;
  } else {
    if ((UsedSpace + 300000) < CARD_SIZE) {
      Display.fillRoundRect(STATUS_RESULT, 180, 160, 18, 2, C_DKGREEN);
      Display.fillRoundRect(STATUS_RESULT, 180, ((float)(UsedSpace * 160.0) / CARD_SIZE) + 2, 18, 2, C_GREEN);
    } else {
      Display.fillRoundRect(STATUS_RESULT, 180, 160, 185, 2, C_DKRED);
      Display.fillRoundRect(STATUS_RESULT, 180, ((float)(UsedSpace * 160.0) / CARD_SIZE) + 2, 18, 2, C_RED);
    }
  }

#ifdef DO_DEBUG
  Serial.print("Current Time   ");
  Serial.print("Month/Day/Hour ");
  Serial.print(month());
  Serial.print("/");
  Serial.print(day());
  Serial.print(", ");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.println(second());

  Serial.print("EEPROM Time   ");
  Serial.print("Month/Day/Hour ");
  Serial.print(RaceMonth);
  Serial.print("/");
  Serial.print(RaceDay);
  Serial.print(", ");
  Serial.print(RaceHour);
  Serial.print(":");
  Serial.print(RaceMinute);
  Serial.print(":");
  Serial.println(RaceSecond);

  Serial.print("Duration:");
  Serial.print(Duration);
  Serial.print(", min:");
  Serial.println(Duration / 60.0);

#endif

  if (RaceStatus == RACE_INPROGRESS) {

#ifdef DO_DEBUG
    Serial.println("Current race, restoring data file.");
#endif
    // the race is still on so use same file and restore any past data
    // get the GPS start as well (which was restored from Get Parameters
    // note we assume race start GPS was successfully captured

    if ((GPSStartLat == 0) || (GPSStartLon == 0)) {
      StartGPSFound = false;
    } else {
      StartGPSFound = true;
    }
    // restore data
    Display.setFont(FONT_14);
    Display.setTextColor(C_CYAN);
    Display.setCursor(STATUS_RESULT, 200);
    Display.print(F("CURRENT"));

    ReturnCode = RestoreRaceData();

    Display.fillRect(STATUS_RESULT, 220, 320 - STATUS_RESULT, 25, C_BLACK);

    Display.setCursor(STATUS_RESULT, 220);

    if (ReturnCode == RR_ERROR) {
      Display.setTextColor(C_RED);
      Display.print(F("FAIL to restore"));
      delay(10);
      // consider this now a new race dataset
      Display.setFont(FONT_14);
      Display.setTextColor(C_CYAN);
      Display.setCursor(STATUS_RESULT, 220);
      Display.print(F("NEW: "));

      RecordSETID++;
      Display.print(RecordSETID);
      RaceStatus = RACE_NOTSTARTED;
      ResetRaceDate();
      SaveStartGPS(false);
      delay(10);
    } else {
      // return code is seconds of downtime
      // restore success, continue with race
      RaceStatus = RACE_INPROGRESS;
      Display.setTextColor(C_CYAN);
      Display.print(F("Restored: "));
      h = (int)(ReturnCode / 3600);
      m = (int)(ReturnCode / 60);
      s = (int)(ReturnCode % 60);
      sprintf(str, "%02d:%02d:%02d", h, m, s);
      // display the amount of time we had to restore
      Display.print(str);
    }
  }

  if (RaceStatus == RACE_NOTSTARTED) {

    // either the race hasnt' started yet, or attempt to restire race faile
    // either way, treat this like it's a new race and move on
    // reset GPS start just in case race did not end naturally

    Display.setFont(FONT_14);
    Display.setTextColor(C_CYAN);
    Display.setCursor(STATUS_RESULT, 200);
    Display.print(F("NEW"));

    Display.setCursor(STATUS_RESULT, 220);
    ResetRaceDate();
    SaveStartGPS(false);

    if (SSDStatus) {
      Display.print(F("RecordSETID: "));
      Display.print(RecordSETID);
    } else {
      Display.setCursor(5, 220);
      Display.setTextColor(C_RED);
      Display.print(F("SDD FAILED"));
    }

#ifdef DO_DEBUG
    Serial.println("New Race.");
    Serial.print("RecordSETID: ");
    Serial.println(RecordSETID);
#endif
  }

  delay(100);

  // display big giant errors
  if (RaceStatus == RACE_NOTSTARTED) {

    DisplayErrors();
  }


  Display.fillScreen(back_color);  //Once done with setup, transition to showing stats

  WatchDogTimer(ENABLE_WDT);

  GraphDrawTimer = 0;
  GraphStoreTimer = 0;
  LapLEDTimer = 0;
  DisplayUpdateTimer = 0;
  RadioUpdateTimer = 0;
  SaveTimer = 0;

  GPSLapTimer = 0;
  GraphDrawTimer = 60000;
  GPSMaxReadTimer = 0;
  SpeedUpdateTimer = 0;
}

/*
  PURPOSE : Main Program Loop
  PARAMS :  -
  RETURNS : None
  NOTES : Continuously runs and checks the car's position to compute data
*/

void loop() {

  //Counter for computing averages; read values until time to display and then compute averages
  Counter++;

  //Measure volts, amps, and temperature
  vVolts = vVolts + (analogRead(VM_PIN));
  aVolts = aVolts + (analogRead(AM_PIN));

  if ((GForceStatus) & (HasASensor)) {

    ax = ASensor.getAccelerationX() / ASensorBits;
    ay = ASensor.getAccelerationY() / ASensorBits;
    az = ASensor.getAccelerationZ() / ASensorBits;

    if (abs(ax) > abs(GForceX)) {
      GForceX = ax;
    }

    if (abs(ay) > abs(GForceY)) {
      GForceY = ay;
    }

    if (abs(az) > abs(GForceZ)) {
      GForceZ = az;
    }
  }

  ButtonPress();

  if (GPSTolerance != 0) {
    GPSRead();
  }

  if (RPM.available()) {
    RPMSum = RPMSum + RPM.read();
    RPMCount++;
    Revolutions += 1.0f / (float)Pickups;
  }

  //Check if we can update the display and them compute, send, and save
  if (SpeedUpdateTimer >= SpeedTimer) {
    ComputeSpeed();
    SpeedUpdateTimer = 0;
    RPMSum = 0;
    RPMCount = 0;
  }


  //Check if we can update the display and them compute, send, and save
  if (DisplayUpdateTimer >= UPDATE_LIMIT) {
    DisplayUpdateTimer = 0;
    WatchDogTimer(RESET_WDT);

    // stop race if longer than pre choosen race time
    // add some time to account for if GPUSA stops clock (red flag?).

    if ((RaceStatus == RACE_INPROGRESS) && ((CarRaceTimer / 1000) >= (RACE_TIME_SECONDS + RACE_EXTENSION))) {

      RaceStatus = RACE_FINISHED;
      // set restoring data to 0
      ResetRaceDate();
      SaveStartGPS(false);
      digitalWrite(GPSLED_PIN, LOW);
    }

    //Compute all data
    ComputeData();

    if (CarSpeed > 7.0f) {
      SpeedTimer = 500;
    } else if (CarSpeed > 4.0f) {
      SpeedTimer = 1000;
    } else if (CarSpeed > 0.0f) {
      SpeedTimer = 2000;
    } else {
      SpeedTimer = 500;
    }


    // Check for key state update
    // key state serves 2 purposes 1) display color header on screens 2) auto-driver change
    // hence we need to flags to track each
    if (digitalRead(KEY_PIN) == HIGH) {  // Detected that key is on
      banner_back = C_RED;
      RestoreType = STATUS_OK;
      KeySwitchedOn = true;

      // for display color
      if ((!oldKeySwitchedOn) && (KeySwitchedOn)) {
        KeyJustSwitchedOn = true;
      }

      // for driver change
      if (!oldKeySwitchedOn) {
        DriverKeySwitchedOn = true;
      }

    } else {
      banner_back = C_DKGREEN;
      KeySwitchedOn = false;
      RestoreType = STATUS_PITSTOP;
      Warnings = Warnings | KEY_OFF;

      DriverKeySwitchedOn = false;
    }
    CheckIfStarting();
    GetStartGPS();

    if (GraphStoreTimer >= 60000l) {
      GraphStoreTimer = 0;
      epoint = CarRaceTimer / 60000l;
      if (epoint < 0) {
        epoint = 0;
      }
      if (epoint > 95) {
        epoint = 95;
      }
      EnergyPoints[epoint] = Energy;
    }

#ifdef DO_DEBUG
    // displays all values to the serial monitor
    Debug();
#endif
    RealClockTime = (hour() * 3600) + (minute() * 60) + second();

    if (RaceStatus == RACE_INPROGRESS) {

      // see if we have passed the start point and if so trigger a lap
      if (GPSTolerance > 0) {
        CheckIfLap();
      }

      RecordType = RT_DATA;

      if (SSD.addRecord()) {
        SSD.saveRecord();
      }
      Point++;
    }

    if (GPSTolerance == 0) {
      // GPS turned off
      digitalWrite(GPSLED_PIN, LOW);
    } else {
      if (!GPSStatus) {
        digitalWrite(GPSLED_PIN, HIGH);
      } else {
        if (LapLEDTimer >= 2000) {
          digitalWrite(GPSLED_PIN, LOW);
        }
      }
    }

    // set the warning states for reciever
    if (!SSDStatus) {
      Warnings = Warnings | SD_WARNING;
    }

    // set the warning states for reciever
    if (RaceStatus == RACE_INPROGRESS) {
      Warnings = Warnings | RACE_START;
    }
    if (Volts < BatWarning) {
      Warnings = Warnings | BAT_WARNING;
    }
    if (Amps > 70.0) {
      Warnings = Warnings | AMP_WARNING;
    }

    if ((GForce > 2.0) || (!GForceStatus)) {
      Warnings = Warnings | GFORCE_WARNING;
    }

    if (LapAmps > 18.0) {
      Warnings = Warnings | LAPAMP_WARNING;
    }

    if ((MotorTemp > TempWarning) | (MotorTemp < 10.0f)) {
      Warnings = Warnings | TEMP_WARNING;
    }

    if (!GPSStatus) {
      Warnings = Warnings | GPS_WARNING;
    }

    if (!RPMStatus) {
      Warnings = Warnings | SPEED_FAIL;
    }

    //GForceStatus = false;

    //uint32_t bTime = millis();
    ASensor.initialize();
    SetupAccelerometer();
    //Serial.print("Restart time ");
    //Serial.print(millis() - bTime);
    //Serial.print(", Counter ");
    //Serial.println(Counter);

    GForceStatus = true;

    // user option to restart the display every time or on screen change
    // may need every draw as display can show artifacts due to high electrical noise
    // note since we don't have MISO connected we can't read display status--hence restarting may be needed
    // works...but takes 120 ms to restart
    // if the display pinouts are ever changed on the PCB- you must send MOSI and GND on a twisted pair,
    // and Vcc and SCK on a twisted pair--otherwise the display will freak out
    // also using 100 ohm series resistors to offset wire capacitance

    if (RestartDisplayAlways) {
      Display.begin();
      SetScreenParameters();
    }

    // store the driver time
    // need to do this before display so race time and driver type synch up
    if (RaceStatus == RACE_INPROGRESS) {
      DriverTime[Driver] = DriverTimer;
    }

    //Switch views upon button press (left or right)
    // RedrawHeader = false;
    if ((DisplayID != OldDisplayID) || (oldKeySwitchedOn != KeySwitchedOn)) {
      RedrawHeader = true;
      OldDisplayID = DisplayID;
    }
    // the DriverChange screen is only for when we have a new driver, show the welcome screen.
    if (DriverChangeScreen > 3000) {
      switch (DisplayID) {
        case 0:
          TimeView();
          break;
        case 1:
          VoltsView();
          break;
        case 2:
          AmpsView();
          break;
        case 3:
          SpeedView();
          break;
        case 4:
          TempView();
          break;
        case 5:
          UsageView();
          break;
        case 6:
          EnergyView();
          break;
        case 7:
          GForceView();
          break;
      }

      //Show warning icons in header of driver display
      DrawWarnings();
    }


    // reset the counters
    vVolts = 0.0f;
    aVolts = 0.0f;
    Counter = 0;
    GPSStatus = false;
    oldKeySwitchedOn = KeySwitchedOn;
  }

  if (RadioUpdate != 0) {
    if (RadioUpdateTimer >= (RadioUpdate * 1000)) {

      RadioUpdateTimer = 0;
      SendData();
      //since we display data more often than send some, data must get reset here
      Warnings = 0;
      GForceX = 0.0;
      GForceY = 0.0;
      GForceZ = 0.0;
    }
  }
}

/*---------------------------------------------------------*/
//PRIMARY FUNCTIONS
/*---------------------------------------------------------*/

/*
  PURPOSE : Computes data for the car
  PARAMS :  -
  RETURNS : None
  NOTES : Called by loop()
*/

void ComputeSpeed() {

  // get car speed from wheel RPM
  WRPM = 0.0;

  if (RPMCount >= MinPulses) {
    WRPM = (60.0f / Pickups) * RPM.countToFrequency(RPMSum / RPMCount);
  }
  if ((WRPM > 4000) || (WRPM < 0)) {
    WRPM = 0;
  }

  mRPM = WRPM * GearRatio;
  // compute the car speed
  // v = omega * r
  // v = velocity
  // omega is radians per time
  // r is tire radius
  // use formula and convert units of measure and convert rpm to radians

  CarSpeed = (WRPM * TireRad * 2.0f * 3.14159f * 60.0f / (12.0f * 5280.0f));

  if ((CarSpeed > 99.0f) || (CarSpeed < 0.0f)) {
    CarSpeed = 0.0f;
  }
  if (CarSpeed > CarMaxSpeed) {
    CarMaxSpeed = CarSpeed;
  }

  // get the driven Distance in miles
  Distance = (Revolutions * TireRad * 2.0f * 3.1416f) / (12.0f * 5280.0f);
}

void ComputeData() {

  // get the battey voltage
  vVolts = vVolts / Counter;
  vVolts = vVolts / (BIT_CONVERSION / 3.3f);

  Volts = (vVolts * VoltageSlope) + VoltageOffset;

  if ((Volts > 99.0f) || (Volts < 0.0f)) {
    Volts = 0.0f;
  }

  if (Volts < MinVolts) {
    MinVolts = Volts;
  }

  // get current draw
  aVolts = aVolts / Counter;
  aVolts = aVolts / (BIT_CONVERSION / 3.3f);

  Amps = ((aVolts - VMid) * 1000.0f) / mVPerAmp;

  if ((Amps > 199.0f) || (Amps < -99.0f)) {
    Amps = 0.0f;
  }
  if (Amps > MaxAmps) {
    MaxAmps = Amps;
  }

  // compute motor casing temperature
  // no need to average, just one read is fine
  thmVolts = analogRead(THM_PIN);
  thmVolts = thmVolts / (BIT_CONVERSION / 3.3f);
  // voltage divider calculation
  // vo = 5 * r2 /(r1+r2)
  // solve for r2
  // get the exact value for voltage divider r2
  tr2 = (thmVolts * ThermResMotor) / (3.3f - thmVolts);
  //equation from data sheet
  TempK = 1.0f / (NTC_A + (NTC_B * (log(tr2 / 10000.0f))) + (NTC_C * pow(log(tr2 / 10000.0f), 2)) + (NTC_D * pow(log(tr2 / 10000.0f), 3)));
  MotorTemp = (TempK * 1.8f) - 459.67f;
  if ((MotorTemp > 299.0f) || (MotorTemp < 0.0f)) {
    MotorTemp = 0.0f;
  }

  // compute motor exhaust temperature
  // no need to average, just one read is fine
  thxVolts = analogRead(THX_PIN);
  thxVolts = thxVolts / (BIT_CONVERSION / 3.3f);
  // voltage divider calculation
  // vo = 5 * r2 /(r1+r2)
  // solve for r2
  // get the exact value for voltage divider r1
  tr2 = (thxVolts * ThermResAux) / (3.3f - thxVolts);
  //equation from data sheet
  TempK = 1.0f / (NTC_A + (NTC_B * (log(tr2 / 10000.0f))) + (NTC_C * pow(log(tr2 / 10000.0f), 2)) + (NTC_D * pow(log(tr2 / 10000.0f), 3)));
  AuxTemp = (TempK * 1.8f) - 459.67f;
  if ((AuxTemp > 499.0f) || (AuxTemp < 0.0f)) {
    AuxTemp = 0.0f;
  }

  // compute Power
  Power = Volts * Amps;

  // compute Energy
  Energy = Energy + (Power * (UPDATE_LIMIT / 3600000.0f));

  // compute remaining Energy note total Energy is based on battery tests
  ERem = ((TotalEnergy - Energy) / TotalEnergy) * 100.0f;
  if (ERem < 0.0f) {
    ERem = 0.0f;
  }
  if (ERem > 100.0f) {
    ERem = 100.0f;
  }

  // do we have GPS?
  // is it connected (buffer filling) AND is the GPSLat & GPSLon valid

  if (GPSTolerance > 0) {
    GPSStatus = false;
    GPSLat = GPS.location.lat();
    GPSLon = GPS.location.lng();
    GPSSpeed = GPS.speed.mph();
    GPSSatellites = GPS.satellites.value();

    if (GPS.location.isValid()) {
      GPSStatus = true;
    }

    if (StartGPSFound) {
      GPSDistance = GPS.distanceBetween(GPSStartLat, GPSStartLon, GPSLat, GPSLon);  // in meters
    } else {
      GPSDistance = 0.0;
    }
  }

  if (AltimiterStatus) {
    Humidity = BMEsensor.readHumidity();
    // option to offset the altitude based on initial reading at a known elevation (the start)
    // since barametric pressure drifts, so does altitude
    // at start get StartAltitude, and if option is set compute AltitudeOffset

    Altitude = BMEsensor.readAltitude(SEALEVELPRESSURE_HPA);
    AmbTemp = (BMEsensor.readTemperature() * 1.8) + 32.0 + AmbTempCF;

    // if we ever want to use the accelerometer to read ambient
    // AmbTemp = (((ASensor.getTemperature() / 340.0) + 36.53) * 1.8) + 32.0 + AmbTempCF;
    // Serial.println(AmbTemp);

    Altitude = Altitude * METERS_TO_FEET;
    Altitude = Altitude + AltitudeOffset;
    if (Altitude > 10000.0f) {
      Altitude = 0.0f;
    }
  }

  // build averages
  AverageCount++;
  AverageAmps = AverageAmps + Amps;
  AverageVolts = AverageVolts + Volts;

  GForce = sqrt((GForceX * GForceX) + (GForceY * GForceY) + (GForceZ * GForceZ));

  if (RaceStatus == RACE_INPROGRESS) {

    TRem = ((RACE_TIME_SECONDS - (CarRaceTimer / 1000.0f)) / RACE_TIME_SECONDS) * 100.0f;

    if (TRem < 0.0f) {
      // End of race
      TRem = 0.0f;
    }
  }
}

void GetStartGPS() {

  if (RaceStatus == RACE_INPROGRESS) {

    if (!StartGPSFound) {
      if (StartGPSDelayTimer > StartGPSDelay) {
        if (GPS.location.isValid()) {
          GPSStatus = true;
          SaveStartGPS(true);
          digitalWrite(GPSLED_PIN, HIGH);
          LapLEDTimer = 0;
        }
      }
    }
  }
}

void CheckIfStarting() {

  if (Amps >= TriggerAmps) {

    if (DriverKeySwitchedOn) {

      DriverKeySwitchedOn = true;
      // test if we are starting from beginning or after a pit
      if (RaceStatus == RACE_NOTSTARTED) {
        // must be start of race
        ShowNewDriverScreen();


        // RaceStatus is computed from time comparison in the EEPROM to MCU time
        // if RaceStatus then we need to use EEPROM GPS otherwise get the stuff from the eeprom
        RaceStatus = RACE_INPROGRESS;

        StartAltitude = Altitude;

        RaceMonth = month();
        RaceDay = day();
        RaceHour = hour();
        RaceMinute = minute();
        RaceSecond = second();

        EEPROM.put(360, RaceDay);
        EEPROM.put(365, RaceHour);
        EEPROM.put(370, RaceMinute);
        EEPROM.put(375, RaceSecond);
        EEPROM.put(400, RaceMonth);
        delay(50);
        // here is where we add a new record, write header data, called only ones when race starts
        AddNewRecordset();

        // back out time up to this Point so driver sees starting time of zero
        GPSLapTimer = 0;
        AverageCount = 0;
        AverageAmps = 0.0;
        AverageVolts = 0.0;
        Distance = 0.0f;
        LapCount = 0;
        Energy = 0.0f;
        LapEnergy = 0.0;
        StartLapEnergy = 0.0;
        DriverTimer = 0;
        CarRaceTimer = 0;
        LapTimer = 0;

        // timer for when to record start GPS location
        StartGPSDelayTimer = 0;
        digitalWrite(GPSLED_PIN, HIGH);
        LapLEDTimer = 0;
        RedrawHeader = true;

      } else if ((DriverTime[Driver] >= 900000l) && (KeySwitchedOn)) {  // 900000l
        // reset the key switch flag so we can detect the next pit change
        // must be driver change
        ChangeDriver();
        ShowNewDriverScreen();
      }
    }
  }
}

void ShowNewDriverScreen() {

  // show a cute splash screen
  //Draw new driver welcome screen
  Display.fillScreen(back_color);

  Display.setCursor(30, 30);
  Display.setFont(FONT_24BI);
  Display.setTextColor(fore_color, back_color);
  Display.print(F("Driver: "));
  if (DriverID[Driver] < ((sizeof(DriverNames) / sizeof(DriverNames[0])))) {
    Display.print(DriverNames[DriverID[Driver]]);
  }

  Display.setFont(FONT_100BINO);
  Display.setTextColor(fore_color, back_color);
  Display.setCursor(140, 100);
  Display.print(Driver + 1);

  // start the screen timer
  RedrawHeader = true;
  DriverChangeScreen = 0;
  DrawGraph = true;
}
void ResetRaceDate() {

  Record = 0;
  RaceStatus = RACE_NOTSTARTED;

  RaceDay = 0;
  RaceHour = 0;
  RaceMinute = 0;
  RaceSecond = 0;
  RaceMonth = 0;
  EEPROM.put(360, RaceDay);
  EEPROM.put(365, RaceHour);
  EEPROM.put(370, RaceMinute);
  EEPROM.put(375, RaceSecond);
  EEPROM.put(400, RaceMonth);

  delay(50);
}

void SaveStartGPS(bool Action) {

  if (Action) {
    if (GPSStatus) {
      GPSStartLat = GPSLat;
      GPSStartLon = GPSLon;
      StartGPSFound = true;
      EEPROM.put(340, GPSStartLat);
      EEPROM.put(350, GPSStartLon);
      delay(10);
    }
  } else {
    GPSStartLat = 0.0;
    GPSStartLon = 0.0;
    EEPROM.put(340, GPSStartLat);
    EEPROM.put(350, GPSStartLon);
    StartGPSFound = false;
  }
  // write time
  delay(50);
}

void AddNewRecordset() {

  Tyear = year();
  Tmonth = month();
  Tday = day();
  Thour = hour();
  Tminute = minute();
  RecordType = RT_HEADER;

  // set the next RecordsetID
  RecordSETID++;

  // we're doing 4 averages per read on analogRead
  AverageCounter = Counter * 4;

  if (SSD.addRecord()) {
    SSD.saveHeader();
  }

  RecordType = RT_DATA;
  // writing new header, begin record count to 1
  Point = 1;
}

/*---------------------------------------------------------*/
//SEND DATA
/*---------------------------------------------------------*/

/*
   PURPOSE : Sends data via transceiver to the Pit Display
    PARAMS :  -
   RETURNS : None
     NOTES : (Future) Reduce compile errors from this function
*/

void SendData() {

  Data.RPM = (uint16_t)(mRPM & 0b0000111111111111);

  Data.WARNINGS = (uint16_t)(Warnings);
  Data.TEMPF_TEMPX = ((uint16_t)MotorTemp << 8) | (((uint16_t)AuxTemp) & 0b0000000011111111);
  Data.VOLTS_LAPS = ((uint16_t)(Volts * 10.0f)) << 7 | (((uint16_t)LapCount) & 0b0000000001111111);
  Data.SPEED_EREM = ((uint16_t)(CarSpeed * 10.0f)) << 7 | (((uint16_t)ERem) & 0b0000000001111111);
  Data.DISTANCE_TREM = ((uint16_t)(Distance * 10.0f)) << 7 | (((uint16_t)TRem) & 0b0000000001111111);

  Data.AMPS_D0ID = ((uint16_t)(abs(Amps) * 10.0f)) << 5 | (((uint16_t)DriverID[0]) & 0b0000000000011111);
  if (Amps < 0) {
    Data.AMPS_D0ID = Data.AMPS_D0ID | 0b1000000000000000;
  }

  Data.ENERGY_D1ID = ((uint16_t)Energy) << 6 | (((uint16_t)DriverID[1]) & 0b0000000000111111);
  Data.LAP2AMPS_D2ID = ((uint16_t)(TargetAmps * 10.0f)) << 7 | (((uint16_t)DriverID[2]) & 0b0000000001111111);

  if (RaceStatus == RACE_NOTSTARTED) {
    Data.RACETIME = (uint16_t)(0);  // data stored in s
  } else {
    Data.RACETIME = (uint16_t)(CarRaceTimer / 1000);  // data stored in ms
  }
  Data.D0TIME = (uint16_t)(DriverTime[0] / 1000);  // data stored in ms
  Data.D1TIME = (uint16_t)(DriverTime[1] / 1000);
  Data.D2TIME = (uint16_t)(DriverTime[2] / 1000);

  // lap time (LT) data stored in seconds
  Data.LT = LapTime;

  GForce = GForce * 100.0f;

  // we can send up to 5.12 G's (we record full amount though) (9 bits)
  Data.GFORCE_DRIVERNO = (uint16_t)(GForce) << 7 | (Driver & 0b0000000000000011) << 5;

  Data.TWHR_LAPAMPS = (uint16_t)((TotalEnergy / 10) << 9) | ((uint16_t)(LapAmps * 10.0f) & 0b0000000111111111);

  Data.LAPENERGY_DTS = ((uint16_t)(LapEnergy * 10.0f) << 7) | ((uint16_t)GPSDistance);

  // ESerial.write((uint8_t*) &Data, PacketSize );
  DataPacket.sendData();

  SmartDelay(20);
}

/*---------------------------------------------------------*/
//GET PARAMETERS
/*---------------------------------------------------------*/

/*
   PURPOSE : Gets parameters from the EEPROM
    PARAMS :  -
   RETURNS : None
     NOTES :
*/

void GetParameters() {

  EEPROM.get(0, NewMCU);

  if ((digitalRead(L_PIN) == LOW) ^ (digitalRead(R_PIN) == LOW)) {

    Display.setRotation(1);
    Display.fillScreen(C_RED);
    Display.setFont(FONT_16B);
    Display.setTextColor(C_WHITE, back_color);
    Display.setCursor(10, 30);
    Display.print(F("Resetting Race"));

    while ((digitalRead(R_PIN) == LOW) || (digitalRead(L_PIN) == LOW)) {
      delay(10);
    }

    ResetRaceDate();
    SaveStartGPS(false);
  }

  // if unprogrammed or user want's to reset
  // could be due to corrupted eeprom data or added parameter and eeprom has some old data
  if ((digitalRead(L_PIN) == LOW) && (digitalRead(R_PIN) == LOW)) {

    Display.setRotation(1);
    Display.fillScreen(C_RED);
    Display.setFont(FONT_16B);
    Display.setTextColor(C_WHITE, back_color);
    Display.setCursor(10, 30);
    Display.print(F("Resetting EEPROM"));
    while ((digitalRead(R_PIN) == HIGH) && (digitalRead(L_PIN) == HIGH)) {
      delay(10);
    }
    NewMCU = 0;
    delay(100);
  }

  if (NewMCU == 0) {
    // new programmer reset the whole thing
    for (i = 0; i < 500; i++) {
      EEPROM.put(i, 0);
      delay(10);
    }

#ifdef DO_DEBUG
    Serial.println(F("Resetting EEPROM data"));
#endif

    // now set some defaults
    NewMCU = 1;
    EEPROM.put(0, NewMCU);
    MotorSprocket = 15;
    EEPROM.put(10, MotorSprocket);
    WheelSprocket = 70;
    EEPROM.put(20, WheelSprocket);
    TireID = 0;
    EEPROM.put(30, TireID);
    Invert = 0;
    EEPROM.put(40, Invert);
    Orientation = 1;
    EEPROM.put(50, Orientation);
    AutoCurrentCal = true;
    EEPROM.put(55, AutoCurrentCal);
    RadioUpdate = 1;
    EEPROM.put(60, RadioUpdate);
    TotalEnergy = 600;
    EEPROM.put(70, TotalEnergy);
    DriverID[0] = 0;
    EEPROM.put(80, DriverID[0]);
    DriverID[1] = 1;
    EEPROM.put(90, DriverID[1]);
    DriverID[2] = 2;
    EEPROM.put(100, DriverID[2]);
    VoltageSlope = 11.0;
    EEPROM.put(110, VoltageSlope);
    VoltageOffset = 0.30;
    EEPROM.put(120, VoltageOffset);
    MotorID = 0;
    EEPROM.put(130, MotorID);
    TempWarning = 140;
    EEPROM.put(140, TempWarning);
    BatWarning = 21.0;
    EEPROM.put(150, BatWarning);
    LapThreashold = 30;
    EEPROM.put(160, LapThreashold);
    Battery1 = 0;
    EEPROM.put(170, Battery1);
    Battery2 = 0;
    EEPROM.put(175, Battery2);
    MinPulses = 3.0;
    EEPROM.put(180, MinPulses);
    AddLapInPit = true;
    EEPROM.put(185, AddLapInPit);
    StartGPSDelayID = 0;
    EEPROM.put(190, StartGPSDelayID);
    AccelLPFilter = 0;
    EEPROM.put(200, AccelLPFilter);
    AccelHPFilter = 0;
    EEPROM.put(205, AccelHPFilter);
    Pickups = 20;
    EEPROM.put(210, Pickups);
    ResetAltitude = false;
    EEPROM.put(215, ResetAltitude);
    mVPerAmp = 20.0f;
    EEPROM.put(220, mVPerAmp);
    VMid = .5f;
    EEPROM.put(230, VMid);
    RestartDisplayAlways = false;
    EEPROM.put(240, RestartDisplayAlways);
    GForceRange = 1;
    EEPROM.put(250, GForceRange);
    UseGPSForSpeedDisplay = false;
    EEPROM.put(260, UseGPSForSpeedDisplay);
    GPSTolerance = 4;
    EEPROM.put(280, GPSTolerance);
    CarID = 0;
    EEPROM.put(300, CarID);
    ThermResMotor = 10000.0;
    EEPROM.put(310, ThermResMotor);
    ThermResAux = 10000.0;
    EEPROM.put(315, ThermResAux);
    DisplayID = 0;
    EEPROM.put(320, DisplayID);
    GPSStartLat = 0.0;
    EEPROM.put(340, GPSStartLat);
    GPSStartLon = 0.0;
    EEPROM.put(350, GPSStartLon);
    RaceDay = 0;
    EEPROM.put(360, RaceDay);
    RaceHour = 0;
    EEPROM.put(365, RaceHour);
    RaceMinute = 0;
    EEPROM.put(370, RaceMinute);
    RaceSecond = 0;
    EEPROM.put(375, RaceSecond);
    TriggerAmps = 20.0f;
    EEPROM.put(380, TriggerAmps);
    RaceMonth = 0;
    EEPROM.put(400, RaceMonth);
    RestoreData = 1;
    EEPROM.put(410, RestoreData);
    AccelCalX = 0;
    EEPROM.put(450, AccelCalX);
    HasASensor = false;
    EEPROM.put(455, HasASensor);
    AccelCalY = 0;
    EEPROM.put(460, AccelCalY);
    AccelCalZ = 0;
    EEPROM.put(470, AccelCalZ);
    AmbTempCF = 0.0f;
    EEPROM.put(480, AmbTempCF);

    Display.fillScreen(C_BLACK);
  }

#ifdef DO_DEBUG
  Serial.println(F("Getting EEPROM data"));
#endif

  EEPROM.get(10, MotorSprocket);
  EEPROM.get(20, WheelSprocket);
  EEPROM.get(30, TireID);
  EEPROM.get(40, Invert);
  EEPROM.get(50, Orientation);
  EEPROM.get(55, AutoCurrentCal);
  EEPROM.get(60, RadioUpdate);
  EEPROM.get(70, TotalEnergy);
  EEPROM.get(80, DriverID[0]);
  EEPROM.get(90, DriverID[1]);
  EEPROM.get(100, DriverID[2]);
  EEPROM.get(110, VoltageSlope);
  EEPROM.get(120, VoltageOffset);
  EEPROM.get(130, MotorID);
  EEPROM.get(140, TempWarning);
  EEPROM.get(150, BatWarning);
  EEPROM.get(160, LapThreashold);
  EEPROM.get(170, Battery1);
  EEPROM.get(175, Battery2);
  EEPROM.get(180, MinPulses);
  EEPROM.get(185, AddLapInPit);
  EEPROM.get(190, StartGPSDelayID);
  EEPROM.get(200, AccelLPFilter);
  EEPROM.get(205, AccelHPFilter);
  EEPROM.get(210, Pickups);
  EEPROM.get(215, ResetAltitude);
  EEPROM.get(220, mVPerAmp);
  EEPROM.get(230, VMid);
  EEPROM.get(240, RestartDisplayAlways);
  EEPROM.get(250, GForceRange);
  EEPROM.get(260, UseGPSForSpeedDisplay);
  EEPROM.get(280, GPSTolerance);
  EEPROM.get(300, CarID);
  EEPROM.get(310, ThermResMotor);
  EEPROM.get(315, ThermResAux);
  EEPROM.get(320, DisplayID);
  EEPROM.get(340, GPSStartLat);
  EEPROM.get(350, GPSStartLon);
  EEPROM.get(360, RaceDay);
  EEPROM.get(365, RaceHour);
  EEPROM.get(370, RaceMinute);
  EEPROM.get(375, RaceSecond);
  EEPROM.get(380, TriggerAmps);
  EEPROM.get(400, RaceMonth);
  EEPROM.get(410, RestoreData);
  EEPROM.get(450, AccelCalX);
  EEPROM.get(455, HasASensor);
  EEPROM.get(460, AccelCalY);
  EEPROM.get(470, AccelCalZ);
  EEPROM.get(480, AmbTempCF);

  GetGearParameters();

  // probably need to bounds check other array type stuff (tire ID for example)
  if (GForceRange < ((sizeof(AccelFSRange) / sizeof(AccelFSRange[0])))) {
  } else {
    GForceRange = 0;
  }

  // bounds check GPS start delay
  if (StartGPSDelayID < ((sizeof(GPSReadTimeText) / sizeof(GPSReadTimeText[0])))) {
    StartGPSDelay = GPSReadTime[StartGPSDelayID];
  } else {
    StartGPSDelay = 0;
  }


#ifdef DO_DEBUG

  Serial.println(F("******* EEPROM Parameters *******"));
  Serial.print(F("MCU: "));
  Serial.println(NewMCU);
  Serial.print(F("Voltage Slope: "));
  Serial.println(VoltageSlope);
  Serial.print(F("Voltage Offset: "));
  Serial.println(VoltageOffset);
  Serial.print(F("Gear Ratio: "));
  Serial.println(GearRatio);
  Serial.print(F("Motor Sprocket: "));
  Serial.println(MotorSprocket);
  Serial.print(F("Wheel Sprocket: "));
  Serial.println(WheelSprocket);
  Serial.print(F("Motor ID: "));

  if (MotorID < ((sizeof(MotorText) / sizeof(MotorText[0])))) {
    Serial.print(F(", "));
    Serial.println(MotorText[(int)MotorID]);
  } else {
    Serial.println(F("MOTOR ID OUT OF RANGE"));
  }

  Serial.print(F("Tire ID: "));
  Serial.println(TireID);
  Serial.print(F("Tire type: "));
  Serial.println(TireText[TireID]);
  Serial.print(F("Tire Rad: "));
  Serial.println(TireRadius[TireID]);
  Serial.print(F("Invert: "));
  Serial.println(Invert);
  Serial.print(F("Orientation: "));
  Serial.println(Orientation);
  Serial.print(F("Update: "));
  Serial.println(DisplayUpdateTimer);
  Serial.print(F("Total Energy: "));
  Serial.println(TotalEnergy);
  Serial.print(F("Pickups: "));
  Serial.println(Pickups);
  Serial.print(F("Min Pulses: "));
  Serial.println(MinPulses);
  Serial.print(F("VMid: "));
  Serial.println(VMid);
  Serial.print(F("mVPerAmp: "));
  Serial.println(mVPerAmp);
  Serial.print(F("Battery Warning: "));
  Serial.println(BatWarning);
  Serial.print(F("Lap Threashold: "));
  Serial.println(LapThreashold);
  Serial.print(F("Temp Warning: "));
  Serial.println(TempWarning);
  Serial.print(F("Driver 1: "));
  Serial.print(DriverID[0]);
  Serial.print(F(", "));
  Serial.println(DriverNames[DriverID[0]]);
  Serial.print(F("Driver 2: "));
  Serial.print(DriverID[1]);
  Serial.print(F(", "));
  Serial.println(DriverNames[DriverID[1]]);
  Serial.print(F("Driver 3: "));
  Serial.print(DriverID[2]);
  Serial.print(F(", "));
  Serial.println(DriverNames[DriverID[2]]);
  Serial.print(F("Car ID: "));
  Serial.println(CarID);
  Serial.print(F("Car Name: "));
  Serial.println(CarText[CarID]);
  Serial.print(F("GPSTolerance: "));
  Serial.println(GPSTolerance);
  Serial.print(F("Temp Motor Res: "));
  Serial.println(ThermResMotor);
  Serial.print(F("Temp Aux Res: "));
  Serial.println(ThermResAux);
  Serial.print(F("TriggerAmps: "));
  Serial.println(TriggerAmps);
  Serial.print(F("GPSStartLat: "));
  Serial.println(GPSStartLat, 2);
  Serial.print(F("GPSStartLon: "));
  Serial.println(GPSStartLon, 2);
  Serial.print(F("RaceDay: "));
  Serial.println(RaceDay);
  Serial.print(F("RaceHour: "));
  Serial.println(RaceHour);
  Serial.print(F("RaceMinute: "));
  Serial.println(RaceMinute);
  Serial.print(F("RaceMonth: "));
  Serial.println(RaceMonth);
  Serial.print(F("RestoreData: "));
  Serial.println(RestoreData);
  Serial.println(F("******* End EEPROM Parameters *******"));
#endif
}

/*
   PURPOSE : Read Last Line From Data File
    PARAMS :  -
   RETURNS : None
     NOTES : Used to get last Point, LapCount, and Distance after a restart.
*/

uint16_t RestoreRaceData() {

  uint32_t TotalDownTime = 0;
  uint32_t RecordsToRestore = 0;
  uint32_t CurrentTime = 0;
  uint16_t fc = 0, rw = 0;
  uint32_t DataRecord = 0;
  uint32_t TempTime = RealClockTime;
  uint32_t TempDriver0 = 0, TempDriver1 = 0, TempDriver2 = 0, ii = 0;
  uint32_t StartRecord = 0;
  // get the last known record and read the data

  Display.setTextColor(C_CYAN);
  Display.setCursor(STATUS_RESULT, 220);
  Display.print(F("Reading: "));

  // go to the last record and get the data
  // note that records are storded 1 based so record 10 records is stored and recalled as 10
  // if power was lost during a record write, we will not have a full record so be save and back up 2 records

  DataRecord = SSD.getLastRecord() - 2;

  SSD.gotoRecord(DataRecord);
  RecordType = SSD.getField(RecordType, frType);

#ifdef DO_DEBUG
  Serial.print("LastRecord ");
  Serial.println(LastRecord);
  Serial.print("RecordType ");
  Serial.println(RecordType);
#endif

  if ((RecordType == RT_HEADER) || (RecordType == NULL_RECORD)) {
    return RR_ERROR;
  }

  // we restore just this data
  // note since these vars are set, addRecord() / saveRecord()
  // will simply use these values
  RecordSETID = (uint8_t)SSD.getField(RecordSETID, frID);

  Point = (uint16_t)SSD.getField(Point, frPoint);
  LapCount = (uint8_t)SSD.getField(LapCount, frLap);
  Driver = (uint8_t)SSD.getField(Driver, frDriver);
  Energy = SSD.getField(Energy, frEnergy);
  Distance = SSD.getField(Distance, frDist);
  RealClockTime = (uint32_t)SSD.getField(RealClockTime, frRT);
  GPSLon = SSD.getField(GPSLon, frLon);
  GPSLat = SSD.getField(GPSLat, frLat);
  Altitude = SSD.getField(Altitude, frAlt);

  // get Revolutions so distance calculations will be restored
  Revolutions = (Distance * 12.0f * 5280.0f) / (TireRad * 2.0f * 3.1416f);

  // we zero out these
  Volts = 0.0;
  Amps = 0.0;
  MotorTemp = 0.0;
  AuxTemp = 0.0;
  mRPM = 0;
  CarSpeed = 0.0;

  // now rip through and get driver times
  // first get first record for the desire recordsetID
  StartRecord = SSD.getFirstRecord(RecordSETID, hrID);

  for (ii = StartRecord; ii <= SSD.getLastRecord(); ii++) {
    SSD.gotoRecord(ii);
    if (SSD.getField(RecordSETID, frID) == RecordSETID) {
      if (0 == SSD.getField(Driver, frDriver)) {
        TempDriver0++;
      }
      if (1 == SSD.getField(Driver, frDriver)) {
        TempDriver1++;
      }
      if (2 == SSD.getField(Driver, frDriver)) {
        TempDriver2++;
      }
    }
    if (SSD.getField(RecordSETID, frID) > RecordSETID) {
      // we are now past the desire recordset, bail out
      break;
    }
  }

  // we need to account for how many points per second
  // then convert to millis

  DriverTime[0] = (TempDriver0 * UPDATE_LIMIT * 1000) / 1000;
  DriverTime[1] = (TempDriver1 * UPDATE_LIMIT * 1000) / 1000;
  DriverTime[2] = (TempDriver2 * UPDATE_LIMIT * 1000) / 1000;

  // restore record to last
  SSD.gotoRecord(SSD.getLastRecord());

  // since we backed up 2 records, advance time and point
  // we will increment in loop below
  RealClockTime += 2;
  Point += 2;

#ifdef DO_DEBUG
  Serial.println(SSD.getLastRecord());
  Serial.print("RecordType ");
  Serial.println(RecordType);
  Serial.print("RecordSETID ");
  Serial.println(RecordSETID);
  Serial.print("Point ");
  Serial.println(Point);
  Serial.print("LapCount ");
  Serial.println(LapCount);
  Serial.print("Driver ");
  Serial.println(Driver);
  Serial.print("Energy ");
  Serial.println(Energy);
  Serial.print("Distance ");
  Serial.println(Distance);
  Serial.print("LastRaceTime ");
  Serial.println(RealClockTime);
  Serial.print("GPSLat ");
  Serial.println(GPSLat);
  Serial.print("GPSLon ");
  Serial.println(GPSLon);
  Serial.print("Altitude ");
  Serial.println(Altitude);
  Serial.print("hour() ");
  Serial.println(hour());
  Serial.print("minute() ");
  Serial.println(minute());
  Serial.print("second() ");
  Serial.println(second());
  Serial.print("Driver 0 records ");
  Serial.println(TempDriver0);
  Serial.print("Driver 1 records ");
  Serial.println(TempDriver1);
  Serial.print("Driver 2 records ");
  Serial.println(TempDriver2);
  Serial.print("Driver 0 Time ");
  Serial.println(DriverTime[0]);
  Serial.print("Driver 1 Time ");
  Serial.println(DriverTime[1]);
  Serial.print("Driver 2 Time ");
  Serial.println(DriverTime[2]);
#endif

  CurrentTime = (hour() * 3600) + (minute() * 60) + second();

  TotalDownTime = CurrentTime - RealClockTime;

  // reset driver timer with last > 0 driver time
  if (DriverTime[0] > 0) {
    DriverTimer = DriverTime[0] + (TotalDownTime * 1000l);
  }
  if (DriverTime[1] > 0) {
    DriverTimer = DriverTime[1] + (TotalDownTime * 1000l);
  }
  if (DriverTime[2] > 0) {
    DriverTimer = DriverTime[2] + (TotalDownTime * 1000l);
  }

#ifdef DO_DEBUG
  Serial.print("CurrentTime ");
  Serial.println(CurrentTime);
  Serial.print("RealClockTime ");
  Serial.println(RealClockTime);
  Serial.print("TotalDownTime ");
  Serial.println(TotalDownTime);
  Serial.print("Duration ");
  Serial.println(Duration);
  Serial.print("CarRaceTimer ");
  Serial.println(CarRaceTimer);
  Serial.print("CurrentTime          [s] ");
  Serial.println(CurrentTime);
  Serial.print("Last known race Time [s] ");
  Serial.println(RealClockTime);
  Serial.print("Total Down Time      [s] ");
  Serial.println(TotalDownTime);
  Serial.print("CarRaceTimer        [ms] ");
  Serial.println(CarRaceTimer);
#endif

  if ((TotalDownTime > (RACE_TIME_SECONDS + RACE_EXTENSION)) || (TotalDownTime <= 0)) {
    return RR_ERROR;
  }

  // we may be saving data more that 1 per second
  // so we need to get down time and multiply by points/second
  RecordsToRestore = TotalDownTime * (1000 / UPDATE_LIMIT);

  Display.fillRoundRect(STATUS_RESULT, 200, 160, 18, 2, C_DKGREEN);

  TempTime = RealClockTime;
  /////////////////////////////////////////////////////

  for (i = 0; i <= RecordsToRestore; i++) {

    fc++;
    rw = ((float)(i * 160.0) / RecordsToRestore) + 2;
    Display.fillRoundRect(STATUS_RESULT, 200, rw, 18, 2, C_GREEN);

#ifdef DO_DEBUG
    Serial.print("Saving to SSD, Point: ");
    Serial.println(Point);
#endif

    Point++;
    // clock must increment by UPDATE_LIMIT
    RealClockTime = TempTime + (int)((i * UPDATE_LIMIT) / 1000);

    RestoreType = STATUS_RESTORE;

    if (Point > ((float)((1000.0 / UPDATE_LIMIT)) * (RACE_TIME_SECONDS + RACE_EXTENSION))) {
      break;
    }
    // now we are on the first writable record
    if (SSD.addRecord()) {
      SSD.saveRecord();
    }
  }

  if (digitalRead(KEY_PIN) == false) {
    // key off, probably still in pit
    RestoreType = STATUS_PITSTOP;
  } else {
    // key on probably restart duing run time
    RestoreType = STATUS_OK;
  }

  Duration = ((hour() * 3600) + (minute() * 60) + second()) - ((RaceHour * 3600) + (RaceMinute * 60) + RaceSecond);
  CarRaceTimer = (Duration * 1000l);  // + (TotalDownTime * 1000l);

  Display.fillRoundRect(STATUS_RESULT, 200, 160, 18, 2, C_GREEN);
  delay(100);
  // return the number of records resored
  return TotalDownTime;
}

/*---------------------------------------------------------*/
//DISPLAY DATA FUNCTIONS
/*---------------------------------------------------------*/

/*
   PURPOSE : Draws Speed View
    PARAMS :  -
   RETURNS : None
     NOTES : Draws Speed view in the Display Data Function when called in the switch
*/

void RestartDisplay() {
  if (!RestartDisplayAlways) {
    return;
  }
  Display.begin();

  SetScreenParameters();
}

void SpeedView() {


  if (RedrawHeader) {
    Display.fillScreen(back_color);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("SPEED"));
    RedrawHeader = false;
  }

  Display.setFont(FONT_16B);
  Display.setCursor(DATA_X, DATA_Y);
  Display.setFont(FONT_100BINO);
  ffSpeed.setTextColor(fore_color, back_color);

  if (UseGPSForSpeedDisplay) {
    ffSpeed.print(GPSSpeed, 1);
  } else {
    ffSpeed.print(CarSpeed, 1);
  }

  //Right Side Info

  //Print Labels
  Display.drawRect(13, 190, 294, 37, fore_color);
  Display.setFont(FONT_16B);
  Display.setCursor(20, 202);
  Display.print(F("MAX"));

  Display.setCursor(160, 202);
  Display.print(F("WRPM"));

  //Print Data
  Display.setCursor(80, 202);
  ffMaxSpeed.setTextColor(fore_color, back_color);
  ffMaxSpeed.print(CarMaxSpeed, 1);

  Display.setCursor(240, 202);
  ffWRPM.setTextColor(fore_color, back_color);
  ffWRPM.print(WRPM);
}

/*
   PURPOSE : Draws Amps View
    PARAMS :  -
   RETURNS : None
     NOTES : Draws Amps view in the Display Data Function when called in the switch
*/

void AmpsView() {
  if (RedrawHeader) {
    Display.fillScreen(back_color);
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("AMPS"));
    RedrawHeader = false;
  }

  //Print Labels
  Display.drawRect(13, 190, 294, 37, fore_color);
  Display.setFont(FONT_16B);
  Display.setCursor(18, 202);
  Display.print(F("LAP"));
  Display.setCursor(166, 202);
  Display.print(F("MAX"));


  ffAmps.setTextColor(fore_color, back_color);
  Display.setFont(FONT_100BINO);
  Display.setCursor(DATA_X, DATA_Y);

  if (Amps > 99) {
    ffAmps.print(Amps, 0);
  } else if (Amps >= 1.0f) {
    ffAmps.print(Amps, 1);
  } else if (Amps >= -1.0) {
    ffAmps.print(Amps, 2);
  } else {
    ffAmps.print(Amps, 1);
  }

  //Bottom Info

  //Print Data
  Display.setFont(FONT_16B);
  Display.setCursor(80, 200);
  ffLapAmps.setTextColor(fore_color, back_color);
  ffLapAmps.print(LapAmps, 1);

  Display.setCursor(230, 200);
  ffMaxAmps.setTextColor(fore_color, back_color);
  ffMaxAmps.print(MaxAmps, 1);
}

/*
   PURPOSE : Draws Volts View
    PARAMS :  -
   RETURNS : None
     NOTES : Draws Volts view in the Display Data Function when called in the switch
*/

void VoltsView() {
  if (RedrawHeader) {
    Display.fillScreen(back_color);
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("VOLTS"));

    RedrawHeader = false;
  }

  Display.setCursor(DATA_X, DATA_Y);
  Display.setFont(FONT_100BINO);
  ffVolts.setTextColor(fore_color, back_color);
  ffVolts.print(Volts, 1);

  //Bottom Info

  //Print Labels
  Display.drawRect(13, 190, 294, 37, fore_color);
  Display.setFont(FONT_16B);
  Display.setCursor(18, 202);
  Display.print(F("LAP"));
  Display.setCursor(165, 202);
  Display.print(F("MIN"));

  //Print Data
  Display.setFont(FONT_16B);
  Display.setCursor(95, 200);
  ffLapVolts.setTextColor(fore_color, back_color);
  ffLapVolts.print(LapVolts, 1);

  Display.setCursor(245, 200);
  ffMinVolts.setTextColor(fore_color, back_color);

  ffMinVolts.print(MinVolts, 1);
}

void EnergyView() {

  if (RedrawHeader) {
    Display.fillScreen(back_color);
    DrawGraph = true;
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("TREND"));

    EnergyG.setYAxis(0, (((int)((TotalEnergy + 99.0f) / 100)) * 100), 100);

    RedrawHeader = false;
  }

  if (DrawGraph) {
    // draw the graph
    DrawGraph = false;
    EnergyG.drawGraph();
    // force data to be drawn
    // once a graph is drawn, the counter in
    // compute will take care of this
    GraphDrawTimer = 60000;
    // plot base line curve
    // if plotted data is above curve we dont finish
    // if plotted data is below we undertuned
    for (i = 0; i < 90; i++) {
      EnergyG.setX(i);
      EnergyG.plot(bEnergyID, BLEnergy[i] * (TotalEnergy / 600.0f));  // base line data for 1 battery
    }
  }

  // if a min has passed plot all data
  if (GraphDrawTimer >= 60000) {
    EnergyG.resetStart(EnergyID);
    GraphDrawTimer = 0;
    // only plot if we have enough points
    if (Point > 1) {
      // plot them...
      for (i = 0; i < 90; i++) {
        if (EnergyPoints[i] > 0) {
          EnergyG.setX(i);
          EnergyG.plot(EnergyID, EnergyPoints[i]);
        }
      }
    }
  }
}

void GForceView() {

  if (RedrawHeader) {
    Display.fillScreen(back_color);
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("G-Force"));
    GForceG.drawGraph();
    RedrawHeader = false;
  }

  GForceXPoint++;
  GForceG.setX(GForceXPoint);
  GForceG.plot(GForceXID, GForceX);
  GForceG.plot(GForceYID, GForceY);
  GForceG.plot(GForceZID, GForceZ);
  GForceG.plot(GForceID, GForce);
}


/*
   PURPOSE : Draws Temp View
    PARAMS :  -
   RETURNS : None
     NOTES : Draws Temp view in the Display Data Function when called in the switch
*/

void TempView() {
  if (RedrawHeader) {
    Display.fillScreen(back_color);
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(fore_color);
    Display.print(F("TEMP"));
    RedrawHeader = false;

    MotorTempG.refresh();
    AuxTempG.refresh();
    AmbTempG.refresh();
  }
  MotorTempG.draw(MotorTemp);
  AuxTempG.draw(AuxTemp);
  AmbTempG.draw(AmbTemp);

  //Bottom Info

  //Print Labels
  Display.drawRect(13, 190, 294, 37, fore_color);

  Display.setFont(FONT_16B);

  Display.setCursor(40, 200);
  if (hour() > 12) {
    sprintf(str, "%d:%02d:%02d", hour() - 12, minute(), second());
  } else {
    sprintf(str, "%d:%02d:%02d", hour(), minute(), second());
  }

  ffTime.setTextColor(fore_color, back_color);
  ffTime.print(str);

  Display.setCursor(180, 200);
  sprintf(str, "%d/%d/%d", month(), day(), year());
  ffDate.setTextColor(fore_color, back_color);
  ffDate.print(str);
}

/*
   PURPOSE : Draws Time View
    PARAMS :  -
   RETURNS : None
     NOTES : Draws Time view in the Display Data Function when called in the switch
*/

void TimeView() {

  if (RedrawHeader) {
    Display.fillScreen(back_color);
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);

    Display.print(F("TIME"));
    RedrawHeader = false;
  }
  Display.setFont(FONT_24BI);

  Display.setCursor(20, 55);
  Display.print(F("Distance [ft]:"));
  Display.setFont(FONT_24BI);
  Display.setCursor(250, 55);

  ffDriver.setTextColor(fore_color, back_color);
  // reminder, distance is in meters, so convert
  if (GPSDistance > 300) {
    strcpy(str, ">999");
  } else {
    sprintf(str, "%d", (int)(GPSDistance * METERS_TO_FEET));
  }
  ffDriver.print(str);

  if (RaceStatus == RACE_INPROGRESS) {
    m = LapTime / 60;
    s = LapTime % 60;
    sprintf(str, "%01d:%02d", m, s);
  } else if (RaceStatus == RACE_FINISHED) {
    strcpy(str, "DONE");
  } else if (RaceStatus == RACE_NOTSTARTED) {
    strcpy(str, "0:00");
  }

  TimeSplit = LapTime - LastLapTime;

  if (RaceStatus == RACE_INPROGRESS) {
    if (TimeSplit > 3) {
      ffThisLap.setTextColor(C_RED, back_color);

    } else if (TimeSplit < 3) {
      ffThisLap.setTextColor(C_GREEN, back_color);

    } else {
      ffThisLap.setTextColor(C_YELLOW, back_color);
    }
  }

  Display.setCursor(DATA_X, DATA_Y + 50);
  Display.setFont(FONT_100BINO);
  ffThisLap.print(str);


  /*
  Display.setFont(FONT_14);
  Display.setTextColor(fore_color);

  //Left Side
  Display.drawRect(13, 50, 65, 50, fore_color);
  Display.drawRect(83, 50, 70, 50, fore_color);
  Display.drawRect(13, 105, 140, 105, fore_color);

  Display.setCursor(18, 55);
  Display.print(F("LAP"));

  Display.setCursor(93, 55);
  Display.print(F("Dist."));

  Display.setCursor(18, 110);
  Display.print(F("RACE TIME"));

  Display.setCursor(18, 161);
  Display.print(F("SEAT TIME"));

  //Right Side
  Display.setTextColor(fore_color);
  Display.drawRect(160, 50, 159, 160, fore_color);

  Display.setCursor(165, 55);
  Display.print(F("LAP TIME"));

  Display.setCursor(165, 140);
  Display.print(F("LAP SPLIT"));

  // Show Data

  Display.setFont(FONT_24BI);

  //Lap Count
  Display.setCursor(18, 70);
  ffLaps.setTextColor(fore_color, back_color);
  ffLaps.print(LapCount);

  //car distance from start coordinates
  Display.setCursor(93, 70);
  ffDriver.setTextColor(fore_color, back_color);
  // reminder, distance is in meters, so convert
  if ((GPSDistance * METERS_TO_FEET) < 99.0) {
    ffDriver.print(GPSDistance * METERS_TO_FEET, 0);
  } else {
    ffDriver.print(0);
  }

  //driver time
  Display.setCursor(18, 180);
  ffDriverTime.setTextColor(fore_color, back_color);
  if (RaceStatus == RACE_INPROGRESS) {
    m = (int)(DriverTime[Driver] / 1000) / 60;
    s = (int)(DriverTime[Driver] / 1000) % 60;
    sprintf(str, "%02d:%02d", m, s);
  } else {
    strcpy(str, "00:00");
  }
  ffDriverTime.print(str);

  //Race Time
  if (RaceStatus == RACE_INPROGRESS) {
    m = (int)(CarRaceTimer / 1000) / 60;
    s = (int)(CarRaceTimer / 1000) % 60;
    sprintf(str, "%02d:%02d", m, s);
  }
  Display.setCursor(18, 128);
  ffCarRaceTimer.setTextColor(fore_color, back_color);
  ffCarRaceTimer.print(str);

  Display.setTextColor(C_PINK);

  // Lap time
  if (RaceStatus == RACE_NOTSTARTED) {
    strcpy(str, "00:00");
  }
  if (RaceStatus == RACE_INPROGRESS) {
    m = LapTime / 60;
    s = LapTime % 60;
    sprintf(str, "%02d:%02d", m, s);
  } else if (RaceStatus == RACE_FINISHED) {
    strcpy(str, "DONE");
  }

  Display.setCursor(165, 80);
  ffThisLap.setTextColor(fore_color, back_color);
  ffThisLap.print(str);

  //Lap Split

  TimeSplit = LapTime - LastLapTime;
  if (LapCount > 0) {
    if (RaceStatus == RACE_NOTSTARTED) {
      strcpy(str, " ");
    }
    if (RaceStatus == RACE_INPROGRESS) {
      if (TimeSplit > 0) {
        m = (TimeSplit / 1000) / 60;
        s = (TimeSplit / 1000) % 60;
        sprintf(str, "+%1d:%02d", m, s);
        ffSplitLap.setTextColor(C_RED, back_color);

      } else if (TimeSplit < 0) {
        ffLaps.setTextColor(C_GREEN, back_color);
        TimeSplit = abs(TimeSplit);
        m = (TimeSplit / 1000) / 60;
        s = (TimeSplit / 1000) % 60;
        sprintf(str, "-%1d:%02d", m, s);
        ffSplitLap.setTextColor(C_GREEN, back_color);
      } else {
        ffSplitLap.setTextColor(C_YELLOW, back_color);
        strcpy(str, "0:00");
      }
    } else if (RaceStatus == RACE_FINISHED) {
      strcpy(str, " ");
      ffSplitLap.setTextColor(C_GREEN, back_color);
    }
    Display.setCursor(165, 165);
    ffSplitLap.print(str);
  }

  */
}

/*
   PURPOSE : Draws Usage View
    PARAMS :  -
   RETURNS : None
     NOTES : Draws Usage view in the Display Data Function when called in the switch
*/

void UsageView() {

  if (RedrawHeader) {
    Display.fillScreen(back_color);
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("USAGE"));
    RedrawHeader = false;
  }

  //Draw Time
  Display.setTextColor(fore_color);

  Display.fillRect(13, 72, 30, 156 - (156 * (TRem) / 100), C_BLACK);
  Display.fillRect(13, 230 - (157 * (TRem) / 100), 30, 157 * (TRem / 100), C_TEAL);

  //Time Levels
  Display.drawFastHLine(13, 111, 5, fore_color);  // 75%
  Display.drawFastHLine(13, 150, 5, fore_color);  // 50%
  Display.drawFastHLine(13, 189, 5, fore_color);  // 25%

  // draw the Energy and time remaining, use red if it gets below time by 3%
  if ((TRem - ERem) > 3) {
    Display.fillRect(53, 72, 30, 156 - (156 * (ERem) / 100), C_BLACK);
    Display.fillRect(53, 230 - (157 * (ERem) / 100), 30, 157 * (ERem / 100), C_RED);
  } else {
    Display.fillRect(53, 72, 30, 156 - (156 * (ERem) / 100), C_BLACK);
    Display.fillRect(53, 230 - (157 * (ERem) / 100), 30, 157 * (ERem / 100), C_GREEN);
  }

  //Energy Levels
  Display.drawFastHLine(53, 111, 5, fore_color);  // 75%
  Display.drawFastHLine(53, 150, 5, fore_color);  // 50%
  Display.drawFastHLine(53, 189, 5, fore_color);  // 25%

  //draw border around graphs
  Display.drawRect(12, 71, 32, 160, fore_color);
  Display.drawRect(52, 71, 32, 160, fore_color);

  Display.setFont(FONT_14);
  Display.setTextColor(fore_color, back_color);

  Display.setCursor(13, 50);
  Display.print(F("T%"));
  Display.setCursor(53, 50);
  Display.print(F("E%"));

  Display.setFont(FONT_14);
  Display.setCursor(95, 67);
  Display.print(F("100"));
  Display.setCursor(95, 106);
  Display.print(F("75"));
  Display.setCursor(95, 145);
  Display.print(F("50"));
  Display.setCursor(95, 184);
  Display.print(F("25"));
  Display.setCursor(95, 218);
  Display.print(F("0"));

  Display.setFont(FONT_14);
  Display.setCursor(140, 50);
  Display.print(F("CONSUMPTION"));

  //Amps
  Display.setFont(FONT_14);
  Display.setTextColor(C_WHITE);

  Display.fillRect(140, 72, 85, 49, C_DKGREY);
  Display.drawRect(140, 72, 179, 49, fore_color);
  Display.drawFastVLine(225, 72, 49, fore_color);
  Display.setCursor(150, 92);
  Display.print(F("AMPS/L"));

  Display.fillRect(140, 127, 85, 49, C_DKGREY);
  Display.drawRect(140, 127, 179, 49, fore_color);
  Display.drawFastVLine(225, 127, 49, fore_color);
  Display.setCursor(148, 147);
  Display.print(F("ENER/L"));

  Display.fillRect(140, 182, 85, 49, C_DKGREY);
  Display.drawRect(140, 182, 179, 49, fore_color);
  Display.drawFastVLine(225, 182, 49, fore_color);
  Display.setCursor(145, 202);
  Display.print(F("ENERGY"));

  Display.setFont(FONT_24BI);

  Display.setCursor(230, 85);
  ffLapAmps.setTextColor(fore_color, back_color);
  ffLapAmps.print(LapAmps, 1);

  Display.setCursor(230, 140);
  ffLapEnergy.setTextColor(fore_color, back_color);
  ffLapEnergy.print(LapEnergy, 1);

  Display.setCursor(230, 195);
  ffEnergy.setTextColor(fore_color, back_color);
  ffEnergy.print(Energy, 0);
}

/*
   PURPOSE : Generates warnings based on car data
    PARAMS :  -
   RETURNS : None
     NOTES : Warnings are displayed in the form of an icon
*/

void DrawWarnings() {

  // force fails to test icons
  // Warnings = 255;

  // show racing status
  if (Warnings & RACE_START) {
    drawBitmap(100, 3, start_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(100, 3, start_icon, 32, 32, banner_back);
  }

  // SSD chip--this is bad...
  if (Warnings & SD_WARNING) {
    drawBitmap(130, 3, ssd_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(130, 3, ssd_icon, 32, 32, banner_back);
  }

  // battery
  if (Warnings & BAT_WARNING) {
    drawBitmap(160, 3, battery_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(160, 3, battery_icon, 32, 32, banner_back);
  }

  // amps
  if (Warnings & AMP_WARNING) {
    // over 70 amps
    drawBitmap(190, 3, amps_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(190, 3, amps_icon, 32, 32, banner_back);
  }

  // GPS
  if (Warnings & GPS_WARNING) {
    drawBitmap(220, 3, gps_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(220, 3, gps_icon, 32, 32, banner_back);
  }

  // temp
  if (Warnings & TEMP_WARNING) {
    drawBitmap(250, 3, temp_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(250, 3, temp_icon, 32, 32, banner_back);
  }

  // g-force
  if (Warnings & GFORCE_WARNING) {
    drawBitmap(280, 3, gforce_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(280, 3, gforce_icon, 32, 32, banner_back);
  }
}

/*---------------------------------------------------------*/
//SECONDARY FUNCTIONS
/*---------------------------------------------------------*/

/*
   PURPOSE : Updates data once lap is detected
    PARAMS :  -
   RETURNS : None
     NOTES :
*/

void CheckIfLap() {

  if ((StartGPSFound) && (GPSDistance <= GPSTolerance) && (RaceStatus == RACE_INPROGRESS) && (GPSLapTimer >= (1000l * LapThreashold))) {

    // we just tiggered get averages
    LapCount++;

    LapAmps = AverageAmps / AverageCount;
    LapVolts = AverageVolts / AverageCount;

    LapEnergy = Energy - StartLapEnergy;

    // to get the target amps get the average of laps 2 and 3 laps
    // target amps are the "initial" amp draw
    // batteries can support 19.5 amps for 90 min in a test environment
    // flat track is about 18.5, hilly around 16
    // this only works if GPS works and finds lap

    if ((LapCount > 1) && (LapCount < 4)) {
      // using laps 2 and 3
      TargetAmps += LapAmps;
    }

    if (LapCount == 5) {
      TargetAmps = TargetAmps / 2.0;
    }

    LastLapTime = LapTime;
    LapTime = LapTimer / 1000;

    if (ResetAltitude) {
      AltitudeOffset = StartAltitude - Altitude;
    }

    AverageCount = 0;
    AverageAmps = 0.0;
    AverageVolts = 0.0;
    LapTimer = 0;
    GPSLapTimer = 0;
    StartLapEnergy = Energy;

    LapLEDTimer = 0;
    digitalWrite(GPSLED_PIN, HIGH);
  }
}

/*
   PURPOSE : Debug
    PARAMS :  -
   RETURNS : None
     NOTES : Used to show more detailed, raw data when called
*/

void Debug() {

#ifdef DO_DEBUG

  Serial.println(F("******* Start Debug *******"));
  Serial.print(F("Code version : "));
  Serial.println(CODE_VERSION);
  Serial.print(F("Race status : "));
  //Race Time
  if (RaceStatus == RACE_NOTSTARTED) {
    Serial.println("RACE_NOTSTARTED");
  }
  if (RaceStatus == RACE_INPROGRESS) {
    Serial.println("RACE_INPROGRESS");
  } else if (RaceStatus == RACE_FINISHED) {
    Serial.println("RACE_FINISHED");
  }

  Serial.print(F("Warnings : "));
  Serial.print(Warnings, BIN);
  Serial.print(F(", "));
  Serial.print(F("DisplayID : "));
  Serial.println(DisplayID);
  Serial.print(F("SD: "));
  Serial.print(YesNoText[SSDStatus]);
  Serial.print(F(", used : "));
  Serial.print(SSD.getUsedSpace() / 1000);
  Serial.print(F("kb, total: "));
  Serial.print(SSD.getTotalSpace() / 1000);
  Serial.println(F("kb"));

  if (hour() > 12) {
    sprintf(str, "%d:%02d:%02d, %d/%02d/%02d", hour() - 12, minute(), second(), month(), day(), year());
  } else {
    sprintf(str, "%d:%02d:%02d, %d/%02d/%02d", hour(), minute(), second(), month(), day(), year());
  }

  Serial.print(F("RTC Time: "));
  Serial.print(str);
  BuildDateStringMS(millis());
  Serial.print(F(", Datalogger Time: "));
  Serial.print(str);
  BuildDateStringMS(CarRaceTimer);
  Serial.print(F(", CarRaceTimer: "));
  Serial.println(str);
  Serial.print(F("Data point: "));
  Serial.print(Point);
  Serial.print(F(", Averages: "));
  Serial.println(Counter);
  Serial.print(F("Volts: "));
  Serial.print(Volts, 2);
  Serial.print(F(", min: "));
  Serial.print(MinVolts, 2);
  Serial.print(F(", pin: "));
  Serial.print(vVolts, 3);
  Serial.print(F(", Amps: "));
  Serial.print(Amps, 2);
  Serial.print(F(", max: "));
  Serial.print(MaxAmps, 2);
  Serial.print(F(", pin: "));
  Serial.print(aVolts, 3);
  Serial.print(F(", MotorTemp: "));
  Serial.print(MotorTemp, 2);
  Serial.print(F(", AuxTemp: "));
  Serial.print(AuxTemp, 2);
  Serial.print(F("Power: "));
  Serial.print(Power, 2);
  Serial.print(F(", Energy: "));
  Serial.println(Energy, 2);
  Serial.print(F("WRPM: "));
  Serial.print(WRPM);
  Serial.print(F(", MRPM: "));
  Serial.print(mRPM);
  Serial.print(F(", pulses: "));
  Serial.print(PulseCount);
  Serial.print(F(", Car Speed: "));
  Serial.print(CarSpeed, 2);
  Serial.print(F("Revolutions: "));
  Serial.print(Revolutions);
  Serial.print(F(", Distance: "));
  Serial.println(Distance, 4);
  Serial.print(F("TRem: "));
  Serial.print(TRem);
  Serial.print(F(", ERem: "));
  Serial.println(ERem);
  Serial.print(F("Lap: "));
  Serial.print(LapCount);
  Serial.print(F(", Amps: "));
  Serial.print(LapAmps);
  Serial.print(F(", Speed: "));
  Serial.print(LapSpeed);
  Serial.print(F(", Energy: "));
  Serial.print(LapEnergy);
  Serial.print(F(", Count: "));
  Serial.print(LapCount);
  Serial.print(F(", time: "));
  Serial.println(LapTime);
  Serial.print(F("GPSStartLat: "));
  Serial.print(GPSStartLat, 6);
  Serial.print(F(", GPSStartLon: "));
  Serial.println(GPSStartLon, 6);
  Serial.print(F("GPSLat: "));
  Serial.print(GPSLat, 6);
  Serial.print(F(", GPSLon: "));
  Serial.print(GPSLon, 6);
  Serial.print(F(", GPSDistance: "));
  Serial.print(GPSDistance);

  Serial.print(F("Drivers: (current): "));
  Serial.println(DriverNames[DriverID[Driver]]);

  Serial.print(F("Names: \t\t"));
  Serial.print(DriverNames[DriverID[0]]);
  Serial.print(F("\t\t"));
  Serial.print(DriverNames[DriverID[1]]);
  Serial.print(F("\t\t"));
  Serial.println(DriverNames[DriverID[2]]);

  Serial.print(F("ID: \t\t"));
  Serial.print(DriverID[0]);
  Serial.print(F("\t\t"));
  Serial.print(DriverID[1]);
  Serial.print(F("\t\t"));
  Serial.println(DriverID[2]);

  Serial.print(F("Time: \t\t"));
  BuildDateStringS(DriverTime[0] / 1000);
  Serial.print(str);
  Serial.print(F("\t"));
  BuildDateStringS(DriverTime[1] / 1000);
  Serial.print(str);
  Serial.print(F("\t"));
  BuildDateStringS(DriverTime[2] / 1000);
  Serial.println(str);


  Serial.println(F("******* End Debug *******"));

#endif
}

/*
   PURPOSE : restore euint8_t defaults
    PARAMS :  -
   RETURNS : None
     NOTES :
*/

bool RestoreEBYTEDefaults() {

  // if this get's called, EBYTE fails to connect

#ifdef DO_DEBUG
  Serial.println("resetting the EBYTE");
#endif
  Radio.SetAddressH(0);
  Radio.SetAddressL(0);
  Radio.SetSpeed(0b00011000);
  Radio.SetChannel(5);
  Radio.SetAirDataRate(0b100);
  Radio.SetOptions(0b01000100);
  Radio.SaveParameters(PERMANENT);
#ifdef DO_DEBUG
  Serial.println("TRANSCEIVER RESET");
#endif

  RadioStatus = Radio.init();

#ifdef DO_DEBUG
  Serial.println(F("******* EBYTE Parameters *******"));
  Radio.PrintParameters();
  Serial.println(F("******* End EBYTE Parameters *******"));
#endif
  return RadioStatus;
}

/*
   PURPOSE : Configure buttons for input
    PARAMS :  -
   RETURNS : None
     NOTES :
*/

void ConfigureButtons() {

  if (Orientation == 0) {
    LPin = R_PIN;
    RPin = L_PIN;
  } else {
    LPin = L_PIN;
    RPin = R_PIN;
  }
}


/*
   PURPOSE : Sets gear ratio
    PARAMS :  -
   RETURNS : None
     NOTES :
*/

void GetGearParameters() {
  GearRatio = (float)WheelSprocket / (float)MotorSprocket;
  TireRad = TireRadius[TireID];
}

/*
   PURPOSE : Smart Delay function
    PARAMS : unsigned long msDelay - in form (9600)
   RETURNS : None
     NOTES : Can run code in the for loop in the future.
*/

void SmartDelay(unsigned long msDelay) {

  amt = millis();

  while ((millis() - amt) < msDelay) {

    WatchDogTimer(RESET_WDT);
  }
}

/*
   PURPOSE : increment next driver
    PARAMS :  -
   RETURNS : None
     NOTES :
*/

void ChangeDriver() {

  // manual set, show fancy screen
  // after delay reset all counters

  // GPUSA may put a sensor in pit to add a lap (most tracks pit is parallel with start line
  // this credits teams in pit with a lap since car is not going throuh start line)
  if (AddLapInPit) {
    LapCount++;
  }

  Driver++;  //Increment driver
  // no going around the corner, last driver stays in the car

  if (Driver > 2) {
    Driver = 2;
  }

  // need to start drivertime based on possible existing time
  // this can happen if driver change happens but driver still the same
  DriverTimer = DriverTime[Driver];

  LapLEDTimer = 0;
  digitalWrite(GPSLED_PIN, HIGH);
}


/*
   PURPOSE : Button press reader
    PARAMS :  -
   RETURNS : None
     NOTES :
*/
void ButtonPress() {

  val = NO_PRESS;

  //main menu trigger
  if (digitalRead(RPin) == LOW) {


    val = Debounce(RPin, ButtonDebounce);

    if (val == LONG_PRESS) {

      if (digitalRead(LPin) == LOW) {
        ChangeDriver();
        ShowNewDriverScreen();

      } else {

        ProcessMainMenu();
        RedrawHeader = true;
      }
    } else if (val == SHORT_PRESS) {

      DrawGraph = true;
      DisplayID++;
      if (DisplayID > 7) {
        DisplayID = 0;
      }
    }
    DrawGraph = true;
  }

  else if (digitalRead(LPin) == LOW) {
    val = Debounce(LPin, ButtonDebounce);

    if (val == LONG_PRESS) {
      if (digitalRead(RPin) == LOW) {
        ChangeDriver();
        ShowNewDriverScreen();
      } else {

        ProcessMainMenu();
        RedrawHeader = true;
      }
    } else if (val == SHORT_PRESS) {

      if (DisplayID == 0) {
        DrawGraph = true;
        DisplayID = 8;
      }
      DisplayID--;
    }

    EEPROM.put(320, DisplayID);
    delay(50);
    DrawGraph = true;
  }
}


/*
   PURPOSE : Debounce function
    PARAMS : int pin -
             unsigned long & dtime -
   RETURNS : unsigned -
     NOTES : Newer deRPMDebouncer function
*/


unsigned int Debounce(int pin, unsigned long &dtime) {

  if (digitalRead(pin) == LOW) {

    if ((millis() - dtime) > DEBOUNCE) {

      // button now debounced, long press or short one
      dtime = millis();
      while (digitalRead(pin) == LOW) {

        WatchDogTimer(RESET_WDT);

        if ((millis() - dtime) > LONG_PRESS) {
          dtime = millis();
          return LONG_PRESS;
        }
      }

      dtime = millis();

      return SHORT_PRESS;
    }
  }
  return NO_PRESS;
}

/*
   PURPOSE : Reads GPS Serial Buffer
    PARAMS :  -
   RETURNS : None
     NOTES :
*/

void GPSRead() {
  // note the runaway check below eliminates the need
  // to include the watch dog timer
  // in loop WDT is enabled
  // in menu code WDT is disabled
  // hence let's not reset if disabled--not sure what will happen
  GPSMaxReadTimer = 0;
  while (GPSSerial.available()) {
    GPSStatus = true;
    //c = GPSSerial.read();
    GPS.encode(GPSSerial.read());
    if (GPSMaxReadTimer > 200) {
// prevent infinite loop
#ifdef DO_DEBUG
      Serial.println("GPS Readtime exceeded");
#endif
      break;
    }
  }
}

/*
   PURPOSE : General icon drawing function
    PARAMS : int16_t x - x-value
             int16_t y - y-value
             const uint8_t *bitmap - icon
             int16_t w - width
             int16_t h - height
             uint16_t color - C_COLOR
   RETURNS : None
     NOTES : Icons must be bitmap images and converted to uint8_t form using image2.cpp, a 3rd party program; must be stored as extern uint16_t variables[]
*/

void drawBitmap(int16_t x, int16_t y, const unsigned char *bitmap, int16_t w, int16_t h, uint16_t color) {

  ByteWidth = (w + 7) / 8;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (i & 7) sByte <<= 1;
      else sByte = pgm_read_byte(bitmap + j * ByteWidth + i / 8);
      if (sByte & 0x80) Display.drawPixel(x + i, y + j, color);
    }
  }
}

/*
   PURPOSE : Creates settings menu
    PARAMS :  -
   RETURNS : None
     NOTES :
*/
void CreateUserInterface() {

  // create menus
  TopMainMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, 35, 5, "Setup Menu",
                   FONT_16B, FONT_16B);

  MainMenuOption1 = TopMainMenu.add565("Race", race_icon565, 32, 32);
  MainMenuOption2 = TopMainMenu.add565("Telemetry", car_icon565, 32, 32);
  MainMenuOption3 = TopMainMenu.add565("Wireless", transceiver_icon565, 32, 32);
  MainMenuOption4 = TopMainMenu.add565("Sensors", calibrate_icon565, 32, 32);
  MainMenuOption7 = TopMainMenu.add565("Accelerometer", GForce_icon565, 32, 32);
  MainMenuOption6 = TopMainMenu.add565("Data Storage", SSD_icon565, 32, 32);
  MainMenuOption5 = TopMainMenu.add565("Clock", clock_icon565, 32, 32);

  TopMainMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  TopMainMenu.setMenuBarMargins(10, 319, 6, 2);
  TopMainMenu.setItemTextMargins(10, 9, 5);
  TopMainMenu.setItemColors(C_GREY, MENU_HIGHBORDER);
  TopMainMenu.setTitleTextMargins(50, 13);

  RaceMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT,
                MENU_SELECTTEXT, MENU_SELECT, 230, 22, 8, "Driver Setup", FONT_14, FONT_16B);
  RaceMenuOption1 = RaceMenu.addNI("Driver #1", DriverID[0], 0, sizeof(DriverNames) / sizeof(DriverNames[0]), 1, 0, DriverNames);
  RaceMenuOption2 = RaceMenu.addNI("Driver #2", DriverID[1], 0, sizeof(DriverNames) / sizeof(DriverNames[0]), 1, 0, DriverNames);
  RaceMenuOption3 = RaceMenu.addNI("Driver #3", DriverID[2], 0, sizeof(DriverNames) / sizeof(DriverNames[0]), 1, 0, DriverNames);

  sprintf(buf, "Tires (%d psi)", TirePSIVal[TireID]);
  RaceMenuOption4 = RaceMenu.addNI(buf, TireID, 0, sizeof(TireText) / sizeof(TireText[0]), 1, 0, TireText);

  sprintf(buf, "Motor sprocket (%.2f)", GearRatio);
  RaceMenuOption5 = RaceMenu.addNI(buf, MotorSprocket, 10, 20, 1);
  sprintf(buf, "Wheel sprocket (%.2f)", GearRatio);
  RaceMenuOption6 = RaceMenu.addNI(buf, WheelSprocket, 20, 90, 1);
  RaceMenuOption7 = RaceMenu.addNI("Battery energy [whr]", TotalEnergy, 550, 750, 1);
  RaceMenuOption8 = RaceMenu.addNI("Battery 1", Battery1, 1, 99, 1);
  RaceMenuOption9 = RaceMenu.addNI("Battery 2", Battery2, 1, 99, 1);
  RaceMenuOption10 = RaceMenu.addNI("Add lap when pitting", AddLapInPit, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);

  RaceMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  RaceMenu.setItemTextMargins(2, 3, 5);
  RaceMenu.setMenuBarMargins(1, 319, 3, 1);
  RaceMenu.setItemColors(C_WHITE, MENU_HIGHBORDER);
  RaceMenu.setTitleTextMargins(50, 13);

  TelemetryMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT,
                     MENU_SELECTTEXT, MENU_SELECT, 230, 22, 8, "Telemetry Setup", FONT_14, FONT_16B);

  TelemetryMenuOption1 = TelemetryMenu.addNI("Motor", MotorID, 0, sizeof(MotorText) / sizeof(MotorText[0]), 1, 0, MotorText);
  TelemetryMenuOption3 = TelemetryMenu.addNI("Orientation", Orientation, 0, sizeof(OrientationText) / sizeof(OrientationText[0]), 1, 0, OrientationText);
  TelemetryMenuOption4 = TelemetryMenu.addNI("Background", Invert, 0, sizeof(InvertText) / sizeof(InvertText[0]), 1, 0, InvertText);
  TelemetryMenuOption5 = TelemetryMenu.addNI("Car", CarID, 0, sizeof(CarText) / sizeof(CarText[0]), 1, 0, CarText);
  TelemetryMenuOption6 = TelemetryMenu.addNI("Trigger amps", TriggerAmps, 20, 90, 5, 0);
  TelemetryMenuOption7 = TelemetryMenu.addNI("Restart race", ResetGPS, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  TelemetryMenuOption8 = TelemetryMenu.addNI("Use same file on restart", RestoreData, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  TelemetryMenuOption9 = TelemetryMenu.addNI("Restart display each draw", RestartDisplayAlways, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  TelemetryMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  TelemetryMenu.setItemTextMargins(2, 3, 5);
  TelemetryMenu.setMenuBarMargins(1, 319, 3, 1);
  TelemetryMenu.setItemColors(C_WHITE, MENU_HIGHBORDER);
  TelemetryMenu.setTitleTextMargins(50, 13);

  WirelessMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT,
                    MENU_SELECTTEXT, MENU_SELECT, 220, 22, 5, "Wireless Setup", FONT_14, FONT_16B);
  WirelessMenuOption1 = WirelessMenu.addNI("Send time", RadioUpdate, 0, sizeof(SendTimeText) / sizeof(SendTimeText[0]), 1, 0, SendTimeText);
  WirelessMenuOption2 = WirelessMenu.addNI("Channel", RadioChannel, 0, 31, 1);
  WirelessMenuOption3 = WirelessMenu.addNI("Data rate", AirDataRate, 0, sizeof(AirRateText) / sizeof(AirRateText[0]), 1, 0, AirRateText);
  WirelessMenuOption4 = WirelessMenu.addNI("GPS trigger range", GPSTolerance, 0, sizeof(GPSToleranceText) / sizeof(GPSToleranceText[0]), 1, 0, GPSToleranceText);
  WirelessMenuOption5 = WirelessMenu.addNI("GPS lap threashold [s]", LapThreashold, 10, 120, 5);
  WirelessMenuOption6 = WirelessMenu.addNI("Display GPS Speed", UseGPSForSpeedDisplay, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  WirelessMenuOption8 = WirelessMenu.addNI("Delay GPS start read", StartGPSDelayID, 0, sizeof(GPSReadTimeText) / sizeof(GPSReadTimeText[0]), 1, 0, GPSReadTimeText);
  WirelessMenuOption7 = WirelessMenu.addNI("RESET WIRELESS", Reset, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  WirelessMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  WirelessMenu.setItemTextMargins(2, 3, 5);
  WirelessMenu.setMenuBarMargins(1, 319, 3, 1);
  WirelessMenu.setItemColors(C_WHITE, MENU_HIGHBORDER);
  WirelessMenu.setTitleTextMargins(50, 13);

  SensorMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT,
                  MENU_SELECTTEXT, MENU_SELECT, 230, 22, 4, "Sensor Setup", FONT_14, FONT_16B);
  SensorMenuOption1 = SensorMenu.addNI("Volt slope (11.0)", VoltageSlope, 10.0, 12.0, 0.005, 3, NULL);
  SensorMenuOption2 = SensorMenu.addNI("Volt offset (0.32)", VoltageOffset, 0.20, 0.40, 0.001, 3, NULL);
  SensorMenuOption3 = SensorMenu.addNI("Amp slope (20.0)", mVPerAmp, 15, 25.0, 0.01, 2);
  SensorMenuOption4 = SensorMenu.addNI("Amp offset (0.5)", VMid, .4, 0.6, 0.001, 3);
  SensorMenuOption5 = SensorMenu.addNI("Temp. OHM (Motor)", ThermResMotor, 9000.0, 15000.0, 50, 0);
  SensorMenuOption6 = SensorMenu.addNI("Temp. OHM (AUX)", ThermResAux, 9000.0, 15000.0, 50, 0);
  SensorMenuOption7 = SensorMenu.addNI("Temp. Amb. (offset)", AmbTempCF, -20.0, 20.0, 1, 0);
  SensorMenuOption8 = SensorMenu.addNI("Temp warning [f]", TempWarning, 70, 160, 5, 0);
  SensorMenuOption9 = SensorMenu.addNI("Volt warning [v]", BatWarning, 10, 22, 0.1, 1);
  SensorMenuOption11 = SensorMenu.addNI("Pickups", Pickups, 1, 50, 1);
  SensorMenuOption12 = SensorMenu.addNI("Min. pulses", MinPulses, 2, 5, 1);
  SensorMenuOption13 = SensorMenu.addNI("Adjust altitude per lap", ResetAltitude, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  SensorMenuOption14 = SensorMenu.addNI("Zero current at startup", AutoCurrentCal, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);

  SensorMenu.disable(SensorMenuOption1);
  SensorMenu.disable(SensorMenuOption2);
  SensorMenu.disable(SensorMenuOption3);
  SensorMenu.disable(SensorMenuOption4);

  SensorMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  SensorMenu.setItemTextMargins(2, 3, 5);
  SensorMenu.setMenuBarMargins(1, 319, 3, 1);
  SensorMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  SensorMenu.setTitleTextMargins(50, 13);

  GForceMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT,
                  MENU_SELECTTEXT, MENU_SELECT, 230, 22, 4, "Accelerometer Setup", FONT_14, FONT_16B);

  GForceMenuOption5 = GForceMenu.addNI("Accelerometer", HasASensor, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  GForceMenuOption1 = GForceMenu.addNI("G-Force range", GForceRange, 0, sizeof(AccelFSRange) / sizeof(AccelFSRange[0]), 1, 0, AccelFSRange);
  GForceMenuOption6 = GForceMenu.addNI("Low Pass filter", AccelLPFilter, 0, sizeof(AccelLPFilterText) / sizeof(AccelLPFilterText[0]), 1, 0, AccelLPFilterText);
  GForceMenuOption7 = GForceMenu.addNI("High Pass filter", AccelHPFilter, 0, sizeof(AccelHPFilterText) / sizeof(AccelHPFilterText[0]), 1, 0, AccelHPFilterText);
  GForceMenuOption8 = GForceMenu.addNI("Auto Calibrate X Y Z", 0, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  GForceMenuOption2 = GForceMenu.addNI("Manual calibrate X", AccelCalX, -8000, 8000, 50, 0);
  GForceMenuOption3 = GForceMenu.addNI("Manual calibrate Y", AccelCalY, -8000, 8000, 50, 0);
  GForceMenuOption4 = GForceMenu.addNI("Manual calibrate Z", AccelCalZ, -8000, 8000, 50, 0);

  GForceMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  GForceMenu.setItemTextMargins(2, 3, 5);
  GForceMenu.setMenuBarMargins(1, 319, 3, 1);
  GForceMenu.setItemColors(C_WHITE, MENU_HIGHBORDER);
  GForceMenu.setTitleTextMargins(50, 13);

  ClockMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT,
                 MENU_SELECTTEXT, MENU_SELECT, 230, 22, 5, "Clock Setup", FONT_14, FONT_16B);
  ClockMenuOption1 = ClockMenu.addNI("Year", years, 2020, 2040, 1);
  ClockMenuOption2 = ClockMenu.addNI("Month", months, 1, 12, 1);
  ClockMenuOption3 = ClockMenu.addNI("Day", days, 1, 31, 1);
  ClockMenuOption4 = ClockMenu.addNI("Hour", hours, 0, 23, 1);
  ClockMenuOption5 = ClockMenu.addNI("Minute", minutes, 0, 60, 1);
  ClockMenuOption6 = ClockMenu.addNI("Allow calibrations", AllowCalibrations, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);

  ClockMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  ClockMenu.setItemTextMargins(2, 3, 5);
  ClockMenu.setMenuBarMargins(1, 319, 3, 1);
  ClockMenu.setItemColors(C_WHITE, MENU_HIGHBORDER);
  ClockMenu.setTitleTextMargins(50, 13);

  SSDMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, 35, 2, "Data Storage Options", FONT_16B, FONT_16B);

  SSDMenuOption1 = SSDMenu.addNI("ERASE SSD chip");
  SSDMenuOption2 = SSDMenu.addNI("Download all data");
  SSDMenuOption3 = SSDMenu.addNI("Download race data");
  SSDMenuOption4 = SSDMenu.addNI("Download GPS data");
  SSDMenuOption5 = SSDMenu.addNI("Download settings");
  SSDMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  SSDMenu.setMenuBarMargins(10, 319, 6, 2);
  SSDMenu.setItemTextMargins(10, 9, 5);
  SSDMenu.setItemColors(C_GREY, MENU_HIGHBORDER);
  SSDMenu.setTitleTextMargins(50, 13);

  MotorTempG.init(10, 181, 40, 105, 40, 160, 20, "Motor", C_WHITE, back_color, C_MAGENTA, C_DKMAGENTA, back_color, Arial_14, Arial_14);
  AuxTempG.init(110, 181, 40, 105, 40, 140, 20, "Auxiliary", C_WHITE, back_color, C_YELLOW, C_DKYELLOW, back_color, Arial_14, Arial_14);
  AmbTempG.init(220, 181, 40, 105, 40, 100, 10, "Ambient", C_WHITE, back_color, C_CYAN, C_DKCYAN, back_color, Arial_14, Arial_14);

  EnergyG.init("xxx", "Time [min]", "Power[W], Energy[Wh]", C_WHITE, C_DKGREY, C_BLUE, C_BLACK, C_BLACK, FONT_16B, FONT_14);

  EnergyID = EnergyG.add("Energy", C_CYAN);
  bEnergyID = EnergyG.add("Energy", C_RED);

  EnergyG.setLineThickness(EnergyID, 4);
  EnergyG.setLineThickness(bEnergyID, 2);

  EnergyG.showLegend(false);
  EnergyG.showTitle(false);
  EnergyG.showAxisLabels(false);

  EnergyG.setXTextOffset(5);

  GForceG.init("0", "0", "0", C_WHITE, C_DKGREY, C_WHITE, C_BLACK, C_BLACK, FONT_16B, FONT_14);
  GForceXID = GForceG.add("X", C_RED);
  GForceYID = GForceG.add("Y", C_GREEN);
  GForceZID = GForceG.add("Z", C_LTBLUE);
  GForceID = GForceG.add("Total", C_YELLOW);
  GForceG.setLineThickness(GForceXID, 2);
  GForceG.setLineThickness(GForceYID, 2);
  GForceG.setLineThickness(GForceZID, 2);
  GForceG.setLineThickness(GForceID, 4);
  GForceG.showLegend(true);
  GForceG.drawLegend(LOCATION_BOTTOM);
  GForceG.showTitle(false);
  GForceG.showAxisLabels(false);
  GForceG.showXScale(false);
  GForceG.setYTextOffset(37);
  GForceG.setYLegendOffset(-5);
}

/*
   PURPOSE : Setup main menu
    PARAMS :  -
   RETURNS : None
     NOTES :
*/
void ProcessMainMenu() {

  WatchDogTimer(DISABLE_WDT);

  MainMenuOption = 1;
  Display.fillScreen(C_BLACK);
  TopMainMenu.draw();

  while ((digitalRead(LPin) == LOW) | (digitalRead(RPin) == LOW)) {
    delay(10);
  }

  while (MainMenuOption > 0) {

    delay(50);

    if (digitalRead(LPin) == LOW) {
      val = Debounce(LPin, ButtonDebounce);

      if (val == LONG_PRESS) {

        MainMenuOption = TopMainMenu.selectRow();

        if (MainMenuOption == MainMenuOption1) {
          Display.fillScreen(C_BLACK);
          ProcessRaceMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption2) {
          Display.fillScreen(C_BLACK);
          ProcessTelemetryMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption3) {
          Display.fillScreen(C_BLACK);
          ProcessWirelessMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption4) {
          Display.fillScreen(C_BLACK);
          ProcessSensorMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption5) {
          Display.fillScreen(C_BLACK);
          ProcessClockMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption6) {
          Display.fillScreen(C_BLACK);
          ProcessSSDMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption7) {
          Display.fillScreen(C_BLACK);
          ProcessGForceMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        }
      } else if (val == SHORT_PRESS) {
        TopMainMenu.MoveUp();
      }
    }
    if (digitalRead(RPin) == LOW) {
      val = Debounce(RPin, ButtonDebounce);

      if (val == LONG_PRESS) {

        MainMenuOption = TopMainMenu.selectRow();

        if (MainMenuOption == MainMenuOption1) {
          Display.fillScreen(C_BLACK);
          ProcessRaceMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption2) {
          Display.fillScreen(C_BLACK);
          ProcessTelemetryMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption3) {
          Display.fillScreen(C_BLACK);
          ProcessWirelessMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption4) {
          Display.fillScreen(C_BLACK);
          ProcessSensorMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption5) {
          Display.fillScreen(C_BLACK);
          ProcessClockMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption6) {
          Display.fillScreen(C_BLACK);
          ProcessSSDMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        } else if (MainMenuOption == MainMenuOption7) {
          Display.fillScreen(C_BLACK);
          ProcessGForceMenu();
          Display.fillScreen(C_BLACK);
          TopMainMenu.draw();
        }
      } else if (val == SHORT_PRESS) {
        TopMainMenu.MoveDown();
      }
    }

    // during menu access, GPSSerial is still getting data
    // and can take time to clear out

    GPSSerial.read();
  }

  WatchDogTimer(ENABLE_WDT);
}


/*
  PURPOSE : Setup driver function
  PARAMS : -
  RETURNS : None
  NOTES : Allows user to set driver id's for later analysis
*/

void ProcessRaceMenu() {

  float ogr = GearRatio;
  uint8_t oTireID = TireID;

  MenuOption = 1;

  RaceMenu.draw();

  Display.setFont(FONT_14);
  Display.setTextColor(C_BLACK, C_GREY);

  RaceMenu.SetItemValue(RaceMenuOption1, DriverID[0]);
  RaceMenu.SetItemValue(RaceMenuOption2, DriverID[1]);
  RaceMenu.SetItemValue(RaceMenuOption3, DriverID[2]);

  while ((digitalRead(LPin) == LOW) | (digitalRead(RPin) == LOW)) {
    delay(10);
  }

  while (MenuOption > 0) {

    if (digitalRead(LPin) == LOW) {

      if ((RaceMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {
        while (digitalRead(LPin) == LOW) {
          RaceMenu.MoveUp();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }
      val = Debounce(LPin, ButtonDebounce);

      if (val == LONG_PRESS) {
        MenuOption = RaceMenu.selectRow();

      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }
        PressCount++;
        RaceMenu.MoveUp();

        MotorSprocket = RaceMenu.value[RaceMenuOption5];  // motor sprocket
        WheelSprocket = RaceMenu.value[RaceMenuOption6];  // wheel sprocket
        TireID = RaceMenu.value[RaceMenuOption4];
        GetGearParameters();

        if (ogr != GearRatio) {
          ogr = GearRatio;
          sprintf(buf, "Motor sprocket (%.2f)", GearRatio);
          RaceMenu.setItemText(RaceMenuOption5, buf);
          sprintf(buf, "Wheel sprocket (%.2f)", GearRatio);
          RaceMenu.setItemText(RaceMenuOption6, buf);
        }
        if (oTireID != TireID) {
          oTireID = TireID;
          sprintf(buf, "Tires (%d psi)", TirePSIVal[TireID]);
          RaceMenu.setItemText(RaceMenuOption4, buf);
        }
      }
    }

    if (digitalRead(RPin) == LOW) {

      if ((RaceMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {
        while (digitalRead(RPin) == LOW) {
          RaceMenu.MoveDown();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }
      val = Debounce(RPin, ButtonDebounce);
      if (val == LONG_PRESS) {
        MenuOption = RaceMenu.selectRow();
      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }

        PressCount++;
        RaceMenu.MoveDown();
        MotorSprocket = RaceMenu.value[RaceMenuOption5];  // motor sprocket
        WheelSprocket = RaceMenu.value[RaceMenuOption6];  // wheel sprocket
        TireID = RaceMenu.value[RaceMenuOption4];
        GetGearParameters();

        if (ogr != GearRatio) {
          ogr = GearRatio;
          sprintf(buf, "Motor sprocket (%.2f)", GearRatio);
          RaceMenu.setItemText(RaceMenuOption5, buf);
          sprintf(buf, "Wheel sprocket (%.2f)", GearRatio);
          RaceMenu.setItemText(RaceMenuOption6, buf);
        }
        if (oTireID != TireID) {
          oTireID = TireID;
          sprintf(buf, "Tires (%d psi)", TirePSIVal[TireID]);
          RaceMenu.setItemText(RaceMenuOption4, buf);
        }
      }
    }
  }

  DriverID[0] = (uint8_t)RaceMenu.value[RaceMenuOption1];
  DriverID[1] = (uint8_t)RaceMenu.value[RaceMenuOption2];
  DriverID[2] = (uint8_t)RaceMenu.value[RaceMenuOption3];
  TireID = (uint8_t)RaceMenu.value[RaceMenuOption4];     // tire id
  MotorSprocket = (int)RaceMenu.value[RaceMenuOption5];  // motor sprocket
  WheelSprocket = (int)RaceMenu.value[RaceMenuOption6];  // wheel sprocket
  TotalEnergy = RaceMenu.value[RaceMenuOption7];         // total energy
  Battery1 = (uint8_t)RaceMenu.value[RaceMenuOption8];
  Battery2 = (uint8_t)RaceMenu.value[RaceMenuOption9];
  AddLapInPit = (bool)RaceMenu.value[RaceMenuOption10];

  EEPROM.put(10, MotorSprocket);
  EEPROM.put(20, WheelSprocket);
  EEPROM.put(30, TireID);
  EEPROM.put(70, TotalEnergy);
  EEPROM.put(80, DriverID[0]);
  EEPROM.put(90, DriverID[1]);
  EEPROM.put(100, DriverID[2]);
  EEPROM.put(170, Battery1);
  EEPROM.put(175, Battery2);
  EEPROM.put(185, AddLapInPit);
}

/*
  PURPOSE : Setup car function
  PARAMS : -
  RETURNS : None
  NOTES :
*/

void ProcessTelemetryMenu() {

  MenuOption = 1;
  TelemetryMenu.draw();

  while ((digitalRead(LPin) == LOW) | (digitalRead(RPin) == LOW)) {
    delay(10);
  }

  while (MenuOption > 0) {

    if (digitalRead(LPin) == LOW) {
      if ((TelemetryMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {

        while (digitalRead(LPin) == LOW) {
          TelemetryMenu.MoveUp();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }
      val = Debounce(LPin, ButtonDebounce);

      if (val == LONG_PRESS) {
        MenuOption = TelemetryMenu.selectRow();

      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }

        PressCount++;
        TelemetryMenu.MoveUp();
      }
    }
    if (digitalRead(RPin) == LOW) {
      if ((TelemetryMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {

        while (digitalRead(RPin) == LOW) {
          TelemetryMenu.MoveDown();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }
      val = Debounce(RPin, ButtonDebounce);
      if (val == LONG_PRESS) {
        MenuOption = TelemetryMenu.selectRow();
      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }
        PressCount++;
        TelemetryMenu.MoveDown();
      }
    }
  }

  MotorID = (int)TelemetryMenu.value[TelemetryMenuOption1];          // motor ID
  Orientation = (uint8_t)TelemetryMenu.value[TelemetryMenuOption3];  // orientation
  Invert = (uint8_t)TelemetryMenu.value[TelemetryMenuOption4];       // invert background color
  CarID = (uint8_t)TelemetryMenu.value[TelemetryMenuOption5];        // car id
  TriggerAmps = (uint8_t)TelemetryMenu.value[TelemetryMenuOption6];
  ResetGPS = (uint8_t)TelemetryMenu.value[TelemetryMenuOption7];
  RestoreData = (uint8_t)TelemetryMenu.value[TelemetryMenuOption8];
  RestartDisplayAlways = (bool)TelemetryMenu.value[TelemetryMenuOption9];

  EEPROM.put(40, Invert);
  EEPROM.put(50, Orientation);
  EEPROM.put(130, MotorID);
  EEPROM.put(240, RestartDisplayAlways);
  EEPROM.put(300, CarID);
  EEPROM.put(380, TriggerAmps);
  EEPROM.put(410, RestoreData);

  if (ResetGPS) {
    RaceStatus = RACE_NOTSTARTED;
    SaveStartGPS(false);
    ResetRaceDate();
  }

  ResetGPS = false;
  TelemetryMenu.SetItemValue(TelemetryMenuOption7, ResetGPS);

  ConfigureButtons();
  SetScreenParameters();
  GetGearParameters();
}

/*
  PURPOSE : Setup transceivers function
  PARAMS : -
  RETURNS : None
  NOTES :
*/

void ProcessWirelessMenu() {

  unsigned long caltime = 0;
  MenuOption = 1;
  Reset = 0;

  // get the current parameters
  RadioChannel = Radio.GetChannel();
  AirDataRate = Radio.GetAirDataRate();
  oRadioChannel = RadioChannel;
  oAirDataRate = AirDataRate;

  // reset the menu data
  WirelessMenu.SetItemValue(WirelessMenuOption2, RadioChannel);
  WirelessMenu.SetItemValue(WirelessMenuOption3, AirDataRate);

  // now we are ready to draw
  WirelessMenu.draw();

  while ((digitalRead(LPin) == LOW) | (digitalRead(RPin) == LOW)) {
    delay(10);
  }

  while (MenuOption > 0) {

    delay(5);
    if (GPSTolerance != 0) {
      GPSRead();
    }

    if ((millis() - caltime) > 1000) {

      caltime = millis();

      Display.setTextColor(C_WHITE, C_BLACK);
      Display.setCursor(5, 170);
      Display.print(F("Coordinates"));
      Display.setCursor(5, 190);
      Display.print(F("Start coord"));
      Display.setCursor(5, 210);
      Display.print(F("Alt/Sat"));

      GPSLat = GPS.location.lat();
      GPSLon = GPS.location.lng();

      Altitude = BMEsensor.readAltitude(SEALEVELPRESSURE_HPA) * METERS_TO_FEET;

      GPSSatellites = GPS.satellites.value();
      GPSStatus = GPS.location.isValid();

      if (!GPSStatus) {
        GPSLon = 0.0;
        GPSLat = 0.0;
        GPSSatellites = 0;
      }

      Display.fillRect(128, 168, 240, 62, C_BLACK);

      Display.setTextColor(C_YELLOW, C_BLACK);
      Display.setCursor(130, 170);
      Display.print(GPSLon, 4);
      Display.print(F(" / "));
      Display.print(GPSLat, 4);

      Display.setCursor(130, 190);
      Display.print(GPSStartLon, 2);
      Display.print(F(" / "));
      Display.print(GPSStartLat, 2);

      Display.setCursor(130, 210);
      Display.print(Altitude, 0);
      Display.print(F(" / "));
      Display.print(GPSSatellites);

      // force a retest of valid GPS
      GPSStatus = false;
    }

    if (digitalRead(LPin) == LOW) {
      if ((WirelessMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {

        while (digitalRead(LPin) == LOW) {
          WirelessMenu.MoveUp();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }
      val = Debounce(LPin, ButtonDebounce);
      if (val == LONG_PRESS) {
        MenuOption = WirelessMenu.selectRow();
      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }

        PressCount++;
        WirelessMenu.MoveUp();
      }
    }

    if (digitalRead(RPin) == LOW) {
      if ((WirelessMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {

        while (digitalRead(RPin) == LOW) {
          WirelessMenu.MoveDown();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }
      val = Debounce(RPin, ButtonDebounce);
      if (val == LONG_PRESS) {
        MenuOption = WirelessMenu.selectRow();
      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }

        PressCount++;
        WirelessMenu.MoveDown();
      }
    }
  }

  RadioUpdate = (uint8_t)WirelessMenu.value[WirelessMenuOption1];
  RadioChannel = (uint8_t)WirelessMenu.value[WirelessMenuOption2];
  AirDataRate = (uint8_t)WirelessMenu.value[WirelessMenuOption3];
  GPSTolerance = (uint8_t)WirelessMenu.value[WirelessMenuOption4];
  LapThreashold = (uint8_t)WirelessMenu.value[WirelessMenuOption5];  // LapThreashold, seconds GPS considers a lap
  UseGPSForSpeedDisplay = (bool)WirelessMenu.value[WirelessMenuOption6];

  Reset = (uint8_t)WirelessMenu.value[WirelessMenuOption7];
  StartGPSDelayID = (uint8_t)WirelessMenu.value[WirelessMenuOption8];



  // bounds check GPS start delay
  if (StartGPSDelayID < ((sizeof(GPSReadTimeText) / sizeof(GPSReadTimeText[0])))) {
    StartGPSDelay = GPSReadTime[StartGPSDelayID];
  } else {
    StartGPSDelay = 0;
  }


  if ((oAirDataRate != AirDataRate) || (oRadioChannel != RadioChannel)) {
    Radio.SetChannel(RadioChannel);
    Radio.SetAirDataRate(AirDataRate);
    Radio.SaveParameters(PERMANENT);
  }

  // save stuff to eeprom
  EEPROM.put(60, RadioUpdate);
  EEPROM.put(160, LapThreashold);
  EEPROM.put(190, StartGPSDelayID);
  EEPROM.put(260, UseGPSForSpeedDisplay);
  EEPROM.put(280, GPSTolerance);

  if (!Radio.GetModel() || Reset == 1) {
    WirelessMenu.SetItemValue(WirelessMenuOption7, 0);
    Display.setFont(FONT_16B);
    Display.fillScreen(C_RED);
    for (i = 0; i < 3; i++) {
      Display.setCursor(20, 100);
      Display.print(F("Restoring... "));
      Display.print(i);
      delay(500);
      RestoreEBYTEDefaults();
      Display.setCursor(20, 150);
      Display.print(F("Retrying..."));
      delay(1000);
      if (Radio.init(3)) {
        Display.setCursor(20, 200);
        Display.print(F("Reset OK"));
        break;
      }
    }
  }

#ifdef DO_DEBUG
  Serial.println(F("******* EBYTE Parameters *******"));
  Radio.PrintParameters();
  Serial.println(F("******* End EBYTE Parameters *******"));
#endif
}

/*
  PURPOSE : Setup calculation function
  PARAMS: -
  RETURNS : None
  NOTES:
*/

void ProcessSensorMenu() {

  unsigned long caltime = millis();

  Counter = 0;
  vVolts = 0.0;
  aVolts = 0.0;
  thmVolts = 0.0;
  thxVolts = 0.0;
  WRPM = 0;
  SpeedUpdateTimer = 0;
  RPMSum = 0;
  RPMCount = 0;
  MenuOption = 1;
  SensorMenu.draw();

  Display.fillRect(0, 138, 319, 101, C_DKGREY);
  Display.setTextColor(C_WHITE);
  Display.setCursor(120, 140);
  Display.print(F("Computed"));
  Display.setCursor(230, 140);
  Display.print(F("Measured"));

  Display.setCursor(5, 160);
  Display.print(F("Volts @A9"));
  Display.setCursor(5, 180);

  Display.print(F("Amps @A3"));
  Display.setCursor(5, 200);
  Display.print(F("Temp M / X / A"));
  Display.setCursor(5, 220);
  Display.print(F("WRPM / Pulse"));

  while ((digitalRead(LPin) == LOW) || (digitalRead(RPin) == LOW)) {
    delay(10);
  }

  while (MenuOption > 0) {

    vVolts = vVolts + analogRead(VM_PIN);
    aVolts = aVolts + analogRead(AM_PIN);
    thmVolts = thmVolts + analogRead(THM_PIN);
    thxVolts = thxVolts + analogRead(THX_PIN);
    delay(1);
    Counter++;
    if (RPM.available()) {
      RPMSum = RPMSum + RPM.read();
      RPMCount++;
    }
    if ((millis() - caltime) > 500) {

      ComputeSpeed();

      vVolts = vVolts / Counter;
      vVolts = vVolts / (BIT_CONVERSION / 3.3f);
      Volts = (vVolts * VoltageSlope) + VoltageOffset;

      aVolts = aVolts / Counter;
      aVolts = aVolts / (BIT_CONVERSION / 3.3f);

      Amps = ((aVolts - VMid) * 1000.0f) / mVPerAmp;

      Display.fillRect(133, 158, 185, 84, C_DKGREY);

      Display.setCursor(140, 160);
      Display.setTextColor(C_YELLOW, C_DKGREY);
      Display.print(Volts, 2);
      Display.setCursor(240, 160);
      Display.setTextColor(C_CYAN, C_DKGREY);
      Display.print(vVolts, 3);

      Display.setCursor(140, 180);
      Display.setTextColor(C_YELLOW, C_DKGREY);
      Display.print(Amps, 3);
      Display.setCursor(240, 180);
      Display.setTextColor(C_CYAN, C_DKGREY);
      Display.print(aVolts, 3);

      // compute motor temperature
      thmVolts = thmVolts / Counter;
      thmVolts = thmVolts / (BIT_CONVERSION / 3.3f);
      // voltage divider calculation
      // vo = 5 * r2 /(r1+r2)
      // solve for r2
      // get the exact value for voltage divider r1
      tr2 = (thmVolts * ThermResMotor) / (3.3f - thmVolts);
      //equation from data sheet
      TempK = 1.0f / (NTC_A + (NTC_B * (log(tr2 / 10000.0f))) + (NTC_C * pow(log(tr2 / 10000.0f), 2)) + (NTC_D * pow(log(tr2 / 10000.0f), 3)));
      MotorTemp = (TempK * 1.8f) - 459.67f;
      if ((MotorTemp > 299.0f) || (MotorTemp < 0.0f)) {
        MotorTemp = 0.0;
      }

      Display.setCursor(140, 200);
      Display.setTextColor(C_YELLOW, C_DKGREY);
      Display.print(MotorTemp, 1);
      Display.print(F(" / "));

      // compute motor temperature
      thxVolts = thxVolts / Counter;
      thxVolts = thxVolts / (BIT_CONVERSION / 3.3f);
      // voltage divider calculation
      // vo = 5 * r2 /(r1+r2)
      // solve for r2
      // get the exact value for voltage divider r1
      tr2 = (thxVolts * ThermResAux) / (3.3f - thxVolts);
      //equation from data sheet
      TempK = 1.0f / (NTC_A + (NTC_B * (log(tr2 / 10000.0f))) + (NTC_C * pow(log(tr2 / 10000.0f), 2)) + (NTC_D * pow(log(tr2 / 10000.0f), 3)));
      AuxTemp = (TempK * 1.8f) - 459.67f;
      if ((AuxTemp > 299.0f) || (AuxTemp < 0.0f)) {
        AuxTemp = 0.0;
      }
      Display.print(AuxTemp, 1);
      Display.print(F(" / "));
      Display.print((BMEsensor.readTemperature() * 1.8) + 32.0 + AmbTempCF, 1);

      Display.setCursor(140, 220);
      Display.setTextColor(C_YELLOW, C_DKGREY);
      Display.print(WRPM);
      Display.setCursor(240, 220);
      Display.setTextColor(C_CYAN, C_DKGREY);
      Display.print(RPMCount);

      // need to get volts / amps and display
      Counter = 0;
      vVolts = 0.0f;
      aVolts = 0.0f;
      thmVolts = 0.0f;
      thxVolts = 0.0f;
      WRPM = 0;
      RPMSum = 0;
      RPMCount = 0;
      caltime = millis();
    }
    if (digitalRead(LPin) == LOW) {

      if ((SensorMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {

        while (digitalRead(LPin) == LOW) {
          SensorMenu.MoveUp();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }

      val = Debounce(LPin, ButtonDebounce);

      if (val == LONG_PRESS) {

        PressCount = 0;
        // we need to get all these parameters so live measurements will update
        MenuOption = SensorMenu.selectRow();
        VoltageSlope = SensorMenu.value[SensorMenuOption1];         // volt slope
        VoltageOffset = SensorMenu.value[SensorMenuOption2];        // volt offset
        mVPerAmp = SensorMenu.value[SensorMenuOption3];             // amp slope
        VMid = SensorMenu.value[SensorMenuOption4];                 // amp offset
        ThermResMotor = SensorMenu.value[SensorMenuOption5];        // temp offset
        ThermResAux = SensorMenu.value[SensorMenuOption6];          // temp offset
        AmbTempCF = SensorMenu.value[SensorMenuOption7];            // temp offset for BME
        Pickups = (uint8_t)SensorMenu.value[SensorMenuOption11];    // pickups
        MinPulses = (uint8_t)SensorMenu.value[SensorMenuOption12];  // minimum pulses
      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }

        PressCount++;
        SensorMenu.MoveUp();
      }
    }

    if (digitalRead(RPin) == LOW) {
      if ((SensorMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {

        while (digitalRead(RPin) == LOW) {
          SensorMenu.MoveDown();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }

      val = Debounce(RPin, ButtonDebounce);
      if (val == LONG_PRESS) {
        MenuOption = SensorMenu.selectRow();
        VoltageSlope = SensorMenu.value[SensorMenuOption1];       // volt slope
        VoltageOffset = SensorMenu.value[SensorMenuOption2];      // volt offset
        mVPerAmp = SensorMenu.value[SensorMenuOption3];           // amp slope
        VMid = SensorMenu.value[SensorMenuOption4];               // amp offset
        ThermResMotor = SensorMenu.value[SensorMenuOption5];      // temp offset
        ThermResAux = SensorMenu.value[SensorMenuOption6];        // temp offset
        AmbTempCF = SensorMenu.value[SensorMenuOption7];          // temp offset for BME
        Pickups = (uint8_t)SensorMenu.value[SensorMenuOption11];  // pickups
        MinPulses = (uint8_t)SensorMenu.value[SensorMenuOption12];
      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }

        PressCount++;

        SensorMenu.MoveDown();
      }
    }
  }

  RPMSum = 0;
  RPMCount = 0;

  VoltageSlope = SensorMenu.value[SensorMenuOption1];         // volt slope
  VoltageOffset = SensorMenu.value[SensorMenuOption2];        // volt offset
  mVPerAmp = SensorMenu.value[SensorMenuOption3];             // amp slope
  VMid = SensorMenu.value[SensorMenuOption4];                 // amp offset
  ThermResMotor = SensorMenu.value[SensorMenuOption5];        // temp thermistor voltage divider ext
  ThermResAux = SensorMenu.value[SensorMenuOption6];          // temp thermistor voltage divider int
  AmbTempCF = SensorMenu.value[SensorMenuOption7];            // temp offset for BME
  TempWarning = SensorMenu.value[SensorMenuOption8];          // temp warning
  BatWarning = SensorMenu.value[SensorMenuOption9];           // battery voltage warning
  Pickups = (uint8_t)SensorMenu.value[SensorMenuOption11];    // pickups
  MinPulses = (uint8_t)SensorMenu.value[SensorMenuOption12];  // min pulses
  ResetAltitude = (uint8_t)SensorMenu.value[SensorMenuOption13];
  AutoCurrentCal = (bool)SensorMenu.value[SensorMenuOption14];

  EEPROM.put(55, AutoCurrentCal);
  EEPROM.put(110, VoltageSlope);
  EEPROM.put(120, VoltageOffset);
  EEPROM.put(140, TempWarning);
  EEPROM.put(150, BatWarning);
  EEPROM.put(180, MinPulses);
  EEPROM.put(210, Pickups);
  EEPROM.put(215, ResetAltitude);
  EEPROM.put(220, mVPerAmp);
  EEPROM.put(230, VMid);
  EEPROM.put(310, ThermResMotor);
  EEPROM.put(315, ThermResAux);
  EEPROM.put(480, AmbTempCF);
}

/*
  PURPOSE : Setup clock menu
  PARAMS: -
  RETURNS : None
  NOTES:
*/

void ProcessClockMenu() {

  years = year();
  months = month();
  days = day();
  hours = hour();
  minutes = minute();
  seconds = second();

  ClockMenu.SetItemValue(ClockMenuOption1, years);
  ClockMenu.SetItemValue(ClockMenuOption2, months);
  ClockMenu.SetItemValue(ClockMenuOption3, days);
  ClockMenu.SetItemValue(ClockMenuOption4, hours);
  ClockMenu.SetItemValue(ClockMenuOption5, minutes);

  // get the last recordset

  ClockMenu.setLimits(ClockMenuOption7, 1, RecordSETID, 1, 0);

  MenuOption = 1;
  ClockMenu.draw();

  Display.fillRect(0, 160, 320, 80, C_GREY);

  Display.setTextColor(C_BLACK, C_GREY);

  Display.setCursor(10, 170);
  Display.print(F("Code v: "));
  Display.print(CODE_VERSION);

  Display.setCursor(10, 190);
  Display.print(F("Utilities v: "));
  Display.print(UTILITIES_VERSION);

  Display.setCursor(10, 210);
  Display.print(F("Database v: "));
  Display.print(BULLET_DB_VER);

  while ((digitalRead(LPin) == LOW) || (digitalRead(RPin) == LOW)) {
    delay(10);
  }

  while (MenuOption > 0) {

    if (digitalRead(LPin) == LOW) {
      if ((ClockMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {
        while (digitalRead(LPin) == LOW) {
          ClockMenu.MoveUp();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }

      val = Debounce(LPin, ButtonDebounce);
      if (val == LONG_PRESS) {
        MenuOption = ClockMenu.selectRow();

      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }

        PressCount++;
        ClockMenu.MoveUp();
      }
    }

    if (digitalRead(RPin) == LOW) {

      if ((ClockMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {

        while (digitalRead(RPin) == LOW) {
          ClockMenu.MoveDown();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }
      val = Debounce(RPin, ButtonDebounce);
      if (val == LONG_PRESS) {
        MenuOption = ClockMenu.selectRow();
      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }

        PressCount++;
        ClockMenu.MoveDown();
      }
    }
  }

  years = (int)ClockMenu.value[ClockMenuOption1];
  months = (int)ClockMenu.value[ClockMenuOption2];
  days = (int)ClockMenu.value[ClockMenuOption3];
  hours = (int)ClockMenu.value[ClockMenuOption4];
  minutes = (int)ClockMenu.value[ClockMenuOption5];
  AllowCalibrations = (bool)ClockMenu.value[ClockMenuOption6];

  seconds = 1;

  setTime(hours, minutes, seconds, days, months, years);

  Teensy3Clock.set(now());

  if (AllowCalibrations) {
    SensorMenu.enable(SensorMenuOption1);
    SensorMenu.enable(SensorMenuOption2);
    SensorMenu.enable(SensorMenuOption3);
    SensorMenu.enable(SensorMenuOption4);

  } else {
    SensorMenu.disable(SensorMenuOption1);
    SensorMenu.disable(SensorMenuOption2);
    SensorMenu.disable(SensorMenuOption3);
    SensorMenu.disable(SensorMenuOption4);
  }
}

void ProcessGForceMenu() {

  MenuOption = 1;

  GForceMenu.draw();

  while ((digitalRead(LPin) == LOW) || (digitalRead(RPin) == LOW)) {
    delay(10);
  }
  Display.setFont(FONT_24BI);
  Display.fillRect(0, 160, 320, 100, C_DKGREY);
  Display.setCursor(40, 170);
  Display.setTextColor(C_RED, C_DKGREY);
  Display.print(F("X"));
  Display.setCursor(140, 170);
  Display.setTextColor(C_GREEN, C_DKGREY);
  Display.print(F("Y"));
  Display.setCursor(240, 170);
  Display.setTextColor(C_BLUE, C_DKGREY);
  Display.print(F("Z"));

  while (MenuOption > 0) {

    if (digitalRead(LPin) == LOW) {
      if ((GForceMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {

        while (digitalRead(LPin) == LOW) {
          GForceMenu.MoveUp();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }
      val = Debounce(LPin, ButtonDebounce);
      if (val == LONG_PRESS) {
        MenuOption = GForceMenu.selectRow();

        if (GForceMenu.value[GForceMenuOption8] == 1) {
          CalibrateAccererometer();
          GForceMenu.SetItemValue(GForceMenuOption8, 0);
          GForceMenu.drawRow(GForceMenuOption8);
          GForceMenu.value[GForceMenuOption2] = AccelCalX;
          GForceMenu.value[GForceMenuOption3] = AccelCalY;
          GForceMenu.value[GForceMenuOption4] = AccelCalZ;
          GForceMenu.drawRow(GForceMenuOption2);
          GForceMenu.drawRow(GForceMenuOption3);
          GForceMenu.drawRow(GForceMenuOption4);
          Display.setFont(FONT_24BI);
          Display.fillRect(0, 160, 320, 100, C_DKGREY);
          Display.setCursor(40, 170);
          Display.setTextColor(C_RED, C_DKGREY);
          Display.print(F("X"));
          Display.setCursor(140, 170);
          Display.setTextColor(C_GREEN, C_DKGREY);
          Display.print(F("Y"));
          Display.setCursor(240, 170);
          Display.setTextColor(C_BLUE, C_DKGREY);
          Display.print(F("Z"));
        }
        if (MenuOption == GForceMenuOption1) {
          GForceRange = (int16_t)GForceMenu.value[GForceMenuOption1];
          GForceG.setYAxis(-pow(2, GForceRange + 1), pow(2, GForceRange + 1), pow(2, GForceRange + 1) / 4);
          SetupAccelerometer();
        }
        if (MenuOption == GForceMenuOption2) {
          AccelCalX = (int16_t)GForceMenu.value[GForceMenuOption2];
          SetupAccelerometer();
        }

        if (MenuOption == GForceMenuOption3) {
          AccelCalY = (int16_t)GForceMenu.value[GForceMenuOption3];
          SetupAccelerometer();
        }
        if (MenuOption == GForceMenuOption4) {
          AccelCalZ = (int16_t)GForceMenu.value[GForceMenuOption4];
          SetupAccelerometer();
        }

      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }
        PressCount++;
        GForceMenu.MoveUp();
      }
    }

    if (digitalRead(RPin) == LOW) {
      if ((GForceMenu.isEditing()) && (PressCount >= 3) && (PressTimer < SPEEDMENU_LIMIT)) {

        while (digitalRead(RPin) == LOW) {
          GForceMenu.MoveDown();
          delay(SPEEDMENU_DELAY);
        }
        PressCount = 0;
        PressTimer = 0;
      }
      val = Debounce(RPin, ButtonDebounce);
      if (val == LONG_PRESS) {
        MenuOption = GForceMenu.selectRow();
        if (GForceMenu.value[GForceMenuOption8] == 1) {
          CalibrateAccererometer();
          GForceMenu.SetItemValue(GForceMenuOption8, 0);
          GForceMenu.drawRow(GForceMenuOption8);
          GForceMenu.value[GForceMenuOption2] = AccelCalX;
          GForceMenu.value[GForceMenuOption3] = AccelCalY;
          GForceMenu.value[GForceMenuOption4] = AccelCalZ;
          GForceMenu.drawRow(GForceMenuOption2);
          GForceMenu.drawRow(GForceMenuOption3);
          GForceMenu.drawRow(GForceMenuOption4);

          Display.setFont(FONT_24BI);
          Display.fillRect(0, 160, 320, 100, C_DKGREY);
          Display.setCursor(40, 170);
          Display.setTextColor(C_RED, C_DKGREY);
          Display.print(F("X"));
          Display.setCursor(140, 170);
          Display.setTextColor(C_GREEN, C_DKGREY);
          Display.print(F("Y"));
          Display.setCursor(240, 170);
          Display.setTextColor(C_BLUE, C_DKGREY);
          Display.print(F("Z"));
        }
        if (MenuOption == GForceMenuOption1) {
          GForceRange = (int16_t)GForceMenu.value[GForceMenuOption1];
          GForceG.setYAxis(-pow(2, GForceRange + 1), pow(2, GForceRange + 1), pow(2, GForceRange + 1) / 4);
          SetupAccelerometer();
        }
        if (MenuOption == GForceMenuOption2) {
          AccelCalX = (int16_t)GForceMenu.value[GForceMenuOption2];
          SetupAccelerometer();
        }

        if (MenuOption == GForceMenuOption3) {
          AccelCalY = (int16_t)GForceMenu.value[GForceMenuOption3];
          SetupAccelerometer();
        }
        if (MenuOption == GForceMenuOption4) {
          AccelCalZ = (int16_t)GForceMenu.value[GForceMenuOption4];
          SetupAccelerometer();
        }

      } else if (val == SHORT_PRESS) {
        if (PressTimer > SPEEDMENU_LIMIT) {
          PressTimer = 0;
          PressCount = 0;
        }

        PressCount++;
        GForceMenu.MoveDown();
      }
    }

    ax = ASensor.getAccelerationX() / ASensorBits;
    ay = ASensor.getAccelerationY() / ASensorBits;
    az = ASensor.getAccelerationZ() / ASensorBits;

    Display.setFont(FONT_24BI);
    Display.setCursor(20, 210);
    ffACalX.setTextColor(C_RED, C_DKGREY);
    ffACalX.print(ax, 2);

    Display.setCursor(120, 210);
    ffACalY.setTextColor(C_GREEN, C_DKGREY);
    ffACalY.print(ay, 2);

    Display.setCursor(220, 210);
    ffACalZ.setTextColor(C_BLUE, C_DKGREY);
    ffACalZ.print(az, 2);
    delay(50);
  }

  AccelCalX = (int16_t)GForceMenu.value[GForceMenuOption2];      // x cal
  AccelCalY = (int16_t)GForceMenu.value[GForceMenuOption3];      // y cal
  AccelCalZ = (int16_t)GForceMenu.value[GForceMenuOption4];      // z cal
  HasASensor = (bool)GForceMenu.value[GForceMenuOption5];        // z cal
  AccelLPFilter = (uint8_t)GForceMenu.value[GForceMenuOption6];  //LP filter DLPF_CFG
  AccelHPFilter = (uint8_t)GForceMenu.value[GForceMenuOption7];  //HP Filtern DHPF_CFG

  if (HasASensor) {

    if (ASensor.isConnected()) {
      GForceStatus = true;
    } else {
      ASensor.initialize();
      if (ASensor.isConnected()) {
        GForceStatus = true;
        SetupAccelerometer();
      }
    }
  }

  EEPROM.put(200, AccelLPFilter);
  EEPROM.put(205, AccelHPFilter);
  EEPROM.put(250, GForceRange);
  EEPROM.put(450, AccelCalX);
  EEPROM.put(455, HasASensor);
  EEPROM.put(460, AccelCalY);
  EEPROM.put(470, AccelCalZ);
}

void ProcessSSDMenu() {

  MenuOption = 1;

  SSDMenu.draw();

  ShowJEDECScreen();

  while ((digitalRead(LPin) == LOW) || (digitalRead(RPin) == LOW)) {
    delay(10);
  }

  while (MenuOption > 0) {

    delay(5);

    if (SSDMenu.item > SSDMenuOption1) {
      if (digitalRead(CD_PIN) == HIGH) {
        SDCardStatus = false;  //no card
        Display.setCursor(185, 138);
        Display.setFont(FONT_14);
        Display.setTextColor(C_RED, C_WHITE);
        Display.print(F("NO SD CARD"));
      } else {
        if (SSDMenu.item > SSDMenuOption1) {
          Display.fillRect(184, 138, 134, 25, C_WHITE);
        }
      }
    }

    if (digitalRead(LPin) == LOW) {
      val = Debounce(LPin, ButtonDebounce);
      if (val == LONG_PRESS) {

        MenuOption = SSDMenu.selectRow();

        if (MenuOption == SSDMenuOption1) {
          Display.fillRect(0, 130, 320, 110, C_BLACK);
          Display.drawRect(0, 130, 320, 110, C_WHITE);
          Display.setFont(FONT_16B);
          Display.setTextColor(C_RED, back_color);
          Display.setCursor(10, 140);
          Display.print(F("This will take approx 1 min."));
          Display.setCursor(10, 165);
          Display.print(F("Erasing chip..."));
          SSD.eraseAll();
          Display.setCursor(10, 190);
          Display.print(F("Resetting race..."));
          delay(500);
          SaveStartGPS(false);
          ResetRaceDate();
          RecordSETID = 0;
          Display.setCursor(10, 215);
          Display.print(F("Process complete."));
          delay(1000);
          DrawInfoScreen();
        }
        if (MenuOption == SSDMenuOption2) {  //all
          DownloadRaceData(2);
          DownloadGPSData(1);
          DownloadEEPROM();
          DrawDownloadInfoScreen();
        }
        if (MenuOption == SSDMenuOption3) {
          DownloadRaceData(1);
          DrawDownloadInfoScreen();
        }
        if (MenuOption == SSDMenuOption4) {
          DownloadGPSData(1);
          DrawDownloadInfoScreen();
        }
        if (MenuOption == SSDMenuOption5) {
          DownloadEEPROM();
          DrawDownloadSetupScreen();
        }

      } else if (val == SHORT_PRESS) {
        SSDMenu.MoveUp();
        if (SSDMenu.item == 0) {
          ShowJEDECScreen();
        }
        if (SSDMenu.item == SSDMenuOption1) {
          DrawInfoScreen();
        }
        if (SSDMenu.item > SSDMenuOption1) {
          DrawDownloadInfoScreen();
        }
      }
    }

    if (digitalRead(RPin) == LOW) {
      val = Debounce(RPin, ButtonDebounce);
      if (val == LONG_PRESS) {

        MenuOption = SSDMenu.selectRow();

        if (MenuOption == SSDMenuOption1) {
          Display.fillRect(0, 130, 320, 110, C_BLACK);
          Display.drawRect(0, 130, 320, 110, C_WHITE);
          Display.setFont(FONT_16B);
          Display.setTextColor(C_RED, back_color);
          Display.setCursor(10, 140);
          Display.print(F("This will take approx 1 min."));
          Display.setCursor(10, 165);
          Display.print(F("Erasing chip..."));
          SSD.eraseAll();
          Display.setCursor(10, 190);
          Display.print(F("Resetting race..."));
          delay(500);
          SaveStartGPS(false);
          ResetRaceDate();
          RecordSETID = 0;
          Display.setCursor(10, 215);
          Display.print(F("Process complete."));
          delay(1000);
          DrawInfoScreen();
        }

        if (MenuOption == SSDMenuOption2) {
          DownloadRaceData(2);
          DownloadGPSData(2);
          DownloadEEPROM();
          DrawDownloadInfoScreen();
        }
        if (MenuOption == SSDMenuOption3) {
          DownloadRaceData(1);
          DrawDownloadInfoScreen();
        }
        if (MenuOption == SSDMenuOption4) {
          DownloadGPSData(1);
          DrawDownloadInfoScreen();
        }
        if (MenuOption == SSDMenuOption5) {
          DownloadEEPROM();
          DrawDownloadSetupScreen();
        }

      } else if (val == SHORT_PRESS) {
        SSDMenu.MoveDown();
        if (SSDMenu.item == 0) {
          ShowJEDECScreen();
        }
        if (SSDMenu.item == SSDMenuOption1) {
          DrawInfoScreen();
        }
        if (SSDMenu.item > SSDMenuOption1) {
          DrawDownloadInfoScreen();
        }
      }
    }
  }
}

void ShowJEDECScreen() {
  Display.fillRect(0, 130, 320, 110, C_BLACK);
  Display.drawRect(0, 130, 320, 110, C_WHITE);
  Display.setFont(FONT_16B);
  Display.setTextColor(C_WHITE, back_color);
  Display.setCursor(15, 140);
  Display.print(F("Chip JEDEC: "));
  Display.print(SSD.getChipJEDEC());
  Display.setCursor(15, 165);
  Display.print(F("Used space (kb)"));
  Display.setCursor(15, 190);
  Display.print(F("Free space (kb)"));
  Display.setCursor(200, 165);
  Display.print(SSD.getUsedSpace() / 1000);
  Display.setCursor(200, 190);
  Display.print((SSD.getTotalSpace() - SSD.getUsedSpace()) / 1000);
}

void DrawDownloadSetupScreen() {
  Display.fillRect(0, 130, 320, 110, C_WHITE);
  Display.drawRect(0, 130, 320, 110, C_BLACK);
  Display.setFont(FONT_14);
  Display.setTextColor(C_BLACK, C_WHITE);
  Display.setCursor(10, 135);
  Display.print(F("Download progress"));
  Display.drawRoundRect(10, 180, 300, 40, 4, C_BLACK);
  Display.drawRoundRect(11, 181, 298, 38, 3, C_BLACK);
}

void DrawInfoScreen() {

  // draw info screen
  Display.fillRect(0, 130, 320, 110, C_WHITE);
  Display.drawRect(0, 130, 320, 110, C_BLACK);
  Display.setFont(FONT_14);
  Display.setTextColor(C_BLACK, C_WHITE);
  Display.setCursor(10, 135);
  Display.print(F("File Storage Information"));
  Display.setCursor(15, 160);
  Display.print(F("Recordsets"));
  Display.setCursor(15, 185);
  Display.print(F("Used space (kb)"));
  Display.setCursor(15, 210);
  Display.print(F("Free space (kb)"));
  Display.setCursor(200, 160);
  Display.print(RecordSETID);
  Display.setCursor(200, 185);
  Display.print(SSD.getUsedSpace() / 1000);
  Display.setCursor(200, 210);
  Display.print((SSD.getTotalSpace() - SSD.getUsedSpace()) / 1000);
}

void DrawDownloadInfoScreen() {

  Display.fillRect(0, 130, 320, 110, C_WHITE);
  Display.drawRect(0, 130, 320, 110, C_BLACK);
  Display.setFont(FONT_14);
  Display.setTextColor(C_BLACK, C_WHITE);
  Display.setCursor(10, 135);
  Display.print(F("Download progress"));
  Display.drawRoundRect(10, 180, 300, 40, 4, C_BLACK);
  Display.drawRoundRect(11, 181, 298, 38, 3, C_BLACK);
}

void DownloadRaceData(uint32_t Count) {

  bool SDCardStatus = false, PitStop = false;
  uint8_t temp = 0, next = 0, rt = 0, tWS = 0, tMS = 0, PitCount = 0;
  int16_t OldLap = -1;
  uint32_t i = 0, rw = 0;
  char FileName[27] = "C_RRR_YYYY-MM-DD_NNN.csv";
  uint16_t tPoint = 0;
  bool OKtoClose = false;
  uint32_t HeaderRecord = 0;
  float TempEnergy = 0.0f, EnergyOffset = 0.0f, CurrentEnergy = 0.0f;

  fc = 0;

  if (digitalRead(CD_PIN) == HIGH) {
    Display.setTextColor(C_RED, C_WHITE);
    Display.fillRect(184, 138, 134, 25, C_WHITE);
    Display.setCursor(200, 135);
    Display.print(F("NO SD CARD"));
    return;
  }

  SDCardStatus = SDCARD.begin(SDCS_PIN, SD_SCK_MHZ(SD_SPI_SPEED));  //SD

  if (!SDCardStatus) {
    Display.setTextColor(C_RED, C_WHITE);
    Display.fillRect(184, 138, 134, 25, C_WHITE);
    Display.setCursor(200, 135);
    Display.print(F("NO SD CARD"));
    return;
  }

  SSD.gotoRecord(1);

  LastRecord = SSD.getLastRecord();

  //Serial.print("Records to download "); Serial.println(tc);
  fc = 0;

  for (i = 1; i <= LastRecord; i++) {

    SSD.gotoRecord(i);

    rt = SSD.getField(RecordType, frType);

    if (rt == NULL_RECORD) {
      //Serial.println("force close");
      SDDataFile.close();
      return;
    }

    // advance progress indicator
    fc++;
    rw = ((float)(fc * 293.0) / (Count * LastRecord)) + 2;
    Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

    if (rt == RT_HEADER) {

      //reset pit stop counter
      PitCount = 0;
      PitStop = false;
      if (OKtoClose) {
        SDDataFile.close();
        OKtoClose = false;
      }

      HeaderRecord = i;

      sprintf(FileName, "C_%03d_%04d-%02d-%02d_000.csv",
              SSD.getHeaderField(RecordSETID, hrID),
              SSD.getHeaderField(Tyear, hrYear),
              SSD.getHeaderField(Tmonth, hrMonth),
              SSD.getHeaderField(Tday, hrDay));

      temp = SSD.getHeaderField(CarID, hrCarID);

      if (temp == BLUE_CAR) {
        FileName[0] = 'B';
      } else if (temp == RED_CAR) {
        FileName[0] = 'R';
      } else {
        FileName[0] = 'W';
      }

      Display.fillRect(10, 160, 285, 20, C_WHITE);
      Display.setCursor(10, 160);
      Display.print(FileName);
      Display.setTextColor(C_BLACK, C_WHITE);
      next = 0;
      while (SDCARD.exists(FileName)) {
        next++;

        FileName[17] = (int)((next / 100) % 10) + '0';
        FileName[18] = (int)((next / 10) % 10) + '0';
        FileName[19] = (int)(next % 10) + '0';

        Display.fillRect(10, 160, 285, 20, C_WHITE);
        Display.setTextColor(C_BLACK, C_WHITE);
        Display.setCursor(10, 160);
        Display.print(FileName);

        if (next > 999) {
          break;
        }
      }

      SDCardStatus = SDDataFile.open(FileName, O_WRITE | O_CREAT);

      if (!SDCardStatus) {
        // Serial.println("NoSd card");
        Display.setTextColor(C_RED, C_WHITE);
        Display.setCursor(200, 160);
        Display.print(F("No SD Card"));
        delay(1000);
        return;
      }

      // to do write file time stamp stuff
      SDDataFile.timestamp(T_CREATE,
                           (int)SSD.getHeaderField(Tyear, hrYear),
                           (int)SSD.getHeaderField(Tmonth, hrMonth),
                           (int)SSD.getHeaderField(Tday, hrDay),
                           (int)SSD.getHeaderField(Thour, hrHour),
                           (int)SSD.getHeaderField(Tminute, hrMinute),
                           0);

      SDDataFile.timestamp(T_WRITE,
                           (int)year(),
                           (int)month(),
                           (int)day(),
                           (int)hour(),
                           (int)minute(),
                           0);

      SDDataFile.timestamp(T_ACCESS,
                           (int)year(),
                           (int)month(),
                           (int)day(),
                           (int)hour(),
                           (int)minute(),
                           0);

      // print fields
      SDDataFile.print(F("Point"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Time [min]"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Lap"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Driver"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Name"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Volts"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Amps"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Temp Motor"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Temp Aux"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Power"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Energy"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("RPM"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Speed"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Distance"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Time"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("GPS Speed"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Altitude"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel X"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel Y"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel Z"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("G-Force"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Humidity"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Amb. Temp"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Energy/Lap"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Comments"));

      // Write first line of header
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("PATRIOT RACING RACE RESULTS - CONFIDENTIAL"));
      SDDataFile.println("");
    }

    if (rt == RT_DATA) {

      OKtoClose = true;
      // point
      tPoint = (uint16_t)SSD.getField(Point, frPoint);
      SDDataFile.print(tPoint);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(tPoint / 60.0 * (UPDATE_LIMIT / 1000.0), 3);

      // laps
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(LapCount, frLap));

      if (SSD.getField(LapCount, frLap) != OldLap) {
        OldLap = SSD.getField(LapCount, frLap);
        EnergyOffset = CurrentEnergy;
      }

      // driver
      temp = (uint8_t)SSD.getField(Driver, frDriver);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(temp + 1);  // driver starts at 0 make pretty and driver 1 is 1, but name lookup remains 0 based
      if (temp < ((sizeof(DriverNames) / sizeof(DriverNames[0])))) {
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(DriverNames[(int)DriverID[temp]]);
      } else {
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(F("UNKNOWN"));
      }
      // volts
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Volts, frVolts), 4);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Amps, frAmps), 4);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(MotorTemp, frMotorTemp), 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(AuxTemp, frAuxTemp), 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Volts, frVolts) * SSD.getField(Amps, frAmps), 0);
      SDDataFile.write(DATA_DELIMITER);
      CurrentEnergy = (SSD.getField(Energy, frEnergy));
      SDDataFile.print(CurrentEnergy, 1);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(mRPM, frRPM));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(CarSpeed, frSpeed), 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Distance, frDist), 4);

      RealClockTime = SSD.getField(RealClockTime, frRT);
      h = (int)(RealClockTime / 3600);
      m = (int)((RealClockTime - (h * 3600)) / 60);
      s = (int)(RealClockTime % 60);
      sprintf(str, "%02d:%02d:%02d", h, m, s);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(str);
      // convert to MPH
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(GPSSpeed, frGSpeed), 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Altitude, frAlt), 1);
      SDDataFile.write(DATA_DELIMITER);

      GForceX = SSD.getField(GForceX, frMax);
      GForceY = SSD.getField(GForceY, frMay);
      GForceZ = SSD.getField(GForceZ, frMaz);

      SDDataFile.print(GForceX, 4);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(GForceY, 4);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(GForceZ, 4);
      SDDataFile.write(DATA_DELIMITER);
      // compute and print the absolute max g force.
      SDDataFile.print(sqrt((GForceX * GForceX) + (GForceY * GForceY) + (GForceZ * GForceZ)), 4);

      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Humidity, frHumidity), 1);  //

      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(AmbTemp, frAmbTemp), 1);  //

      SDDataFile.write(DATA_DELIMITER);
      TempEnergy = CurrentEnergy - EnergyOffset;
      SDDataFile.print(TempEnergy, 2);  //

      // write comments
      SDDataFile.write(DATA_DELIMITER);

      if (SSD.getField(RestoreType, frRestoreType) == STATUS_OK) {
        if (!PitStop) {
          PitCount++;
          PitStop = true;
        }
      } else if (SSD.getField(RestoreType, frRestoreType) == STATUS_RESTORE) {

        SDDataFile.print(F("Restored"));
      } else if (SSD.getField(RestoreType, frRestoreType) == STATUS_PITSTOP) {
        PitStop = false;
        if (PitCount <= 2) {
          SDDataFile.print(F("Pit "));
          SDDataFile.print(PitCount);
        } else {
          SDDataFile.print(F("Pit X"));
        }
      }

      // print the summary block
      if (tPoint <= 55) {

        SSD.gotoRecord(HeaderRecord);

        // Leave 2 columns between data and header
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.write(DATA_DELIMITER);
        switch (tPoint) {
          case 2:
            // car details
            SDDataFile.print(F("CAR SETUP"));
            break;
          case 3:
            // car details
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Car"));
            SDDataFile.write(DATA_DELIMITER);
            temp = (uint8_t)SSD.getHeaderField(CarID, hrCarID);
            if (temp < ((sizeof(CarText) / sizeof(CarText[0])))) {
              SDDataFile.print(CarText[temp]);
            } else {
              SDDataFile.print(F("UNKNOWN"));
            }
            break;
          case 4:
            // date
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Date"));
            SDDataFile.write(DATA_DELIMITER);
            // Writing ="<date>" forces the date to be recongnized as text
            SDDataFile.write(61);
            SDDataFile.write(34);
            SDDataFile.print((int)SSD.getHeaderField(Tmonth, hrMonth));
            SDDataFile.print(F("/"));
            SDDataFile.print((int)SSD.getHeaderField(Tday, hrDay));
            SDDataFile.print(F("/"));
            SDDataFile.print((int)SSD.getHeaderField(Tyear, hrYear));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print((int)SSD.getHeaderField(Thour, hrHour));
            SDDataFile.print(F(":"));
            if (SSD.getHeaderField(Tminute, hrMinute) < 10) {
              SDDataFile.print(F("0"));
            }
            SDDataFile.print((int)SSD.getHeaderField(Tminute, hrMinute));
            break;
          case 5:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Tires"));
            temp = (uint8_t)SSD.getHeaderField(TireID, hrTireID);
            SDDataFile.write(DATA_DELIMITER);
            if (temp < ((sizeof(TireText) / sizeof(TireText[0])))) {
              SDDataFile.print(TireText[temp]);
            } else {
              SDDataFile.print(F("UNKNOWN"));
            }
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Diameter"));
            SDDataFile.write(DATA_DELIMITER);
            if (temp < ((sizeof(TireRadius) / sizeof(TireRadius[0])))) {
              SDDataFile.print(TireRadius[temp] * 2.0, 3);
            } else {
              SDDataFile.print(F("UNKNOWN"));
            }
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Pressure"));
            SDDataFile.write(DATA_DELIMITER);
            if (temp < ((sizeof(TirePSIVal) / sizeof(TirePSIVal[0])))) {
              SDDataFile.print(TirePSIVal[temp]);
            } else {
              SDDataFile.print(F("UNKNOWN"));
            }
            break;
          case 6:
            // motor ID
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Motor"));
            temp = SSD.getHeaderField(MotorID, hrMotorID);
            if (temp < ((sizeof(MotorText) / sizeof(MotorText[0])))) {
              SDDataFile.write(DATA_DELIMITER);
              SDDataFile.print(MotorText[temp]);
            }
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Battery 1"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(Battery1, hrBattery1));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Battery 2"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(Battery2, hrBattery2));
            break;
          case 7:
            SDDataFile.write(DATA_DELIMITER);
            tWS = SSD.getHeaderField(WheelSprocket, hrWSprocket);
            tMS = SSD.getHeaderField(MotorSprocket, hrMSprocket);
            SDDataFile.print(F("Sprocket Ratio"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(((float)tWS / (float)tMS), 3);
            SDDataFile.print(F(" ("));
            SDDataFile.print(tMS);
            SDDataFile.print(F(" - "));
            SDDataFile.print(tWS);
            SDDataFile.print(F(")"));
            break;
          case 9:
            SDDataFile.print(F("RACE CONDITIONS"));
            break;
          case 10:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Ambient Temp"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(iTemp, hrTemp));
            break;
          case 11:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Atmospheric Pressure (pa/Hg)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            iPressure = SSD.getHeaderField(iPressure, hrPressure);
            SDDataFile.print(iPressure, 0);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(iPressure * 0.0002953f, 2);
            break;
          case 12:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Relative Humidity"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(Humidity, hrHumidity));
            break;
          case 13:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Start Altitude"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(StartAltitude, hrAltitude));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Start GPS Lon"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(GPSStartLon, hrLon), 6);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Start GPS Lat"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(GPSStartLat, hrLat), 6);
            break;
          case 15:
            SDDataFile.print(F("PERFORMANCE STATISTICS"));
            break;
          case 16:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Available Energy"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(TotalEnergy, hrEnergy));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Pit 1 Time [s]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=COUNTIF(Y2:Y13000, "));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F("Pit 1"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(") / 2"));
            SDDataFile.write(34);
            break;
          case 17:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Energy Used"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(K2: K13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=AC18 / AC17 * 100"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Pit 2 Time [s]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=COUNTIF(Y2:Y13000, "));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F("Pit 2"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(") / 2"));
            SDDataFile.write(34);
            break;
          case 18:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Laps"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(C2: C13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Pit X Time [s]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=COUNTIF(Y2:Y13000, "));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F("Pit X"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(") / 2"));
            SDDataFile.write(34);
            break;
          case 19:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Distance"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(N2: N13000)"));
            break;
          case 20:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("GPUSA Laps"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("(Enter Data)"));
            break;
          case 21:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Lap Distance"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("(Enter Data)"));
            break;
          case 22:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Actual Distance"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= AC21 * AC22"));
            break;
          case 23:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Max V"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(F2:F13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Min V"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(F2:F13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Avg V"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", F2:F13000)"));
            SDDataFile.write(34);
            break;
          case 24:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Max A"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(G2: G13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Min A"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MIN(G2: G13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Avg A"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= AVERAGE(G2:G13000)"));
            break;
          case 25:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Motor Temp Max"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(H2: H13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Motor Temp Min"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(H2:H13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Motor Temp Avg"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", H2:H13000)"));
            SDDataFile.write(34);
            break;
          case 26:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Aux Temp Max"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(I2: I13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Aux Temp Min"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(I2:I13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Aux Temp Avg"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", I2:I13000)"));
            SDDataFile.write(34);
            break;
          case 27:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("RPM Max"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(L2: L13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("RPM Min"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(L2:L13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("RPM Avg"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", L2:L13000)"));
            SDDataFile.write(34);
            break;
          case 28:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Speed Max"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(M2: M13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Speed Min"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(M2:M13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Speed Avg"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", M2:M13000)"));
            SDDataFile.write(34);
            break;
          case 29:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Max Altitude"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(Q2: Q13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Min Altitude"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MIN(Q2: Q13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Altitude Change"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=AC30 - AE30"));
            break;
          case 31:
            SDDataFile.print(F("DRIVER STATISTICS"));
            break;

          case 32:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Driver,1,2,3"));  // driver 1 ID is actually 0, but to make easier for the data analysist driver 1 = 1
            break;
          case 33:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Name"));
            // driver 0
            temp = (uint8_t)SSD.getHeaderField(DriverID[0], hrD0ID);
            if (temp < ((sizeof(DriverNames) / sizeof(DriverNames[0])))) {
              SDDataFile.write(DATA_DELIMITER);
              SDDataFile.print(DriverNames[temp]);
            } else {
              SDDataFile.write(DATA_DELIMITER);
              SDDataFile.print(F("UNKNOWN"));
            }
            // driver 1

            temp = (uint8_t)SSD.getHeaderField(DriverID[1], hrD1ID);
            if (temp < ((sizeof(DriverNames) / sizeof(DriverNames[1])))) {
              SDDataFile.write(DATA_DELIMITER);
              SDDataFile.print(DriverNames[temp]);
            } else {
              SDDataFile.write(DATA_DELIMITER);
              SDDataFile.print(F("UNKNOWN"));
            }
            // driver 2
            temp = (uint8_t)SSD.getHeaderField(DriverID[2], hrD2ID);
            if (temp < ((sizeof(DriverNames) / sizeof(DriverNames[2])))) {
              SDDataFile.write(DATA_DELIMITER);
              SDDataFile.print(DriverNames[temp]);
            } else {
              SDDataFile.write(DATA_DELIMITER);
              SDDataFile.print(F("UNKNOWN"));
            }
            break;
          case 34:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Time"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                 // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=COUNTIF(D2:D13000,AC33)/120"));  // driver time
            SDDataFile.write(34);                                 // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                 // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=COUNTIF(D2:D13000,AD33)/120"));  // driver time
            SDDataFile.write(34);                                 // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                 // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=COUNTIF(D2:D13000,AE33)/120"));  // driver time
            SDDataFile.write(34);                                 // write end " to keep comma in formula
            break;
          case 35:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Energy"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                        // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(K2:K13000, D2:D13000, AC33)"));  // driver time //MAXIFS(A2:A7,B2:B7,1)
            SDDataFile.write(34);                                        // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                             // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(K2:K13000, D2:D13000, AD33)-AC36"));  // driver time
            SDDataFile.write(34);                                             // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                                  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(K2:K13000, D2:D13000, AE33)-AC36-AD36"));  // driver time
            SDDataFile.write(34);                                                  // write end " to keep comma in formula
            break;
          case 36:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Laps"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                        // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(C2:C13000, D2:D13000, AC33)"));  // driver time
            SDDataFile.write(34);                                        // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                             // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(C2:C13000, D2:D13000, AD33)-AC37"));  // driver time
            SDDataFile.write(34);                                             // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                                  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(C2:C13000, D2:D13000, AE33)-AC37-AD37"));  // driver time
            SDDataFile.write(34);                                                  // write end " to keep comma in formula
            break;
          case 37:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Distance"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                        // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(N2:N13000, D2:D13000, AC33)"));  // driver time
            SDDataFile.write(34);                                        // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                             // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(N2:N13000, D2:D13000, AD33)-AC38"));  // driver time
            SDDataFile.write(34);                                             // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                                  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(N2:N13000, D2:D13000, AE33)-AC38-AD38"));  // driver time
            SDDataFile.write(34);                                                  // write end " to keep comma in formula
            break;
          case 38:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Watts / Mile"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AC37>0,AC36/AC38,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AD37>0,AD36/AD38,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AE37>0,AE36/AE38,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            break;
          case 39:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Watts / Minute"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AC35>0,AC36/AC35,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AD35>0,AD36/AD35,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AE35>0,AE36/AE35,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            break;
          case 41:
            SDDataFile.print(F("G-FORCE MEASUREMENTS"));
            break;
          case 42:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("X"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Y"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Z"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Total"));
            break;
          case 43:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Max"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(R2:R13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(S2:S13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(T2:T13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(U2:U13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            break;
          case 44:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Min"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(R2:R13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(S2:S13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(T2:T13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(U2:U13000, Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            break;
          case 45:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Average"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", R2:R13000)"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", S2:S13000)"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", T2:T13000)"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(Y2:Y13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", U2:U13000)"));
            SDDataFile.write(34);
            break;
          case 47:
            SDDataFile.print(F("DATABASE STATISTICS"));
            break;
          case 48:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Code Version"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(CODE_VERSION);
            break;
          case 49:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Utilities Version"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(UTILITIES_VERSION);
            break;
          case 50:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("BulletDB Version"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(BULLET_DB_VER);
            break;
          case 51:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Race Time"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(B2:B13000)"));
            break;
          case 52:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Points Recorded"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=COUNTA(A2:A13000)"));
            break;
          case 53:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Datapoint Averages"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(AverageCounter, hrCounter));
            break;
          case 54:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Telemetry Downtime [s]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=COUNTIF(Y2:Y13000, "));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F("Restored"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")/2"));
            SDDataFile.write(34);
            break;
        }

        // return record back to it's original
        SSD.gotoRecord(i);
      }

      SDDataFile.println("");
    }
  }

  //Serial.println("End of read, closing SD ");
  SDDataFile.close();
}

void DownloadGPSData(uint32_t Count) {

  bool SDCardStatus = false;
  uint8_t oLapCount = 255;
  uint8_t temp = 0, next = 0, rt = 0;
  uint32_t i = 0, rw = 0;
  char FileName[31] = "C_RRR_YYYY-MM-DD_NNN_GPS.csv";
  bool OKtoClose = false;

  if (Count == 1) {
    fc = 0;
  }

  if (digitalRead(CD_PIN) == HIGH) {
    Display.fillRect(184, 138, 134, 25, C_WHITE);
    Display.print(F("NO SD CARD"));
    return;
  }

  SDCardStatus = SDCARD.begin(SDCS_PIN, SD_SCK_MHZ(SD_SPI_SPEED));  //SD

  if (!SDCardStatus) {
    Display.fillRect(184, 138, 134, 25, C_WHITE);
    Display.print(F("NO SD CARD"));
    return;
  }
  SSD.gotoRecord(1);
  LastRecord = SSD.getLastRecord();

  for (i = 1; i <= LastRecord; i++) {

    SSD.gotoRecord(i);
    rt = SSD.getField(RecordType, frType);

    // reset the pit counter

    if (rt == NULL_RECORD) {
      //Serial.println("force close");
      SDDataFile.close();

      return;
    }

    // advance progress indicator
    fc++;
    rw = ((float)(fc * 293.0) / (Count * LastRecord)) + 2;
    Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

    if (rt == RT_HEADER) {

      // reset the pit counter
      if (OKtoClose) {
        SDDataFile.close();
        OKtoClose = false;
      }

      GPSStartLon = SSD.getHeaderField(GPSStartLon, hrLon);
      GPSStartLat = SSD.getHeaderField(GPSStartLat, hrLat);

      sprintf(FileName, "C_%03d_%04d-%02d-%02d_000_GPS.csv",
              SSD.getHeaderField(RecordSETID, hrID),
              SSD.getHeaderField(Tyear, hrYear),
              SSD.getHeaderField(Tmonth, hrMonth),
              SSD.getHeaderField(Tday, hrDay));

      temp = SSD.getHeaderField(CarID, hrCarID);

      if (temp == BLUE_CAR) {
        FileName[0] = 'B';
      } else if (temp == RED_CAR) {
        FileName[0] = 'R';
      } else {
        FileName[0] = 'W';
      }

      Display.fillRect(10, 160, 285, 20, C_WHITE);
      Display.setCursor(10, 160);
      Display.print(FileName);
      Display.setTextColor(C_BLACK, C_WHITE);

      while (SDCARD.exists(FileName)) {
        next++;

        FileName[17] = (int)((next / 100) % 10) + '0';
        FileName[18] = (int)((next / 10) % 10) + '0';
        FileName[19] = (int)(next % 10) + '0';

        Display.fillRect(10, 160, 285, 20, C_WHITE);
        Display.setTextColor(C_BLACK, C_WHITE);
        Display.setCursor(10, 160);
        Display.print(FileName);

        if (next > 999) {
          break;
        }
      }

      SDCardStatus = SDDataFile.open(FileName, O_WRITE | O_CREAT);

      if (!SDCardStatus) {
        // Serial.println("NoSd card");
        Display.setTextColor(C_RED, C_WHITE);
        Display.setCursor(200, 160);
        Display.print(F("No SD Card"));
        delay(1000);
        return;
      }

      // to do write file time stamp stuff
      SDDataFile.timestamp(T_CREATE,
                           (int)SSD.getHeaderField(Tyear, hrYear),
                           (int)SSD.getHeaderField(Tmonth, hrMonth),
                           (int)SSD.getHeaderField(Tday, hrDay),
                           (int)SSD.getHeaderField(Thour, hrHour),
                           (int)SSD.getHeaderField(Tminute, hrMinute),
                           0);

      SDDataFile.timestamp(T_WRITE,
                           (int)year(),
                           (int)month(),
                           (int)day(),
                           (int)hour(),
                           (int)minute(),
                           0);

      SDDataFile.timestamp(T_ACCESS,
                           (int)year(),
                           (int)month(),
                           (int)day(),
                           (int)hour(),
                           (int)minute(),
                           0);

      // print fields
      // Serial.println("print fields");
      SDDataFile.print(F("Volts"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Amps"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Power"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("RPM"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Speed"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Time"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Lon"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Lat"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Altitude"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Gps Speed"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel X"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel Y"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel Z"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("G-Force"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Lap"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("new_track"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Plot data: "));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("https://www.gpsvisualizer.com/map_input?form=leaflet"));

      SDDataFile.println("");
    }

    if (rt == RT_DATA) {

      OKtoClose = true;

      SDDataFile.print(SSD.getField(Volts, frVolts), 6);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Amps, frAmps), 3);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Amps, frAmps) * SSD.getField(Volts, frVolts), 0);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(mRPM, frRPM));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(CarSpeed, frSpeed), 2);

      RealClockTime = SSD.getField(RealClockTime, frRT);
      h = (int)(RealClockTime / 3600);
      m = (int)((RealClockTime - (h * 3600)) / 60);
      s = (int)(RealClockTime % 60);
      sprintf(str, "%02d:%02d:%02d", h, m, s);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(str);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(GPSLon, frLon), 7);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(GPSLat, frLat), 7);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Altitude, frAlt) / METERS_TO_FEET, 1);  // very weird but GPS plotting software thinks this is in meters
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(GPSSpeed, frGSpeed), 1);
      SDDataFile.write(DATA_DELIMITER);

      GForceX = SSD.getField(GForceX, frMax);
      GForceY = SSD.getField(GForceY, frMay);
      GForceZ = SSD.getField(GForceZ, frMaz);

      SDDataFile.print(GForceX, 4);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(GForceY, 4);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(GForceZ, 4);
      SDDataFile.write(DATA_DELIMITER);

      // compute and print the absolute max g force.
      SDDataFile.print(sqrt((GForceX * GForceX) + (GForceY * GForceY) + (GForceZ * GForceZ)), 4);

      SDDataFile.write(DATA_DELIMITER);

      LapCount = SSD.getField(LapCount, frLap);
      SDDataFile.print(LapCount);

      // special setting in the gps plotting software that will have each lap as a track
      if (LapCount != oLapCount) {
        oLapCount = LapCount;
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(1);
      }

      SDDataFile.println("");
    }
  }

  // now that the data has been written, write the Start location as a lap, that way we can verify where it was

  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.print(GPSStartLon, 7);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.print(GPSStartLat, 7);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.print(LapCount + 1);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.println(1);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.print(GPSStartLon, 7);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.print(GPSStartLat, 7);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.print(LapCount + 1);
  SDDataFile.write(DATA_DELIMITER);
  SDDataFile.println("");

  SDDataFile.close();
}

void DownloadEEPROM() {

  char SetupFileName[28] = "C_EEPROM_YYYY-MM-DD_NNN.txt";
  uint8_t next = 0;
  int rw = 0;
  bool SDCardStatus = false;
  uint32_t StartRecord = 0;
  uint32_t NextRecord = 0;
  uint32_t FirstID = 0;

  if (digitalRead(CD_PIN) == HIGH) {
    Display.setTextColor(C_RED, C_WHITE);
    Display.fillRect(184, 138, 134, 25, C_WHITE);
    Display.setCursor(200, 135);
    Display.print(F("NO SD CARD"));
    return;
  }

  SDCardStatus = SDCARD.begin(SDCS_PIN, SD_SCK_MHZ(SD_SPI_SPEED));  //SD

  if (!SDCardStatus) {
    Display.setTextColor(C_RED, C_WHITE);
    Display.fillRect(184, 138, 134, 25, C_WHITE);
    Display.setCursor(200, 135);
    Display.print(F("NO SD CARD"));
    return;
  }

  rw = ((float)(1 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

  sprintf(SetupFileName, "X_EEPROM_%04d-%02d-%02d_000.txt", year(), month(), day());

  if (CarID == BLUE_CAR) {
    SetupFileName[0] = 'B';
  } else if (CarID == RED_CAR) {
    SetupFileName[0] = 'R';
  } else if (CarID == WHITE_CAR) {
    SetupFileName[0] = 'W';
  }

  while (SDCARD.exists(SetupFileName)) {
    next++;

    SetupFileName[20] = (int)((next / 100) % 10) + '0';
    SetupFileName[21] = (int)((next / 10) % 10) + '0';
    SetupFileName[22] = (int)(next % 10) + '0';

    Display.fillRect(10, 160, 285, 20, C_WHITE);
    Display.setTextColor(C_BLACK, C_WHITE);
    Display.setCursor(10, 160);
    Display.print(SetupFileName);

    if (next > 999) {
      return;
    }
  }
  rw = ((float)(2 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

  SDCardStatus = SDDataFile.open(SetupFileName, O_WRITE | O_CREAT);

  delay(100);

  if (!SDCardStatus) {
#ifdef DO_DEBUG
    Serial.println(F("Write Setup data file FAIL"));
#endif
    return;  // don't even try to write anything
  }

  // to do write file time stamp stuff
  SDDataFile.timestamp(T_CREATE,
                       (int)SSD.getHeaderField(Tyear, hrYear),
                       (int)SSD.getHeaderField(Tmonth, hrMonth),
                       (int)SSD.getHeaderField(Tday, hrDay),
                       (int)SSD.getHeaderField(Thour, hrHour),
                       (int)SSD.getHeaderField(Tminute, hrMinute),
                       0);

  SDDataFile.timestamp(T_WRITE,
                       (int)year(),
                       (int)month(),
                       (int)day(),
                       (int)hour(),
                       (int)minute(),
                       0);

  SDDataFile.timestamp(T_ACCESS,
                       (int)year(),
                       (int)month(),
                       (int)day(),
                       (int)hour(),
                       (int)minute(),
                       0);

  rw = ((float)(3 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

  SDDataFile.println(F("CAR PARAMETERS"));
  SDDataFile.println();

  if (CarID < ((sizeof(CarText) / sizeof(CarText[0])))) {
    SDDataFile.print(F("Car ID: "));
    SDDataFile.print(CarID);
    SDDataFile.print(F(", "));
    SDDataFile.println(CarText[CarID]);
  } else {
    SDDataFile.println(F("Car: UNKNOWN"));
  }
  if (hour() > 12) {
    sprintf(str, "Report date: %d:%02d:%02d, %d/%d/%d", hour() - 12, minute(), second(), month(), day(), year());
  } else {
    sprintf(str, "Report date: %d:%02d:%02d, %d/%d/%d", hour(), minute(), second(), month(), day(), year());
  }
  SDDataFile.println(str);
  SDDataFile.println();

  rw = ((float)(4 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

  SDDataFile.println(F("Hardware Information"));
  SDDataFile.print(F("Memory chip JEDEC: "));
  SDDataFile.println(SSD.getChipJEDEC());
  SDDataFile.print(F("Total Records: "));
  SDDataFile.println(SSD.getLastRecord());
  SDDataFile.print(F("Total Fields: "));
  SDDataFile.println(SSD.getFieldCount());
  SDDataFile.print(F("Field length: "));
  SDDataFile.println(SSD.getRecordLength());
  SDDataFile.print(F("Header field length: "));
  SDDataFile.println(SSD.getHeaderRecordLength());
  SDDataFile.print(F("SSD Size: "));
  SDDataFile.println(SSD.getTotalSpace());
  SDDataFile.print(F("SSD Used: "));
  SDDataFile.println(SSD.getUsedSpace());

  SSD.gotoRecord(1);
  FirstID = SSD.getField(RecordSETID, hrID);

  SDDataFile.print(F("Total recordsets: "));
  SDDataFile.println(RecordSETID - FirstID + 1);
  StartRecord = 0;

  Tmonth = SSD.getHeaderField(Tmonth, hrMonth);
  Tday = SSD.getHeaderField(Tday, hrDay);
  Tyear = SSD.getHeaderField(Tyear, hrYear);
  Thour = SSD.getHeaderField(Thour, hrHour);
  Tminute = SSD.getHeaderField(Tminute, hrMinute);
  sprintf(str, "Date: %d/%d/%d, Time: %d:%02d", Tmonth, Tday, Tyear, Thour, Tminute);

  for (i = FirstID + 1; i <= RecordSETID; i++) {
    NextRecord = SSD.getFirstRecord(i, hrID);
    SSD.gotoRecord(NextRecord);

    SDDataFile.print(F("Record Set: "));
    SDDataFile.print(i - 1);
    SDDataFile.print(F(", Records: "));
    SDDataFile.print(NextRecord - StartRecord);
    StartRecord = NextRecord;
    SDDataFile.print(F(", "));
    SDDataFile.println(str);

    Tmonth = SSD.getHeaderField(Tmonth, hrMonth);
    Tday = SSD.getHeaderField(Tday, hrDay);
    Tyear = SSD.getHeaderField(Tyear, hrYear);
    Thour = SSD.getHeaderField(Thour, hrHour);
    Tminute = SSD.getHeaderField(Tminute, hrMinute);
    sprintf(str, "Date: %d/%d/%d, Time: %d:%02d", Tmonth, Tday, Tyear, Thour, Tminute);
  }
  SDDataFile.print(F("Record Set: "));
  SDDataFile.print(RecordSETID);
  SDDataFile.print(F(", Records: "));
  SDDataFile.print(SSD.getLastRecord() - StartRecord);
  SDDataFile.print(F(", "));
  SDDataFile.println(str);

  SDDataFile.print(F("Car display has black background: "));
  SDDataFile.println(Invert ? "NO" : "Yes");
  SDDataFile.print(F("Car display orientation: "));
  SDDataFile.println(Orientation ? "Buttons up" : "Buttons down");
  SDDataFile.print(F("Update (ms): "));
  SDDataFile.println(UPDATE_LIMIT);
  SDDataFile.println();

  rw = ((float)(5 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

  SDDataFile.println(F("Software versions"));
  SDDataFile.print(F("Code : "));
  SDDataFile.println(CODE_VERSION);
  SDDataFile.print(F("Utilities :"));
  SDDataFile.println(UTILITIES_VERSION);
  SDDataFile.print(F("BulletDB : "));
  SDDataFile.println(BULLET_DB_VER);
  SDDataFile.print(F("Menu : "));
  SDDataFile.println(ILI9341_MENU_VER);
  SDDataFile.print(F("Controls : "));
  SDDataFile.println(ILI9341_t3_CONTROLS_VER);
  SDDataFile.print(F("FlickerFree : "));
  SDDataFile.println(FLICKER_FREE_PRINT_VER);
  SDDataFile.print(F("EBYTE version: "));
  SDDataFile.println(EBYTE_H_LIB_VER);
  SDDataFile.println();

  rw = ((float)(6 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

  SDDataFile.println(F("Calibration information"));
  SDDataFile.print(F("Voltage Slope: "));
  SDDataFile.println(VoltageSlope, 3);

  SDDataFile.print(F("Voltage Offset: "));
  SDDataFile.println(VoltageOffset, 3);
  SDDataFile.print(F("Calibrate voltage Offset at startup: "));
  SDDataFile.println(AutoCurrentCal ? "Yes" : "No");

  SDDataFile.print(F("V@0 amp: "));
  SDDataFile.println(VMid, 3);
  SDDataFile.print(F("mV/Amp: "));
  SDDataFile.println(mVPerAmp, 3);
  SDDataFile.print(F("Pickups: "));
  SDDataFile.println(Pickups);
  SDDataFile.print(F("Min Pulses: "));
  SDDataFile.println(MinPulses);
  SDDataFile.print(F("Temp resistor Motor: "));
  SDDataFile.println(ThermResMotor);
  SDDataFile.print(F("Temp resistor Aux: "));
  SDDataFile.println(ThermResAux);
  SDDataFile.print(F("Ambient temp offset: "));
  SDDataFile.println(AmbTempCF);

  SDDataFile.print(F("Enable Accelerometer: "));
  SDDataFile.println(HasASensor ? "Yes" : "No");
  SDDataFile.print(F("Accel Fullscale: "));
  if (GForceRange < ((sizeof(AccelFSRange) / sizeof(AccelFSRange[0])))) {
    SDDataFile.println(AccelFSRange[(int)GForceRange]);
  } else {
    SDDataFile.println(F("Bad GForce Range"));
  }

  SDDataFile.print(F("Accel X Offset: "));
  SDDataFile.println(AccelCalX);
  SDDataFile.print(F("Accel Y Offset: "));
  SDDataFile.println(AccelCalY);
  SDDataFile.print(F("Accel Z Offset: "));
  SDDataFile.println(AccelCalZ);

  if (AccelLPFilter < ((sizeof(AccelLPFilterText) / sizeof(AccelLPFilterText[0])))) {
    SDDataFile.print(F("Accelerometer LP Filter: "));
    SDDataFile.println(AccelLPFilterText[(uint8_t)AccelLPFilter]);
  } else {
    SDDataFile.println(F("Bad Accelerometer LP filter data"));
  }

  if (AccelHPFilter < ((sizeof(AccelHPFilterText) / sizeof(AccelHPFilterText[0])))) {
    SDDataFile.print(F("Accelerometer HP Filter: "));
    SDDataFile.println(AccelHPFilterText[(uint8_t)AccelHPFilter]);
  } else {
    SDDataFile.println(F("Bad Accelerometer HP filter data"));
  }

  SDDataFile.print(F("Restart Display: "));
  if (RestartDisplayAlways) {
    SDDataFile.println(F("On each page draw"));
  } else {
    SDDataFile.println(F("On screen change"));
  }

  SDDataFile.println();

  rw = ((float)(7 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

  SDDataFile.println(F("Mechanical information"));
  // motor ID
  SDDataFile.print(F("MotorID: "));
  SDDataFile.print(MotorID);
  if (MotorID < ((sizeof(MotorText) / sizeof(MotorText[0])))) {
    SDDataFile.print(F(", Motor: "));
    SDDataFile.print(MotorText[(int)MotorID]);
  }
  SDDataFile.println();
  SDDataFile.print(F("Gear Ratio: "));
  SDDataFile.println(GearRatio, 3);
  SDDataFile.print(F("Motor Sprocket: "));
  SDDataFile.println(MotorSprocket);
  SDDataFile.print(F("Wheel Sprocket: "));
  SDDataFile.println(WheelSprocket);

  SDDataFile.print(F("Tire ID: "));
  SDDataFile.print(TireID);
  if (TireID < ((sizeof(TireText) / sizeof(TireText[0])))) {
    SDDataFile.print(F(", Tire: "));
    SDDataFile.println(TireText[TireID]);
    SDDataFile.print(F("Pressure: "));
    SDDataFile.println(TirePSIVal[TireID]);
    SDDataFile.print(F("Radius: "));
    SDDataFile.println(TireRadius[TireID], 3);
  } else {
    SDDataFile.print(F("Car: UNKNOWN"));
  }
  SDDataFile.println();

  rw = ((float)(8 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

  SDDataFile.println(F("Driver information"));
  if (DriverID[0] < ((sizeof(DriverNames) / sizeof(DriverNames[0])))) {
    SDDataFile.print(F("Driver 1: ID "));
    SDDataFile.print(DriverID[0]);
    SDDataFile.print(F(", "));
    SDDataFile.println(DriverNames[DriverID[0]]);
  } else {
    SDDataFile.println(F("Driver 0: UNKNOWN"));
  }
  if (DriverID[1] < ((sizeof(DriverNames) / sizeof(DriverNames[0])))) {
    SDDataFile.print(F("Driver 2: ID "));
    SDDataFile.print(DriverID[1]);
    SDDataFile.print(F(", "));
    SDDataFile.println(DriverNames[DriverID[1]]);
  } else {
    SDDataFile.println(F("Driver 1: UNKNOWN"));
  }
  if (DriverID[2] < ((sizeof(DriverNames) / sizeof(DriverNames[0])))) {
    SDDataFile.print(F("Driver 3: ID "));
    SDDataFile.print(DriverID[2]);
    SDDataFile.print(F(", "));
    SDDataFile.println(DriverNames[DriverID[2]]);
  } else {
    SDDataFile.println(F("Driver 2: UNKNOWN"));
  }
  SDDataFile.println();

  rw = ((float)(9 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);

  SDDataFile.println(F("Race settings information"));
  SDDataFile.print(F("Total Energy: "));
  SDDataFile.println(TotalEnergy);
  SDDataFile.print(F("Battery 1: "));
  SDDataFile.println(Battery1);
  SDDataFile.print(F("Battery 2: "));
  SDDataFile.println(Battery2);
  SDDataFile.print(F("Battery Warning (volts): "));
  SDDataFile.println(BatWarning);
  SDDataFile.print(F("Temp Warning (deg F): "));
  SDDataFile.println(TempWarning);

  SDDataFile.println();

  SDDataFile.println(F("Wireless information"));

  sprintf(str, "0x%02x", Radio.GetModel());
  SDDataFile.print(F("Transceiver Model: "));
  SDDataFile.println(str);
  SDDataFile.print(F("Transceiver address: "));
  SDDataFile.println((Radio.GetAddressH() << 8) | (Radio.GetAddressL()));
  SDDataFile.print(F("Transceiver air data rate: "));
  SDDataFile.println(Radio.GetAirDataRate());
  SDDataFile.print(F("Transceiver channel: "));
  SDDataFile.println(Radio.GetChannel());
  SDDataFile.print(F("Transceiver options uint8_t (BIN): "));
  SDDataFile.println(Radio.GetOptions(), BIN);
  SDDataFile.print(F("Transceiver speed uint8_t (BIN): "));
  SDDataFile.println(Radio.GetSpeed(), BIN);
  SDDataFile.print(F("Send time [s]: "));
  SDDataFile.println(RadioUpdate);
  SDDataFile.print(F("Use GPS speed when displaying car speed: "));
  SDDataFile.println(UseGPSForSpeedDisplay ? "Yes" : "No");

  SDDataFile.println();

  SDDataFile.println(F("Lap trigger information"));
  SDDataFile.print(F("Lap trigger range (m): "));
  SDDataFile.println(GPSTolerance);
  SDDataFile.print(F("Start/Change trigger (amps): "));
  SDDataFile.println(TriggerAmps);
  SDDataFile.print(F("Reset altitude per lap: "));
  SDDataFile.println(ResetAltitude ? "Yes" : "No");

  SDDataFile.print(F("Add lap when pitting: "));
  SDDataFile.println(AddLapInPit ? "Yes" : "No");
  SDDataFile.print(F("Delay GPS read at start: "));
  SDDataFile.println(GPSReadTimeText[StartGPSDelay]);
  SDDataFile.print(F("Lap threashold (s): "));
  SDDataFile.println(LapThreashold);
  SDDataFile.print(F("Use same data on restart: "));
  SDDataFile.println(RestoreData ? "Yes" : "No");
  SDDataFile.println();

  SDDataFile.println(F("END CAR PARAMETERS"));

  SDDataFile.close();

  rw = ((float)(10 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 182, rw, 36, 2, C_GREEN);
  delay(100);
}

/*
  PURPOSE : Screen settings
  PARAMS: -
  RETURNS : None
  NOTES:
*/

void SetScreenParameters() {

  if (Invert == 0) {
    // black background
    back_color = C_BLACK;
    fore_color = C_WHITE;
  } else {
    // black background
    back_color = C_WHITE;
    fore_color = C_BLACK;
  }

  if (Orientation == 0) {
    Display.setRotation(1);
  } else if (Orientation == 1) {
    Display.setRotation(3);
  } else {
    Display.setRotation(1);  //default setting
  }

  RedrawHeader = true;
}



void BuildDateStringMS(unsigned long val) {

  val = val / 1000;
  h = (int)(val / 3600);
  m = (int)((val - (h * 3600)) / 60);
  s = (int)(val % 60);
  sprintf(str, "%02d:%02d:%02d", h, m, s);
}

void BuildDateStringS(unsigned long val) {

  h = (int)(val / 3600);
  m = (int)((val - (h * 3600)) / 60);
  s = (int)(val % 60);
  sprintf(str, "%02d:%02d:%02d", h, m, s);
}

void BuildFieldList() {

  // adding fields is fine, but you may need to adjust the limit the library is capped at
  // in BulletDB.h change the limit as needed #define MAX_FIELDS 25

  frType = SSD.addField("Type", &RecordType);
  frID = SSD.addField("Recordset ID", &RecordSETID);
  frPoint = SSD.addField("Point", &Point);
  frLap = SSD.addField("Lap Count", &LapCount);
  frDriver = SSD.addField("Driver", &Driver);
  frVolts = SSD.addField("Volts", &Volts);
  frAmps = SSD.addField("Amps", &Amps);
  frMotorTemp = SSD.addField("MotorTemp", &MotorTemp);
  frAuxTemp = SSD.addField("AuxTemp", &AuxTemp);
  frAmbTemp = SSD.addField("AmbTemp", &AmbTemp);
  frEnergy = SSD.addField("Energy", &Energy);
  frRPM = SSD.addField("MRPM", &mRPM);
  frSpeed = SSD.addField("Speed", &CarSpeed);
  frDist = SSD.addField("Distance", &Distance);
  frRT = SSD.addField("RealClockTime", &RealClockTime);
  frLon = SSD.addField("GPSLon", &GPSLon);
  frLat = SSD.addField("GPSLat", &GPSLat);
  frAlt = SSD.addField("Alt", &Altitude);
  frGSpeed = SSD.addField("GPSSpeed", &GPSSpeed);
  frMax = SSD.addField("Max X", &GForceX);
  frMay = SSD.addField("Max Y", &GForceY);
  frMaz = SSD.addField("Max Z", &GForceZ);
  frHumidity = SSD.addField("Humidity", &Humidity);
  frRestoreType = SSD.addField("RestoreType", &RestoreType);

  // you cannot have more header fields that data fields
  // header fields can be any type
  // headers are simply the first record and can hold setup data (start time for example) to be printed later
  hrType = SSD.addHeaderField("Record Type", &RecordType);
  hrID = SSD.addHeaderField("Recordset ID", &RecordSETID);
  hrYear = SSD.addHeaderField("Year", &Tyear);
  hrMonth = SSD.addHeaderField("Month", &Tmonth);
  hrDay = SSD.addHeaderField("Day", &Tday);
  hrHour = SSD.addHeaderField("Hour", &Thour);
  hrMinute = SSD.addHeaderField("Minute", &Tminute);
  hrMSprocket = SSD.addHeaderField("Motor Sprocket", &MotorSprocket);
  hrWSprocket = SSD.addHeaderField("Wheel Sprocket", &WheelSprocket);
  hrTireID = SSD.addHeaderField("Tire ID", &TireID);
  hrD0ID = SSD.addHeaderField("Driver 0 ID", &DriverID[0]);
  hrD1ID = SSD.addHeaderField("Driver 1 ID", &DriverID[1]);
  hrD2ID = SSD.addHeaderField("Driver 2 ID", &DriverID[2]);
  hrCarID = SSD.addHeaderField("Car ID", &CarID);
  hrMotorID = SSD.addHeaderField("Motor ID", &MotorID);
  hrTemp = SSD.addHeaderField("Amb Temp", &iTemp);
  hrPressure = SSD.addHeaderField("Amp Press", &iPressure);
  hrAltitude = SSD.addHeaderField("Start Alt", &StartAltitude);
  hrHumidity = SSD.addHeaderField("Amp RH", &Humidity);
  hrEnergy = SSD.addHeaderField("Energy", &TotalEnergy);
  hrCounter = SSD.addHeaderField("Counter", &AverageCounter);
  hrBattery1 = SSD.addHeaderField("Battery 1", &Battery1);
  hrBattery2 = SSD.addHeaderField("Battery 2", &Battery2);
  hrLon = SSD.addHeaderField("SGPSLon", &GPSStartLon);
  hrLat = SSD.addHeaderField("SGPSLat", &GPSStartLat);
}



// this is in a function to setup accelerometer parameters
// we may have to restart during a race or we may change settings in menu
// all this code needed after a restart or parameter change
void SetupAccelerometer() {

  ASensor.setZeroMotionDetectionThreshold(2);

  ASensor.setDLPFMode(AccelLPFilter);
  ASensor.setDHPFMode(AccelHPFilter);

  ASensor.setFullScaleGyroRange((MPU6050_IMU::MPU6050_GYRO_FS_1000));
  ASensor.setFullScaleAccelRange(GForceRange);

  ASensor.setXAccelOffset(AccelCalX);
  ASensor.setYAccelOffset(AccelCalY);
  ASensor.setZAccelOffset(AccelCalZ);

  ASensorBits = AccelFSBits[GForceRange];

  // GForceG.setYAxis(-pow(2, GForceRange + 1), pow(2, GForceRange + 1), pow(2, GForceRange + 1) / 4);

  //delay(10);
  //ax = ASensor.getAccelerationX() / ASensorBits;
  //ay = ASensor.getAccelerationY() / ASensorBits;
  //az = ASensor.getAccelerationZ() / ASensorBits;
  delay(50);
}

/*

function to setup most devices some (display and flash chip) must be started early in setup

*/
void InitializeSensors() {

  bool status = false;

  // init the accelerometer
  Display.setCursor(STATUS_RESULT, 40);

  ASensor.initialize();

  GForceStatus = false;
  // most reliable way to see if it's connected...
  // add this to the MPU6050 library

  if (ASensor.isConnected()) {
    GForceStatus = true;
  }

  Display.setTextColor(C_GREEN);
  if (!GForceStatus) {
    Display.setTextColor(C_RED);
    Display.print(F("Fail"));
    Warnings = Warnings | GFORCE_WARNING;
  }

  SetupAccelerometer();

  ax = ASensor.getAccelerationX() / ASensorBits;
  ay = ASensor.getAccelerationY() / ASensorBits;
  az = ASensor.getAccelerationZ() / ASensorBits;

  // GForceStatus = true;
  Display.print(ax, 2);
  Display.print(F(", "));
  Display.print(ay, 2);
  Display.print(F(", "));
  Display.println(az, 2);

  analogReadRes(12);
  analogReadAveraging(4);

  delay(1000);

  // get the battey voltage
  vVolts = analogRead(VM_PIN);
  vVolts = vVolts / (BIT_CONVERSION / 3.3f);
  Volts = (vVolts * VoltageSlope) + VoltageOffset;
  Display.setCursor(STATUS_RESULT, 60);
  if ((Volts < BatWarning) || (Volts > 30.0f)) {
    Display.setTextColor(C_RED);
    Display.print(Volts, 1);
  } else {
    Display.setTextColor(C_GREEN);
    Display.print(Volts, 1);
  }
  Display.print(F(" / "));
  Display.setTextColor(C_GREEN);

  // data logger may have been started when key is on
  // or we may need to reset race when key is on

  if ((AutoCurrentCal) && (RaceStatus == RACE_NOTSTARTED)) {

    aVolts = 0;
    for (i = 0; i < 2000; i++) {
      aVolts = aVolts + analogRead(AM_PIN);
      delay(2);
    }
    aVolts = aVolts / 2000.0f;
    aVolts = aVolts / (BIT_CONVERSION / 3.3f);

    if (aVolts < 0.4f) {
      Warnings = Warnings | AMP_WARNING;
    }

    // datalogger consumes 0.066 amps and since its current is not running through
    // current sensor, we're just compensate the offset and subtract 0.001 volts
    VMid = aVolts - 0.001;
    EEPROM.put(230, VMid);
    delay(50);
    // update the menu value
    SensorMenu.SetItemValue(SensorMenuOption4, VMid);
    Display.setTextColor(C_CYAN);
  }

  // get current draw
  aVolts = analogRead(AM_PIN);
  aVolts = aVolts / (BIT_CONVERSION / 3.3f);
  Amps = ((aVolts - VMid) * 1000.0f) / mVPerAmp;

  if ((Amps < -2.0f) || (Amps > 70.0f)) {
    Display.setTextColor(C_RED);
    Display.print(Amps, 3);
    Warnings = Warnings | AMP_WARNING;
  } else {
    Display.print(Amps, 3);
  }

  status = false;

  status = BMEsensor.begin(0x76);
  //BMEsensor.setSampling();
  delay(50);
  Display.setCursor(STATUS_RESULT, 80);

  if (!status) {
    Display.setTextColor(C_RED);
    Display.print(F("0 / 0 / 0"));
    AltimiterStatus = false;
    Warnings = Warnings | AMBIENT_FAIL;
  } else {
    AltimiterStatus = true;
    // get starting condidtions, save to SSD in the header field
    iTemp = (BMEsensor.readTemperature() * 1.8) + 32.0 + AmbTempCF;
    Altitude = BMEsensor.readAltitude(SEALEVELPRESSURE_HPA) * METERS_TO_FEET;
    iPressure = BMEsensor.readPressure();
    Humidity = BMEsensor.readHumidity();

    Display.setTextColor(C_GREEN);
    Display.print(Altitude, 0);
    Display.print(F("ft / "));
    Display.print(iTemp, 0);
    Display.print(F("f / "));
    Display.print(Humidity, 0);
    Display.print(F("%"));
  }

  // test Motor temp sensor
  thmVolts = (analogRead(THM_PIN));

  thmVolts = thmVolts / (BIT_CONVERSION / 3.3f);

  tr2 = (thmVolts * ThermResMotor) / (3.3f - thmVolts);
  TempK = 1.0 / (NTC_A + (NTC_B * (log(tr2 / 10000.0f))) + (NTC_C * pow(log(tr2 / 10000.0f), 2)) + (NTC_D * pow(log(tr2 / 10000.0f), 3)));
  MotorTemp = (TempK * 1.8f) - 459.67f;

  if ((MotorTemp < 10.0f) || (MotorTemp > TempWarning)) {
    Display.setTextColor(C_RED);
    Display.setCursor(STATUS_RESULT, 100);
    Display.print("NC");
    Warnings = Warnings | TEMP_WARNING;
  } else {
    Display.setTextColor(C_GREEN);
    Display.setCursor(STATUS_RESULT, 100);
    Display.print(MotorTemp, 0);
  }

  // test auxiliary temp sensor
  thxVolts = (analogRead(THX_PIN));

  thxVolts = thxVolts / (BIT_CONVERSION / 3.3f);

  tr2 = (thxVolts * ThermResAux) / (3.3f - thxVolts);
  TempK = 1.0 / (NTC_A + (NTC_B * (log(tr2 / 10000.0f))) + (NTC_C * pow(log(tr2 / 10000.0f), 2)) + (NTC_D * pow(log(tr2 / 10000.0f), 3)));
  AuxTemp = (TempK * 1.8f) - 459.67f;

  if ((AuxTemp < 10.0f) || (AuxTemp > TempWarning)) {
    Display.setTextColor(C_RED);
    Display.print(F(" / "));
    Display.print("NC");
    Warnings = Warnings | TEMP_WARNING;
  } else {
    Display.setTextColor(C_GREEN);
    Display.print(F(" / "));
    Display.print(AuxTemp, 0);
  }

  Display.print(F(" / "));
  Display.print(iTemp, 0);

  Display.setCursor(STATUS_RESULT, 120);

  // test GPS
  GPSUpdateTimer = 0;
  // alert pit that it's time to test GPS

  digitalWrite(GPSLED_PIN, HIGH);
  GPSStatus = false;
  while (GPSUpdateTimer < 5000) {  // test for 5 sec 5000
    GPSRead();
    GPSLat = GPS.location.lat();
    GPSLon = GPS.location.lng();
    GPSStatus = GPS.location.isValid();
    if (GPSStatus) {
      digitalWrite(GPSLED_PIN, LOW);
      break;
    }
  }

  if (!GPSStatus) {
    Display.setTextColor(C_RED);
    Display.print(F("0.000, 0.000"));
    Warnings = Warnings | GPS_WARNING;
  } else {
    Display.setTextColor(C_GREEN);
    Display.print(GPSLat, 3);
    Display.print(F(", "));
    Display.print(GPSLon, 3);
  }

  // Setup speed sensor
  RPMStatus = RPM.begin(RPM_PIN);

  Display.setCursor(STATUS_RESULT, 140);
  if (UseGPSForSpeedDisplay) {
    Display.setTextColor(C_CYAN);
    Display.print(F("From GPS"));
  } else {

    if (!RPMStatus) {
      Display.setTextColor(C_RED);
      Display.print(F("Sensor FAIL"));
      Warnings = Warnings | SPEED_FAIL;
    } else {
      Display.setTextColor(C_GREEN);
      Display.print(F("From Sensor"));
    }
  }

  // test transceiver
  delay(10);

  RadioStatus = Radio.init();

  /*
  Serial.print("DID not fire up ");Serial.println(RadioStatus);

  Radio.SetAddressH(0);
  Radio.SetAddressL(0);
  Radio.SetOptions(0b01000100);
  Radio.SetAirDataRate(0b100);
  Radio.SetUARTBaudRate(0b011);
  Radio.SetChannel(15);
  Radio.SaveParameters(PERMANENT);
  Radio.PrintParameters();
  Serial.println(Radio.GetModel(), HEX);
*/

  DataPacket.begin(details(Data), &ESerial);

  if ((!RadioStatus) || (Radio.GetModel() != 0x44)) {
    Display.setTextColor(C_RED);
    Display.setCursor(STATUS_RESULT, 160);
    Display.print(F("FAIL"));
    Warnings = Warnings | EBYTE_FAIL;
  } else {
    RadioChannel = Radio.GetChannel();
    AirDataRate = Radio.GetAirDataRate();

    Display.setTextColor(C_GREEN);
    Display.setCursor(STATUS_RESULT, 160);
    Display.print(F("Ch: "));
    Display.print(RadioChannel);
    Display.print(F(" / "));
    if (AirDataRate < ((sizeof(AirRateText) / sizeof(AirRateText[0])))) {
      Display.print(AirRateText[AirDataRate]);
    } else {
      Display.print(F("?"));
    }
  }

#ifdef DO_DEBUG
  Serial.println(F("******* EBYTE Parameters *******"));
  Radio.PrintParameters();
  Serial.println(F("******* End EBYTE Parameters *******"));
#endif
}

void StartRTC() {

  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);

  RTCTime = processSyncMessage();

  if (RTCTime != 0) {
    setTime(RTCTime);
    Teensy3Clock.set(RTCTime);  // set the RTC
  }

  // setTime(hours, minutes, seconds, days, months, years);
  // setTime(21, 41, 0, 18, 3, 2022);
  // Teensy3Clock.set(now());
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600;  // Jan 1 2013
  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();

    return pctime;
    if (pctime < DEFAULT_TIME) {  // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L;                // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}


/*
   PURPOSE : Gets the time and returns it
    PARAMS :  -
   RETURNS : time_t -
     NOTES : Returns time_t object
*/

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void WatchDogTimer(uint8_t state) {

  if (state == ENABLE_WDT) {
#ifdef DO_DEBUG
    Serial.println("enabling WDT");
#endif
    // Setup WDT
    noInterrupts();
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);
    // values will reset after 10 seconds (hex based on seconds x 7,200,000)
    WDOG_TOVALH = 0x044A;
    WDOG_TOVALL = 0xA200;
    WDOG_PRESC = 0x400;
    WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
    interrupts();
  } else if (state == DISABLE_WDT) {
#ifdef DO_DEBUG
    Serial.println("Disabling WDT");
#endif
    NVIC_DISABLE_IRQ(IRQ_WDOG);
    noInterrupts();                  // don't allow interrupts while setting up WDOG
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;  // unlock access to WDOG registers
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);  // Need to wait a bit..
    // Set options to enable WDT. You must always do this as a SINGLE write to WDOG_CTRLH
    WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;
    interrupts();
  } else {
    // reset WDT, must be called in any while loop
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
  }
}

void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  int16_t ax, ay, az, gx, gy, gz;


  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    ASensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) {  //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2);  //Needed so we don't get repeated measures
  }
}

void CalibrateAccererometer() {

  uint16_t rw = 0;
  uint8_t Found = 0, oFound = 0;
  uint8_t Tries = 0;
  ax_offset = -mean_ax / ASensorBits;
  ay_offset = -mean_ay / ASensorBits;
  az_offset = (16384 - mean_az) / ASensorBits;

  gx_offset = -mean_gx / ASensorBits;
  gy_offset = -mean_gy / ASensorBits;
  gz_offset = -mean_gz / ASensorBits;

  ASensor.setXAccelOffset(0);
  ASensor.setYAccelOffset(0);
  ASensor.setZAccelOffset(0);

  ASensor.setXGyroOffset(0);
  ASensor.setYGyroOffset(0);
  ASensor.setZGyroOffset(0);

  Display.setFont(FONT_16B);
  Display.fillRect(0, 160, 320, 100, C_DKGREY);

  Display.setTextColor(C_WHITE, C_DKGREY);
  Display.setCursor(13, 170);
  Display.print(F("Hold any button to exit."));
  Display.fillRoundRect(13, 195, 293, 36, 2, C_DKGREEN);

  while (1) {
    Found = 0;
    ASensor.setXAccelOffset(ax_offset);
    ASensor.setYAccelOffset(ay_offset);
    ASensor.setZAccelOffset(az_offset);

    ASensor.setXGyroOffset(gx_offset);
    ASensor.setYGyroOffset(gy_offset);
    ASensor.setZGyroOffset(gz_offset);

    meansensors();

    if (abs(mean_ax) <= acel_deadzone) Found++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) Found++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(ASensorBits - mean_az) <= acel_deadzone) Found++;
    else az_offset = az_offset + (ASensorBits - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) Found++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) Found++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) Found++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (Found < oFound) {
      Display.fillRect(0, 160, 320, 30, C_DKGREY);
      Display.setCursor(13, 170);
      Display.setTextColor(C_WHITE, C_DKGREY);
      Display.print(F("Please sit still. Restarting"));
      Tries = 0;
      Display.fillRoundRect(13, 195, 293, 36, 2, C_DKGREEN);
    }
    if (Found >= oFound) {
      Display.fillRect(0, 160, 320, 30, C_DKGREY);
      Display.setTextColor(C_WHITE, C_DKGREY);
      Display.setCursor(13, 170);
      Display.print(F("Hold any button to exit."));
    }

    oFound = Found;

    Tries++;
    rw = (float)((Tries / 50.0) * 293.0);
    Display.fillRoundRect(13, 195, rw, 36, 2, C_GREEN);
    if ((Found == 6) || (Tries >= 50) || (digitalRead(R_PIN) == LOW) || (digitalRead(L_PIN) == LOW)) break;
  }

  if (Found != 6) {
    Display.fillRect(0, 160, 320, 100, C_DKGREY);
    Display.setFont(FONT_16B);
    Display.setCursor(13, 170);
    Display.setTextColor(C_RED, C_DKGREY);
    Display.print(F("Auto-calibration failed."));
    Display.setCursor(13, 190);
    Display.print(F("Calibrate accelerometer"));
    Display.setCursor(13, 210);
    Display.print(F("manually. Press any button."));
    // if user bails out wait for button depress
    while ((digitalRead(R_PIN) == LOW) || (digitalRead(L_PIN) == LOW)) {
      delay(10);
    }

    // now look for button press
    while ((digitalRead(R_PIN) == HIGH) && (digitalRead(L_PIN) == HIGH)) {
      delay(10);
    }

  } else {
    Display.fillRect(0, 160, 320, 30, C_DKGREY);
    Display.setTextColor(C_WHITE, C_DKGREY);
    Display.setCursor(13, 170);
    Display.print(F("Calibration complete."));
    Display.fillRoundRect(13, 195, 293, 36, 2, C_GREEN);
    AccelCalX = ax_offset;
    AccelCalY = ay_offset;
    AccelCalZ = az_offset;
    delay(5000);
  }
}

void WaitForExit() {

  uint8_t WaitTime = 10;
  uint32_t Start = millis();

  Display.setTextColor(C_WHITE);
  Display.setCursor(40, 160);
  Display.print("PRESS ANY BUTTON");
  Display.setCursor(40, 180);
  Display.print("OR WAIT");
  Display.setCursor(160, 180);
  Display.fillRect(160, 180, 40, 30, C_RED);
  Display.print(WaitTime--);

  GPSLapTimer = 0;
  while ((digitalRead(R_PIN) == HIGH) && (digitalRead(L_PIN) == HIGH)) {

    if (millis() - Start > 1000) {
      Start = millis();
      Display.setCursor(160, 180);
      Display.fillRect(160, 180, 40, 30, C_RED);
      Display.print(WaitTime--);
    }

    if (GPSLapTimer > 10000) {
      break;
    }

    WatchDogTimer(RESET_WDT);
    delay(10);
  }

  while ((digitalRead(R_PIN) == LOW) || (digitalRead(L_PIN) == LOW)) {
    delay(10);
  }
}

void DisplayErrors() {

  if (Warnings & TEMP_WARNING) {
    Display.fillRect(26, 26, 268, 188, C_WHITE);
    Display.fillRect(30, 30, 260, 180, C_RED);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE);
    Display.setCursor(40, 40);
    Display.print("FATAL ERROR");
    Display.setFont(FONT_16B);
    Display.setCursor(40, 80);
    Display.print("CHECK THE");
    Display.setCursor(40, 110);
    Display.print("TEMP SENSOR");

    WaitForExit();
  }

  if (Warnings & AMP_WARNING) {
    Display.fillRect(26, 26, 268, 188, C_WHITE);
    Display.fillRect(30, 30, 260, 180, C_RED);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE);
    Display.setCursor(40, 40);
    Display.print("FATAL ERROR");
    Display.setFont(FONT_16B);

    Display.setCursor(40, 80);
    Display.print("CHECK THE");
    Display.setCursor(40, 110);
    Display.print("CURRENT SENSOR");

    WaitForExit();
  }
  if (Warnings & GFORCE_WARNING) {
    Display.fillRect(26, 26, 268, 188, C_WHITE);
    Display.fillRect(30, 30, 260, 180, C_RED);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE);
    Display.setCursor(40, 40);
    Display.print("FATAL ERROR");
    Display.setFont(FONT_16B);

    Display.setCursor(40, 80);
    Display.print("CHECK THE");
    Display.setCursor(40, 110);
    Display.print("ACCELEROMETER");

    WaitForExit();
  }
  if (Warnings & GPS_WARNING) {
    Display.fillRect(26, 26, 268, 188, C_WHITE);
    Display.fillRect(30, 30, 260, 180, C_RED);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE);
    Display.setCursor(40, 40);
    Display.print("FATAL ERROR");
    Display.setFont(FONT_16B);

    Display.setCursor(40, 80);
    Display.print("CHECK THE");
    Display.setCursor(40, 110);
    Display.print("GPS SENSOR");

    WaitForExit();
  }
  if (Warnings & SPEED_FAIL) {
    Display.fillRect(26, 26, 268, 188, C_WHITE);
    Display.fillRect(30, 30, 260, 180, C_RED);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE);
    Display.setCursor(40, 40);
    Display.print("FATAL ERROR");
    Display.setFont(FONT_16B);

    Display.setCursor(40, 80);
    Display.print("CHECK THE");
    Display.setCursor(40, 110);
    Display.print("SPEED SENSOR");

    WaitForExit();
  }
  if (Warnings & AMBIENT_FAIL) {
    Display.fillRect(26, 26, 268, 188, C_WHITE);
    Display.fillRect(30, 30, 260, 180, C_RED);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE);
    Display.setCursor(40, 40);
    Display.print("FATAL ERROR");
    Display.setFont(FONT_16B);

    Display.setCursor(40, 80);
    Display.print("CHECK THE");
    Display.setCursor(40, 110);
    Display.print("INTERNAL BME SENSOR");

    WaitForExit();
  }
  if (Warnings & EBYTE_FAIL) {
    Display.fillRect(26, 26, 268, 188, C_WHITE);
    Display.fillRect(30, 30, 260, 180, C_RED);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE);
    Display.setCursor(40, 40);
    Display.print("FATAL ERROR");
    Display.setFont(FONT_16B);

    Display.setCursor(40, 80);
    Display.print("RESET THE");
    Display.setCursor(40, 110);
    Display.print("TRANSCEIVER");

    WaitForExit();
  }
  if (Warnings & SSD_FAIL) {
    Display.fillRect(26, 26, 268, 188, C_WHITE);
    Display.fillRect(30, 30, 260, 180, C_RED);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE);
    Display.setCursor(40, 40);
    Display.print("FATAL ERROR");
    Display.setFont(FONT_16B);

    Display.setCursor(40, 80);
    Display.print("DOWNLOAD AND");
    Display.setCursor(40, 110);
    Display.print("ERASE SSD CHIP");

    WaitForExit();
  }
}

/*---------------------------------------------------------*/
//FINAL LINKS AND INFORMATION
/*---------------------------------------------------------*/

/*
  componnet links
  Teensy                https://www.pjrc.com/store/teensy32.html
  Display               https://www.amazon.com/inch-240x320-Serial-Module-ILI9341/dp/B0749RKRFN
  Transceiver           http://www.cdeuint8_t.com/en/product-view-news.aspx?id=131
  Antenna               TX915-XP-100, TX915-JK-20
  Current sensor        https://www.ebay.com/itm/50A-100A-150A-200A-Bi-Uni-AC-DC-Current-Sensor-Module-arduino-compatible-/111689533182
  Speed sensor          https://www.amazon.com/gp/product/B01I57HIJ0/ref=oh_aui_detailpage_o00_s00?ie=UTF8&psc=1
  GPS device            https://www.amazon.com/gp/product/B01H5FNA4K/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1
  Thermsistors          https://www.digikey.com/product-detail/en/vishay-bc-components/NTCLE100E3103JB0/BC2301-ND/769411
  Power supplies        https://www.amazon.com/eBoot-LM2596-Converter-3-0-40V-1-5-35V/dp/B01GJ0SC2C/ref=sr_1_2_sspa?ie=UTF8&qid=1535223084&sr=8-2-spons&keywords=buck+converter&psc=1
  GPS plotting website: http://www.gpsvisualizer.com/map_input?form=google
  Buttons for display   https://www.amazon.com/gp/product/B0177ALAAE/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1




  connectivity map
  Teensy 3.2   device
  A0        IR-based revelotion counter
  A1        input from thermistor voltage divider
  A2        hall effect current sensor input for motor current draw
  A5        secondary temp sensor
  A9        voltage divider input for battery voltage
  0         TX for E44-TTL-100 (E32-91520D)
  1         RX for E44-TTL-100 (E32-91520D)
  2         DC on the display
  3         DN button for display mode
  4         UP button for display mode
  6         CS for the flash chip
  7         RX for GPS sensor
  8         TX for GPS sensor
  9         Chip select on the display
  10        Chip select on the SD card
  11        MOSI on the SD card and MOSI on display
  12        MISO on the SD card
  13        SCK on the SD card and SCK on the display
  Display RESET is vcc but through a resistor cap to slow the charge--prevents white screen
*/

/*
  ---------------------------------------------------------
  REVISION HISTORY
  ---------------------------------------------------------
  PRE-BOARD v4
  ---------------------------------------------------------
  REVISION      AUTHOR      DATE      DESCRIPTION
  ---------------------------------------------------------
  1.0           jacob                 leveraged Jacob's battery tester code into car data logger
  1.1           kris                  added rpm sensor
  1.2           ben         10/1      added RF integration
  1.5           ben         10/13     added dislay integration for new display
  1.6           ben         10/14     added debouncing and inputs for display swap
  1.7           kris        10/15     mild bug fixes, i.e. break; ==
  2.4           kris        1/17      added Holybro 915 mhz serial transceiver
  2.5           kris        2/17      modified for E51-TTl-500 transceiver
  2.8           Kris        2/25      added code to eliminate unit crashing SSD1306 display
  4.0           Kris        4/9       moved to teensy 3.2 and SPI display ILI9341 driver
  4.1           Kris        5/1       made provisions for configurable transmitter, and current sensor
  4.2           Kris        9/17      changed speed sensor to time based and not pulse based--namely for more accuracy
  4.3           Kris        10/28     added current calibration method
  4.4           Kris        2/18/18   added complete screen repaint every draw to compensate for display zingers, simplified speed code
  4.5           Kris        2/19/18   added support for 100 mW and 500 mW transceivers
  4.6           Kris        2/26/18   added configuration for bus speed and debouce time
  4.7           Kris        2/26/18   added configuration for current sensor type
  4.8           Kris        3/16/18   added lap logic and averaging code
  5.0           Kris        5/25      added time checker for speed sensor , updated Exx sender to structure based
  5.4           Kris        5/25      moved speed sensor to max pulses per sec, fixed lap counter to start at 0
  5.5           Kris        8/23      added gps
  5.6           Kyle        8/26      added Altitude and GPS LED support
  5.7           Kyle        8/26      fixed lap LED support
  5.8           Kyle        9/30      reworked speed sensor to report when it has a speed and not every second
  5.9           Kyle        10/1      Undated Screen Verison and updated all Display.print(F("")): statements to Display.print(F(""));
  5.9           kris        10/21     Changed setup to allow for car id (car ID then drives repeater and reciever
  BOARD v4
  ---------------------------------------------------------
  REVISION      AUTHOR      DATE      DESCRIPTION
  ---------------------------------------------------------
  B4V01.0       kris        11/23     Coded for new PCB where display is RJ45
  B4V01.1       kris        3/2       added code to compare pulse and time speed--surely one has to be right...
  B4V01.2       kris        4/12      change some setup text, reworked GPS error handler
  B4V01.3       kris        4/14      changed error handler for speed sensor
  B4V01.4       kris        6/16      removed e44 power, removed crash code for speed sensor, moved speed time to millis()
  B4V01.6       kris        6/19      moved some speed sensor code inside cli() and sei()
  B4V01.61      tom         9/3       changed "20" to "25" in the energy print screen
  B4V01.7       tom/yashas  9/3       inserted the commands for the Excel printout in the WriteHeader() function
  B4V01.8       tom         9/26      completed the Excel header file stuff
  B4V01.81      Kris        9/26      added code for multi-pickups
  B4V01.9       tom         1/9       fixed auto calcs so speed section is based on col J; added wheel diameter to autocalc section after gear ratio line and fixed next line to start from 13 and not 12
  B4V01.91      tom         1/9       created data type RevsPerLap and prevRevs
  B6V01.5       kris        3/23      updated menu system, updated transmit library to better struct packing
  ---------------------------------------------------------
  BOARD v6 (New format for changes)
  ---------------------------------------------------------
  B6_v1.0-alpha   Tom   4/20/2020
  Upgraded from Board V4 to Board V6; Major update of software with the following additions:
  - new icons for warnings
  - readjustment of headings
  - added pin for SD card reader
  - tire diameter settings update
  - update to use EBYTE library
  - added support for real-time clock
  - added time-setup menu
  - brand new consumption page
  - added timestamp to SD card write function
  - added bottom time and date header to main menu
  - added graphic to Temeprature view to show visually whether temperature is hot or not
  - configured the old lap and lap time view to show new information (i.e., lapcount, driver #, race time, pit time, current lap, last lap, best lap, and split)
  - added pit time configuration setting (for automatic pit switching)
  - added new information for driver variables, including placing a new Tranceiver struct in PatriotRacing_Utilities
  ---------------------------------------------------------
  B6_v1.1-alpha   Tom   4/28/2020
  - transitioned datalogger to use EBYTE library
  - changed driver toggle from automatic to manual (LONG PRESS on left button when abs(Amps) < 3)
  - added new Driver Setup Menu to select the race drivers
  - moved deep functions into main setup() and loop() codes to relieve stress on stack
  - to accomodate for new driver setup, recreated old icons in 32x32 px bitmap size to fit in Main Menu screen
  - reorganized code to be more sequential to follow
  - placed new visual screens in priority order (time first, then usage)
  - repositioned extra statistics and improved accuracy of reported values
  - welcome screen loads immediately upon power, not after 3 or 4 seconds as it used to
  - fixed Usage page bug where energy bar would cover parts of screen that it shouldn't
  ---------------------------------------------------------
  B6_v1.1.1       Tom   5/8/2020
  - Minor bug fixes and updated transceiver struct information
  ---------------------------------------------------------
  B6_v1.5.1       Joshua   5/16/2021
  - Split GPS data into a different file.
  - Add driver order to data file header.
  - Move headers to the right of the data.
  - Add pit detection and automatic driver change.
  - Store latest view screen.
  - GPS trigger range now uses *GPSToleranceText[] from PatriotRacing_Utilities.h.
  - Complete function documentation.
  - Reorganize data header to follow order of columns.
  - Make menu unit location more consistant.
  - Fix issue with splashscreen logo not being displayed after SD card error (#11).
  ---------------------------------------------------------
  B6_v2.0       Joshua   6/9/2021
  - Add new current sensor code to detect key state and change view title to be red when key is off and green when on.
  - The Key's state changes the color behind the view text. Green means key is on, red mean key is off.
  - Store race start position in EEPROM when race starts.
  - Add documentation to global variables.
  - Add logic to recover from powerloss during a race.
  ---------------------------------------------------------
  B6_v2.10       Joshua   6/9/2021
  - Write GPS plotting URL to GPS file header.
  - Fix data header bug.
  - Add menu option under car menu to change race start amp draw trigger threashold.
  - Fix auto driver change.
  ---------------------------------------------------------
  B6_v2.21       Coach K   03/19/2022
  - finalized data restart code
  - implemented ILI93431_t3_menu library
  - implemented watch dog timer
  - removed GPS searching in setup()
  - reduced NO SD card time

  B6_v2.25       Coach K   05/11/2022
  - fixed gps start so we search if not found at actual startup
  - added graph drawing for energy

  B6_v2.27       Coach K   07/21/2022
  - changed calibration factor to modify R25 value for different thermistors

  BOARD v7
  ---------------------------------------------------------
  B7_v1.xx   Kris   10/4/2022
  Upgraded from Board V6 to Board V7 (added SSD drive, SD cards are too dicey,
  water, humidity, fondling breaks them too often
  - Added on-board memory chip to store data
  - routines to download data from memory chip to SD card
  - updated code for a Teensy 4.0. Love 3.2 but too many supplier issues

  B7_v2.10   Kris   1/24/2022
  backed out magic voltage slope / offset. Offset needs to be based on diode drop

  B7_v2.20   Kris   1/28/2022
  Integrated latest BulletDB (pure record / field based)

  Board 8: never used due to changes to board 9

  B9_v1.0   Kris   5/14/2023
  Added MS5837 sensor for better altitude measuring, and ambient temp
  Added MPU6050 for G-force measuring
  NEW PCB

  B9_v1.10   Kris   5/14/2023
  Switched to BME280 for faster reads, and humidity measuring
  Updated output header to be more informitive
  NEW PCB
  Reduced amount of transmit data, we never look at most transmitted data, not worth the trouble
  
  B9_v2.10   Kris   9/14/2023
  added report summary data for the g-force sensor
  added MPU6050 sensor calibration
  added better menu selection for MPU6050

  B9_v2.30   Kris   10/08/2023
  added driver time restoring capability, required updated BulletDB

  B9_v2.40   Kris   10/08/2023
  added setup options for accelerometer
  
  B9_v2.50   Kris   11/17/2023
  changed how race start is determined--making it similar to pit change detection
  added Energy/Lap output

    B9_v2.74   Kris   1/30/2023
  changed speed sensor lib to be completely hardware based--hard is perfect, hall sensors have some +/- 1 RPM errors

*/



/*
  END OF DATALOGGER CODE
  CODED BY:
  Jacob H., Ben Runyan, JOSHUA C., YASHAS G., THOMAS T.
*/
