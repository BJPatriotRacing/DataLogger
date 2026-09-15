/*
  ---------------------------------------------------------
  PROGRAM INFORMATION
  ---------------------------------------------------------
  Bob Jones Patriot Racing Car Datalogger
  Copyright 2016-2026, All Rights reserved
  This code is property of Patriot Racing and Kris Kasprzak
  This code cannot be used outside of Bob Jones High School
  Code for Teensy 3.2 or 4.0
  ---------------------------------------------------------
  COMPILE INSTRUCTIONS
  ---------------------------------------------------------
  if the MCU is a 3.2
  Compile Speed:  72 MHz       
  Optimize:       smallest code

  if the MCU is a 4.0
  Compile Speed:  600 MHz       
  Optimize:       fastest code

  LIBRARY CHANGES
  1) in ILI9341_Menu.h has #define MAX_OPT 16 (or max number of menu items)
  adding fields is fine, but you may need to adjust the limit the library

  // in BulletDB.h change 
  1) the limit as needed #define MAX_FIELDS 25 (change to max files in BuildFieldList--the 
  lib will not automatically accomodate larger field lists
  2) the WRITE_SPEED should be 25000000

  // in ILI9341_t3_Controls.h change 
  1) #define MAX_GRAPHS 12 (change to max number of graphs Graph.add())

  ---------------------------------------------------------
  CODE PURPOSE
  ---------------------------------------------------------
  1. MEASURE: Volts, Amps, Motor Temperature (internal and external), Wheel RPM, G-Force, and GPS location
  2. COMPUTE: Speed, Power, Energy, Averages, Driver Statistics
  3. OUTPUT:  Volts, Amps, Power, etc.
  4. WRITE:   Data to a flash chip, and later download to an SD card for analysis
  5. SEND:    Data Wirelessly to RECEIVER module (see WebServer -  WIFIStation_xx Code)

*/

/*-------------------*/
// Libraries
/*-------------------*/

// #include <avr/io.h>                    // standard library that ships with Teensy
// #include <avr/interrupt.h>
#include <SPI.h>                       // MUST USE THIS LIB https://github.com/PaulStoffregen/SPI
#include <PatriotRacing_Utilities.h>   // custom utilities definition
#include <PatriotRacing_Icons.h>       // lib of dedicated defines, struct, etc.
#include <ILI9341_t3_Menu.h>           // Menu library      https://github.com/KrisKasprzak/ILI9341_t3_Menu
#include <ILI9341_t3_Controls.h>       // Controls library  https://github.com/KrisKasprzak/ILI9341_t3_controls
#include <ILI9341_t3.h>                // Display library   https://github.com/PaulStoffregen/ILI9341_t3
#include <SdFat.h>                     // SD card           https://github.com/greiman/SdFat
#include <EEPROM.h>                    // standard library that ships with Teensy
#include <EasyTransfer.h>              // manages struct compression for sending data https://github.com/madsci1016/Arduino-EasyTransfer
#include <TinyGPSPlus.h>               // GPS module lib    https://github.com/mikalhart/TinyGPSPlus
#include <TimeLib.h>                   // time libs         https://github.com/PaulStoffregen/Time https://github.com/PaulStoffregen/Time/blob/master/TimeLib.h
#include <EBYTE_E220.h>                // transceiver lib   https://github.com/KrisKasprzak/EBYTE
#include <ILI9341_FlickerFreePrint.h>  // eliminates text drawing flicker https://github.com/KrisKasprzak/FlickerFreePrint/blob/master/FlickerFreePrint.h
#include <Arial_100_BINO.h>            // special font file that is numbers only. stored in PatriotRacing_Fonts, recreate use https://spooksoft.pl/download/, https://spooksoft.pl/en/download-3/
#include <Arial_48_BINO.h>             // special font file that is numbers only. stored in PatriotRacing_Fonts, recreate use https://spooksoft.pl/download/, https://spooksoft.pl/en/download-3/
#include <font_ArialBold.h>            // comes with display or go here https://github.com/PaulStoffregen/ILI9341_fonts
#include <font_Arial.h>                // comes with display or go here https://github.com/PaulStoffregen/ILI9341_fonts
#include <font_ArialBoldItalic.h>      // https://github.com/PaulStoffregen/ILI9341_fonts
#include <BulletDB.h>                  // flash chip database driver https://github.com/KrisKasprzak/BulletDB
#include <Wire.h>
#include <MPU6050.h>  // accelerometer lib  https://github.com/ElectronicCats/mpu6050

#if defined(__MK20DX256__)     // Teensy 3.2
#include <FreqMeasureMulti.h>  // lib for speed sensor https://github.com/PaulStoffregen/FreqMeasure
#elif defined(__IMXRT1062__)   // Teensy 4.0 or 4.1
#include <FreqMeasure.h>
#else
#include <FreqMeasureMulti.h>  // lib for speed sensor https://github.com/PaulStoffregen/FreqMeasure
#endif
#include <MS5837_02BA.h>  // altimiter chip https://github.com/KrisKasprzak/MS5837_02BA
#include <PID_v1.h>       // https://github.com/mblythe86/C-PID-Library/tree/master/PID_v1
#include <MCP3208.h>      // ADC chip https://github.com/KrisKasprzak/MCP3208
#include <XGZP6897D.h>    // air flow sensor https://github.com/fanfanlatulipe26/XGZP6897D
#if defined(__IMXRT1062__)
#include "Watchdog_t4.h"  // https://github.com/tonton81/WDT_T4/tree/master
#endif


// #define DO_DEBUG
// #define LR_DEBUG

/*-------------------*/
// Code Version
/*-------------------*/
// BoardMajor.BoardMinor.CodeMajor.CodeMinor

#define CODE_VERSION "10.30.03.26"

/*-------------------*/
//Constant Definitions
/*-------------------*/

// see PatriotRacing_Utilities.h for many other defines

//Global Fonts and Locations
#define FONT_100BINO Arial_100_BINO     // font for the large data
#define FONT_48BINO Arial_48_BINO       // custom font for lap averages
#define FONT_24BI Arial_24_Bold_Italic  // font for the small data
#define FONT_16B Arial_16_Bold          // font for all headings
#define FONT_14 Arial_14                // font for menus
#define DATA_X 305                      // left justify would be 15;
#define PITMESSAGEX 310
#define PITTIMEX 215
#define PEAK_TIMER 4000
// colors for graphing
#define GCOLOR_VOLTS C_CYAN
#define GCOLOR_AMPS C_YELLOW
#define GCOLOR_LAMPS C_LTPURPLE
#define GCOLOR_SPEED C_GREEN
#define GCOLOR_TEMP C_RED
#define GCOLOR_ALT C_LTBLUE
#define GCOLOR_CBGIN C_BLUE
#define GCOLOR_CBGOUT C_ORANGE
#define GCOLOR_PNRG C_MAGENTA
#define MENU_TEXT C_WHITE
#define MENU_BACK C_BLACK
#define MENU_HIGHTEXT C_WHITE
#define MENU_HIGHLIGHT C_MDBLUE
#define MENU_HIGHBORDER C_DKBLUE
#define MENU_SELECTTEXT C_WHITE
#define MENU_SELECT C_RED
#define MENU_TITLETEXT C_WHITE
#define MENU_TITLEBACK C_DKBLUE

#if defined(__MK20DX256__)    // Teensy 3.2
#define GPSSerial Serial3     // setup serial port for GPS Teensy 3.2
#elif defined(__IMXRT1062__)  // Teensy 4.0 or 4.1
#define GPSSerial Serial2     // setup serial port for GPS Teensy 4.0
#else
#define GPSSerial Serial3  // setup serial port for GPS Teensy 3.2
#endif

/*-------------------*/
// Program Variables
/*-------------------*/

bool DrawGraph = true;
uint16_t EnergyPoints[100];

// 0 to 526
const uint16_t BLEnergy[527] = { 0, 1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24, 26, 27, 28, 29, 30, 31, 32, 34, 35, 36, 37, 39, 40, 41, 42, 44, 45, 46, 47, 48, 50, 51,
                                 52, 53, 55, 56, 57, 58, 60, 61, 62, 63, 64, 66, 67, 68, 69, 71, 72, 73, 74, 76, 77, 78, 79, 80, 82, 83, 84, 85, 87, 88, 89, 90, 92, 93, 94, 95, 96, 98, 99, 100, 101, 103, 104, 105, 106, 107, 108,
                                 109, 111, 112, 113, 114, 115, 117, 118, 119, 120, 122, 123, 124, 125, 126, 128, 129, 130, 131, 133, 134, 135, 136, 137, 139, 140, 141, 142, 143, 145, 146, 147, 148, 150, 151, 152, 153,
                                 154, 156, 157, 158, 159, 160, 162, 163, 164, 165, 166, 168, 169, 170, 171, 172, 174, 175, 176, 177, 178, 180, 181, 182, 183, 184, 185, 186, 188, 189, 190, 191, 192, 194, 195, 196, 197,
                                 198, 200, 201, 202, 203, 204, 206, 207, 208, 209, 210, 212, 213, 214, 215, 216, 218, 219, 220, 221, 222, 224, 225, 226, 227, 228, 229, 231, 232, 233, 234, 235, 237, 238, 239, 240, 241,
                                 243, 244, 245, 246, 247, 248, 250, 251, 252, 253, 254, 256, 257, 258, 259, 260, 261, 262, 263, 264, 266, 267, 268, 269, 270, 272, 273, 274, 275, 276, 277, 279, 280, 281, 282, 283, 284,
                                 286, 287, 288, 289, 290, 291, 293, 294, 295, 296, 297, 298, 300, 301, 302, 303, 304, 305, 307, 308, 309, 310, 311, 312, 314, 315, 316, 317, 318, 319, 321, 322, 323, 324, 325, 326, 327,
                                 329, 330, 331, 332, 333, 334, 335, 336, 337, 339, 340, 341, 342, 343, 344, 345, 347, 348, 349, 350, 351, 352, 353, 355, 356, 357, 358, 359, 360, 361, 363, 364, 365, 366, 367, 368, 369,
                                 371, 372, 373, 374, 375, 376, 377, 379, 380, 381, 382, 383, 384, 385, 387, 388, 389, 390, 391, 392, 393, 394, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407, 409, 410, 411, 412,
                                 413, 414, 415, 416, 418, 419, 420, 421, 422, 423, 424, 425, 427, 428, 429, 430, 431, 432, 433, 434, 435, 437, 438, 439, 440, 441, 442, 443, 444, 445, 447, 448, 449, 450, 451, 452, 453,
                                 454, 455, 456, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479, 480, 481, 482, 483, 484, 486, 487, 488, 489, 490, 491, 492, 493, 494,
                                 495, 496, 497, 498, 500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 514, 515, 516, 517, 518, 519, 520, 521, 522, 523, 524, 525, 526, 527, 528, 529, 531, 532, 533, 534, 535,
                                 536, 537, 538, 539, 540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551, 552, 553, 554, 555, 556, 557, 558, 559, 560, 562, 563, 564, 565, 566, 567, 568, 569, 570, 571, 572, 573, 574,
                                 575, 576, 577, 578, 579, 580, 581, 582, 583, 584, 585, 586, 587, 588, 589, 590, 591, 592, 593, 594, 595, 596, 597, 598, 599, 600 };
uint32_t EnergyXPoint = 0;
int8_t EnergyID = 0, bEnergyID = 0;
int8_t GraphVoltsID = 0, GraphAmpsID = 0, GraphLapAmpsID = 0, GraphSpeedID = 0, GraphMTempID = 0, GraphAltitudeID = 0;
uint8_t GraphCyborgInID = 0, GraphCyborgOutID = 0, GraphPredictedID = 0;
uint8_t Button = 0;
uint8_t ButtonPressed = NO_BUTTON;
float ASTemp = 0.0f;
float ASPressure = 0.0f;
float fpm = 0.0f;
float AirSpeed = 0.0f;
float AirSpeedOffset = 0.0f;
uint8_t MaxDisplayIDs = (sizeof(DisplayIDText) / sizeof(DisplayIDText[0])) - 1;

// CYBORG
bool EnablePIDTuning = false;
float CyborgMinRange = 16.0f;
float CyborgMaxRange = 24.0f;
float ThrottleMinRange = 0.0f;
float ThrottleMaxRange = 0.0f;
uint32_t ThrottleInputBits = 0;
float ThrottleInputVolts = 0.0f;
bool EnableCyborg = false;
uint16_t CyborgOutputPWM = 0;
uint32_t CyborgAmpBits = 0;
uint32_t CyborgFLBits = 0;
float CyborgFL = 0.0f;
float CyborgAmps = 0.0f;
float CyborgAmpVolts = 0.0f;
float CyborgFirstLimit = 19.0f;
float OldCyborgFirstLimit = 19.0f;
float CyborgSetpoint = 19.0f;
float CyborgSecondLimit = 25.0f;
uint16_t CyborgUpdateTime = 100;
uint8_t CyborgInput = CYBORG_CONTROL_AMPS;
bool CyborgTurbo = false;
bool CyborgActive = false;
float ESCVoltsOn = 3.1f;
float ESCVoltsOff = 0.8f;
double Setpoint = 0.0, Input = 0.0, Output = 0.0;
double Kp = 5.0, Ki = 500.0, Kd = 0.0;
int CyborgXPoint = 0;
float PredictionCompensation = 86.0f;
float PredictedEnergy = 0.0f;
double m_PE = 0.0, b_PE = 0.0;
float EnergyArray[PREDICT_SAMPLES];
float PointArray[PREDICT_SAMPLES];
uint8_t PredictionArrayCounter = 0;
uint16_t iCP = 0;
double sum_y = 0.0, sum_xx = 0, sum_x = 0, sum_xy = 0, den = 0.0;
double c1 = 0, c2 = 0, c3 = 0, c4 = 0;
uint16_t EPointer = 0, oEPointer = 0;
uint8_t CyborgInSignal = 0;
uint8_t CyborgOutSignal = 0;

// device operation status
bool RPMStatus = false, RadioStatus = false, GForceStatus = false, SDCardStatus = false, GPSStatus = false, SSDStatus = false;
bool EXTADCStatus = false, RedrawDisplay = false, RedrawHeader = false, DriverTimeOK = false, AltimiterStatus = false;
bool AirFlowSensorStatus = false, SpeedSensorStatus = false;

uint8_t RecordSETID = 0;          // each stored race is a recordset
uint8_t RecordType = RT_HEADER;   // flag to indicated if the record is race data or a race header (fixed settings)
uint8_t RestoreType = STATUS_OK;  // if restored data upon datalogger restart, we track this
uint16_t Point = 0;               // Counter for the data Point
uint8_t LapCount = 0;
uint8_t Driver = 0;  // variable for current driver 0-2
float mRPM = 0.0f;
float Volts = 0.0f;
float Power = 0.0f;
float AmbTemp = 0.0f;
float MotorTemp = 0.0f;
float AuxTemp = 0.0f;
float Energy = 0.0f;
float AmpHours = 0.0f;
float Amps = 0.0f;
float CarSpeed = 0.0f;
float Distance = 0.0f;
float TrackLength = 0.0f;
float StartDistance = 0.0f, EndDistance = 0.0f;
float StartTime = 0, EndTime = 0;
float Altitude = 0.0f;
float AtmPressure = 0.0f;
int16_t AltCorrection = 0, GPSAltCorrection = 0;
float GPSAltitude = 0.0f;
float GPSStartLat = 0.0f, GPSStartLon = 0.0f, GPSSpeed = 0.0f;
float GForceX = 0.0f, GForceY = 0.0f, OldGForceY = 0.0f, GForceZ = 0.0f, GForce = 0.0f;
bool NeedToUpdateTime = true;
bool NeedToReplaceBattery = true;
uint8_t PeakGValue = 0;
bool EnableAirFlowSensor = false;

int32_t PitTime = 0;
int32_t MinDriveTime = 0;  // ms
int32_t MaxDriveTime = 0;
bool StartGPSFound = false;
uint16_t banner_back = C_YELLOW;
int Tyear = 0;
int Tmonth = 0;
int Tday = 0;
int Thour = 0;
int Tminute = 0;
uint8_t GForceRange = 4;
uint8_t AccelLPFilter = 0;
uint8_t AccelHPFilter = 0;
bool RestartDisplayAlways = false;
bool AddLapInPit = true;
uint32_t LastRecord = 0;
uint32_t Record = 0;

//Setup Variables
uint8_t RadioUpdate = 1;          // duration to send data through EBYTE
uint16_t CarID = 0;               // 0 = blue, 1 = red, 3 = white
uint8_t MotorSprocket = 15;       // number of teeth for motor sprocket
uint8_t WheelSprocket = 70;       // number of teeth for driven sprocket
float GearRatio = 4.66666f;       // gear ratio
uint8_t TireID = 0;               // tire id for tiretext lookup
uint8_t MotorID = 0;              // motor id for tiretext and tirerad arrays
uint8_t TirePressureFront = 115;  // input for tire pressure
uint8_t TirePressureRear = 115;   // input for tire pressure
float TireRad = 9.0f;             // will get set later by tire ID
uint16_t TotalEnergy = 600;       // default energy value, sum of both batteries at 10.5 volt mark

uint8_t Pickups = 4;              // default pickups
uint8_t Theme = 0, OldTheme = 0;  // track white / black display background
uint8_t Orientation = 0;          // track display orientation
uint8_t GPSTolerance = 4;         // tolerance for GPS for getting start location

// variables to track if we draw a graph of saved data
bool RPBDrawGraphs = false, RPBRaceLines = false, RPBPlotVolts = false, RPBPlotAmps = false, RPBPlotLapAmps = false, RPBPlotSpeed = false;
bool RPBPlotMTemp = false, RPBPlotAltitude = false, RPBCyborgIn = false, RPBCyborgOut = false, RPBPlotMPEnergy = false;

bool AutoCurrentCal = true;
uint8_t Battery1 = 0, Battery2 = 0;

// note the ACS-770 U200 curent sensor has sensitivity of 20 mV/Amp, 0.5 volts offset at 0 amps,
// unit is powered with 5.0 Vcc
float VMid = 0.5f;               // offset for current sensor, right from data sheet (note we have an option to zero this out on startup)
float mVPerAmp = 20.0f;          // sensitivity for current sensor, right from data sheet
float VoltageSlope = 11.0f;      // Vin slope from the voltage divider.
float VoltageOffset = 0.299f;    // Vin is comming through diodes need to compensate that loss.
float ThermResMotor = 10000.0f;  // voltage divider resistor for external thermistor sensor
float ThermResAux = 10000.0f;    // voltage divider resistor for external thermistor sensor
uint8_t LapThreshold = 30;       // time in seconds required to elapse before another lap is allowed to be counted
float ASensorBits = 4096.0f;
int16_t AccelCalX = 0, AccelCalY = 0, AccelCalZ = 0, gx = 0, gy = 0, gz = 0;
uint32_t Duration = 0;
uint8_t ASensorDirection = 0;
float ax = 0.0f, ay = 0.0f, az = 0.0f;

uint32_t DelayAmount = 0;
char buf[50];     // generic buffer for various uses
char pitbuf[50];  // generic buffer for various uses
float GraphPointX = 0.0f;

//Time Variables (RTCTime is a time_t object; others are used to set RTCTime)
int16_t hours = 0, minutes = 0, seconds = 0, days = 0, months = 0, years = 0;
int16_t RaceDay = 0, RaceHour = 0, RaceMinute = 0, RaceSecond = 0, RaceMonth = 0;
uint32_t ExitStartTime = 0;
bool EnableAutoExit = true;

uint8_t RaceStatus = RACE_NOTSTARTED, KeyState = LOW, OldKeyState = HIGH, DisplayOldKeyState = LOW, KeyStatus = KEYSTATUS_TURNED_ON;

uint8_t MaxG = 0;
float LapSpeed = 0.0f, MaxLapGForceY = 0.0f, GForceYLapMaxL = 0.0f, GForceYLapMaxR = 0.0f, TempGForceYLapMaxL = 0.0f, TempGForceYLapMaxR = 0.0f;

// Speed and Distance Variables
float RPMSum = 0.0f, RPMCount = 0.0f;

float Revolutions = 0.0f;  // counts total to compute Distance
uint32_t Counter = 0;      // the number of measurements between each display
uint32_t AverageCounter = 0;

//Driver Variables
uint8_t DriverLaps[3] = { 0, 0, 0 };  // Array for number of laps per driver
int32_t DriverTime[3] = { 0, 0, 0 };  // Array for driver time, note this is signed so we can alert driver if time is exceeded

//Transceivers Variables
uint8_t AirDataRate = 0, RadioPower = 0, RadioChannel = 0, RadioAddressL = 0, RadioAddressH = 0, ResetEBYTE = 0;

//Car Variables
float WRPM = 0.0f;  // wheel rpm (measured)
float aVolts = 0.0f, vVolts = 0.0f, thmVolts = 0.0f, thxVolts = 0.0f, TempK = 0.0f;
float AmbTempCF = 0.0f, LapAmps = 0.0f;
float ERem = 100.0f, TRem = 100.0f;  // Energy and Time remaining
float ThermistorResistence = 0.0f;   // computed thermistor resistance
int16_t hr = 0, mn = 0, sc = 0;      // for formatting min and sec

//Buttons
uint8_t DisplayID = 0, OldDisplayID = 0, L_PIN = LEFT_PIN, R_PIN = RIGHT_PIN;

//Variables for Average Calculations
uint32_t AverageCount = 0;
float LapVolts = 0.0f, AverageAmps = 0.0f, TargetAmps = 0.0f, TempTargetAmps = 0.0f;
float TriggerAmps = 15.0f;  // Minimum amp required to trigger a race start or dirver change
float AverageVolts = 0.0f, LapEnergy = 0.0f, StartLapEnergy = 0.0f, AverageCarSpeed = 0.0f;
uint8_t StartGPSDelayID = 0;
uint32_t StartGPSDelay = 0;

float GPSLat = 0.0f, GPSLon = 0.0f, GPSDistance = 0.0f;
uint16_t LapTime = 0, LastLapTime = 0, GPSSatellites = 0;
uint16_t ForeColor = 0, BackColor = 0, InactiveColor, BarColor = 0, PeakColor = 0, DetailsColor = 0;  // remember the foreground and background colors
uint16_t i = 0, j = 0;                                                                                // just some storage variables
int TimeDelta = 0;
//Warning Global Variable
uint16_t Warnings = 0;

// menu ID variables
// to track when to exit a menu
uint8_t MainMenuOption = 0, MenuOption = 0;

// ID's for menu option
uint8_t MainMenuID1 = 0, MainMenuID2 = 0, MainMenuID3 = 0, MainMenuID4 = 0, MainMenuID5 = 0, MainMenuID6 = 0, MainMenuID7 = 0;
uint8_t MainMenuID8 = 0, MainMenuID9 = 0;

uint8_t RaceMenuID1 = 0, RaceMenuID2 = 0, RaceMenuID3 = 0, RaceMenuID4 = 0, RaceMenuID5 = 0, RaceMenuID6 = 0, RaceMenuID7 = 0;
uint8_t RaceMenuID8 = 0, RaceMenuID9 = 0, RaceMenuID10 = 0, RaceMenuID12 = 0, RaceMenuID13 = 0, RaceMenuID14 = 0;

uint8_t SettingsMenuID1 = 0, SettingsMenuID2 = 0, SettingsMenuID3 = 0, SettingsMenuID4 = 0, SettingsMenuID5 = 0, SettingsMenuID6 = 0;

uint8_t WirelessMenuID1 = 0, WirelessMenuID2 = 0, WirelessMenuID3 = 0, WirelessMenuID4 = 0;
uint8_t WirelessMenuID5 = 0, WirelessMenuID6 = 0, WirelessMenuID7 = 0, WirelessMenuID8 = 0, WirelessMenuID9 = 0;

uint8_t SensorMenuID1 = 0, SensorMenuID2 = 0, SensorMenuID3 = 0, SensorMenuID4 = 0, SensorMenuID5 = 0, SensorMenuID6 = 0, SensorMenuID7 = 0;
uint8_t SensorMenuID9 = 0, SensorMenuID10 = 0, SensorMenuID11 = 0, SensorMenuID12 = 0;

uint8_t GForceMenuID1 = 0, GForceMenuID2 = 0, GForceMenuID3 = 0, GForceMenuID4 = 0, GForceMenuID5 = 0, GForceMenuID6 = 0;
uint8_t GForceMenuID7 = 0, GForceMenuID8 = 0;

uint8_t ClockMenuID1 = 0, ClockMenuID2 = 0, ClockMenuID3 = 0, ClockMenuID4 = 0, ClockMenuID5 = 0, ClockMenuID6 = 0;

uint8_t CyborgMenuID1 = 0, CyborgMenuID2 = 0, CyborgMenuID3 = 0, CyborgMenuID4 = 0;
uint8_t CyborgMenuID5 = 0, CyborgMenuID6 = 0, CyborgMenuID7 = 0, CyborgMenuID8 = 0;
uint8_t CyborgMenuID9 = 0, CyborgMenuID10 = 0, CyborgMenuID11 = 0, CyborgMenuID12 = 0, CyborgMenuID13 = 0;

uint8_t SSDMenuID1 = 0, SSDMenuID3 = 0;

uint8_t PlayBackID1 = 0, PlayBackID2 = 0, PlayBackID3 = 0, PlayBackID4 = 0, PlayBackID5 = 0;
uint8_t PlayBackID6 = 0, PlayBackID7 = 0, PlayBackID8 = 0, PlayBackID9 = 0, PlayBackID10 = 0;
uint8_t PlayBackID11 = 0;

// field ID variables
uint8_t frType = 0, frID = 0, frPoint = 0, frLap = 0, frDriver = 0, frVolts = 0, frAmps = 0, frMotorTemp = 0, frAuxTemp = 0;
uint8_t frEnergy = 0, frRPM = 0, frWRPM = 0, frAmpHours;
uint8_t frSpeed = 0, frDist = 0, frRT = 0, frLon = 0, frLat = 0, frAltitude = 0, frGPSpeed = 0, frRestoreType = 0, frMax = 0, frMay = 0, frMaz = 0;
uint8_t frAmbTemp = 0, frCyborgInSignal = 0, frCyborgOutSignal = 0, frAirSpeed = 0, frAirOffset = 0, frGPSAltitude = 0;
uint8_t frPredictedEnergy = 0, frCyborgFirstLimit = 0, frDisplayID = 0, frTrackLength = 0, frLapTime = 0;

// header ID variables
uint8_t hrType = 0, hrID = 0, hrYear = 0, hrMonth = 0, hrDay = 0, hrHour = 0, hrMinute = 0, hrMSprocket = 0, hrWSprocket = 0;
uint8_t hrStartAltitude = 0, hrTireID = 0, hrTirePressureFront = 0, hrTirePressureRear = 0;
uint8_t hrD0ID = 0, hrD1ID = 0, hrD2ID = 0, hrCarID = 0, hrMotorID = 0, hrTemp = 0, hrAmbTemp = 0, hrStartPressure = 0;
uint8_t hrEnergy = 0, hrCounter = 0, hrBattery1 = 0, hrBattery2 = 0, hrLon = 0, hrLat = 0;

uint16_t ReturnCode = 0;
uint32_t UsedSpace = 0;
uint32_t RealClockTime = 0;

/*---------------------------------------------------------*/
//OBJECT INITIALIZATION
/*---------------------------------------------------------*/

// display
ILI9341_t3 Display(DCS_PIN, DRS_PIN);  //Display object

// gps sensor
TinyGPSPlus GPSSensor;

// sd card
SdFat SDCARD;
SdFile SDDataFile;
SdFile SDSetupFile;

// structure for the wireless data transmission
Transceiver Data;

// wireless device
EBYTE_E220 Radio(&ESerial, M0_PIN, M1_PIN, AX_PIN);  //Transceiver object

// special lib to address data packing for wireless
EasyTransfer DataPacket;

// flicker free objects for data display
ILI9341_FlickerFreePrint ffMainData(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffLapData(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffEnergy(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffEnergyPerLap(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffAmpsPerLap(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffDriverLapTime(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffDriverTime(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffTime(&Display, C_WHITE, C_BLACK, JUSTIFY_LEFT);
ILI9341_FlickerFreePrint ffCyborgSetpoint(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffCyborgInput(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffCyborgThrottle(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffGForceY(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffGForceYLapMaxL(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffGForceYLapMaxR(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffPitMessage(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffPredict(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffLap(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);

// top level menu items
ItemMenu TopMainMenu(&Display);
ItemMenu SSDMenu(&Display);

// sub menu items
EditMenu RaceMenu(&Display);
EditMenu SettingsMenu(&Display);
EditMenu WirelessMenu(&Display);
EditMenu GForceMenu(&Display);
EditMenu SensorMenu(&Display);
EditMenu ClockMenu(&Display);
EditMenu CyborgMenu(&Display);
EditMenu PlaybackMenu(&Display);

// timers
elapsedMillis GraphDrawTimer = 0;
elapsedMillis RadioUpdateTimer = 0;
elapsedMillis GraphStoreTimer = 0;
elapsedMillis DisplayUpdateTimer = 0;
elapsedMillis GPSLapTimer = 0;
elapsedMillis CarRaceTimer = 0;
elapsedMillis DriverTimer = 0;
elapsedMillis GPSMaxReadTimer = 0;
elapsedMillis LapTimer = 0;
elapsedMillis SpeedUpdateTimer = 0;
elapsedMillis PressTimer = 0;
elapsedMillis DriverChangeScreen = 0;
elapsedMillis StartGPSDelayTimer = 0;
elapsedMillis CyborgTimer = 0;
elapsedMillis AMETimer = 0;
elapsedMillis PeakTimer = 0;
elapsedMillis GPSLEDTimer = 0;
elapsedMillis CyborgScreenUpdate = 0;

// accelerometer
MPU6050 AccelSensor;

#if defined(__MK20DX256__)  // Teensy 3.2
FreqMeasureMulti RPMSensor;
#endif

// flash chip data driver
BulletDB SSD(SSD_PIN);

// object for external ADC chip
MCP3208 EXTADC(EXTADC_CS_PIN);  // my chip select pin

// object for dedicated altimiter (not GPS altimiter)
MS5837_02BA PressureSensor;

// air flow sensor
XGZP6897D AirFlowSensor(FLOW_SENSOR_K);

// graph function for energy
CGraph EnergyG(&Display, GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, 0, 90, 15, 0, 700, 100);

// bar charts for temperature, time, energy
BarChartV MotorTempG(&Display);
BarChartV AuxTempG(&Display);
BarChartV AmbTempG(&Display);
BarChartV TRemG(&Display);
BarChartV ERemG(&Display);

BarChartH LapAverageBar(&Display);

// lib to manage constant current to esc
PID CyborgPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#if defined(__IMXRT1062__)
WDT_T4<WDT1> WDT;
WDT_timings_t WDTConfig;
#endif

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

  Serial.println("Starting");

  SPI.begin();

  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
  pinMode(CD_PIN, INPUT_PULLUP);
  pinMode(TURBO_PIN, INPUT_PULLUP);

  // shut down unused pins
  pinMode(24, INPUT_DISABLE);
  pinMode(25, INPUT_DISABLE);
  pinMode(26, INPUT_DISABLE);
  pinMode(27, INPUT_DISABLE);
  pinMode(28, INPUT_DISABLE);
  pinMode(29, INPUT_DISABLE);
  pinMode(30, INPUT_DISABLE);
  pinMode(31, INPUT_DISABLE);
  pinMode(32, INPUT_DISABLE);
  pinMode(33, INPUT_DISABLE);

  analogWriteResolution(12);
  analogReadRes(12);
  analogReadAveraging(1);

  analogWriteFrequency(OUTPUT_PIN, 10000);
  analogWrite(OUTPUT_PIN, 0);

  StartDisplay();

  Display.fillScreen(C_BLACK);

  // init the SSD chip
  SSDStatus = SSD.init();

  StartRTC();

  Wire.begin();

  Wire.setClock(400000);

  ////////////////////////////////////
  // this is the magic trick for printf to support float
  asm(".global _printf_float");

  // this is the magic trick for scanf to support float
  asm(".global _scanf_float");

  GetParameters();

  Display.fillScreen(C_BLACK);

  SetScreenParameters();

  EXTADCStatus = EXTADC.init();

  delay(10);

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

  Display.setFont(FONT_24BI);
  Display.setCursor(10, 5);
  Display.print(F("PATRIOT RACING"));

  Display.setFont(FONT_14);
  Display.setTextColor(C_WHITE);

  Display.setCursor(STATUS_TYPE, 40);
  Display.print("SSD / ADC: ");
  Display.setCursor(STATUS_TYPE, 60);
  Display.print(F("G-Force: X,Y,Z"));
  Display.setCursor(STATUS_TYPE, 80);
  Display.print(F("Volts / Amps"));
  Display.setCursor(STATUS_TYPE, 100);
  Display.print(F("Temp M / X / A"));
  Display.setCursor(STATUS_TYPE, 120);
  Display.print(F("GPS"));
  Display.setCursor(STATUS_TYPE, 140);
  Display.print(F("Gas / Sped / Air"));
  Display.setCursor(STATUS_TYPE, 160);
  Display.print(F("Wireless"));
  Display.setCursor(STATUS_TYPE, 180);
  Display.print(F("Flash memory"));
  Display.setCursor(STATUS_TYPE, 200);
  Display.print(F("Status"));
  Display.setCursor(STATUS_TYPE, 220);
  Display.print(F("Time / Date: "));

  Display.setCursor(STATUS_RESULT, 220);
  if (hour() > 12) {
    sprintf(buf, "%d:%02d, %d/%02d/%02d", hour() % 12, minute(), month(), day(), year() - 2000);
  } else {
    sprintf(buf, "%d:%02d, %d/%02d/%02d", hour(), minute(), month(), day(), year() - 2000);
  }
  Display.print(buf);

  ESerial.begin(9600);

  // remap MCU Tx pin so we can light up an LED on lap trigger
  // this must be called acter GPSSerial.begin()
  GPSSerial.begin(9600);
  pinMode(LAPLED_PIN, OUTPUT);
  digitalWrite(LAPLED_PIN, LOW);

  delay(10);

  CreateUserInterface();

  BuildFieldList();

  LastRecord = SSD.findLastRecord();

  SSD.gotoRecord(LastRecord);

  RecordSETID = SSD.getField(RecordSETID, frID);

  if ((RecordSETID == 0xFFFF) || (LastRecord == 0)) {
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
  Serial.print("max records: ");
  Serial.println(SSD.getMaxRecords());
  Serial.print("total space: ");
  Serial.println(SSD.getTotalSpace());
  Serial.print("Chip ID: ");
  Serial.println(SSD.getUniqueChipID());
  SSD.listFields();
  SSD.listHeaderFields();
  // SSD.dumpRecords(0, 500);
#endif

  if (SSD.getUsedSpace() > 7000000000) {  // 8 mb chip and race typically needs 970K
    Warnings = Warnings | SSD_FAIL;
  }

  AreWeInARace();

  //Configure up/down
  ConfigureButtons();

  InitializeDevices();

  HandleRaceStatus();

  // display big giant errors for any failed devices
  if (RaceStatus == RACE_NOTSTARTED) {
    DisplayErrors();
    delay(500);
  }

  //Once done with setup, transition to showing stats on car screen
  Display.fillScreen(BackColor);

  WatchDogTimer(ENABLE_WDT);

  TempGForceYLapMaxL = 0.0;
  TempGForceYLapMaxR = 0.0;
  GraphDrawTimer = 0;
  GraphStoreTimer = 0;
  DisplayUpdateTimer = 0;
  RadioUpdateTimer = 0;
  GPSLapTimer = 0;
  LapTimer = 0;
  GraphDrawTimer = 60000;
  GPSMaxReadTimer = 0;
  SpeedUpdateTimer = 0;
  RedrawHeader = true;
  DriverChangeScreen = 6000;
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

#if defined(__MK20DX256__)  // Teensy 3.2
  if (RPMSensor.available()) {
    RPMSum = RPMSum + RPMSensor.read();
    RPMCount = RPMCount + 1.0f;
    Revolutions = Revolutions + (1.0f / (float)Pickups);
  }
#else
  if (FreqMeasure.available()) {
    RPMSum = RPMSum + FreqMeasure.read();
    RPMCount = RPMCount + 1.0f;
    Revolutions = Revolutions + (1.0f / (float)Pickups);
  }
#endif

  ButtonPressed = WhatButtonWasPressed();

  if ((ButtonPressed == L_BUTTON) || (ButtonPressed == C_BUTTON_LONG) || (ButtonPressed == R_BUTTON)) {
    ButtonPress();
    WaitForRelease();
  }

  // Measure and average volts, amps, temperature
  vVolts = vVolts + EXTADC.analogRead(EXTADC_VM_PIN);
  aVolts = aVolts + EXTADC.analogRead(EXTADC_AM_PIN);
  thmVolts = thmVolts + EXTADC.analogRead(EXTADC_THM_PIN);
  thxVolts = thxVolts + EXTADC.analogRead(EXTADC_THX_PIN);

  if (GPSTolerance != 0) {
    GPSRead();
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // CYBORG, this section updates pid on each read
  // if this works better, roll it into the next section
  if (EnableCyborg) {

    if (EnablePIDTuning) {
      Kp = EXTADC.analogRead(EXTADC_KP_PIN);
      Ki = EXTADC.analogRead(EXTADC_KI_PIN);
      Kd = 0;

      Kp = ((uint16_t)(Kp / 5.0)) * 5.0;
      Ki = ((uint16_t)(Ki / 5.0)) * 5.0;

      CyborgPID.SetTunings(Kp, Ki, Kd);
      /*
      Serial.print(Kp);
      Serial.print(", ");
      Serial.print(Ki);
      Serial.print(", ");
      Serial.println(Kd);
      */
    }


    ThrottleInputBits = EXTADC.analogRead(EXTADC_THROTTLE_PIN);
    CyborgAmpBits = EXTADC.analogRead(EXTADC_AM_PIN);
    CyborgFLBits = analogRead(EXTADC_CYBORGFIRSTLIMIT_PIN);

    CyborgTurbo = digitalRead(TURBO_PIN);

    ThrottleInputVolts = (float)ThrottleInputBits / (EXADC_BIT_CONVERSION / EXADC_VREF);

    CyborgAmpVolts = (float)CyborgAmpBits / (EXADC_BIT_CONVERSION / EXADC_VREF);
    CyborgAmps = ((CyborgAmpVolts - VMid) * 1000.0f) / mVPerAmp;

    CyborgFL = (float)CyborgFLBits / (EXADC_BIT_CONVERSION / EXADC_VREF);

    // input from the driver control pad
    CyborgFirstLimit = FloatMap(CyborgFL, 0.0f, 3.3f, CyborgMinRange, CyborgMaxRange);

    CyborgFirstLimit = (int)((CyborgFirstLimit + 0.05f) * 10.0f) / 10.0f;
    CyborgSetpoint = FloatMap(ThrottleInputVolts, ThrottleMinRange, ThrottleMaxRange, 0.0f, CyborgFirstLimit);

    if (ThrottleInputVolts >= ESCVoltsOff) {
      CyborgActive = true;
      Setpoint = CyborgSetpoint;
    } else {
      // throttle is off, this is a safeguard to put the setpoint way below the input to prevent the car
      // from taking off if in pit and amps get negative 10 used to speed motor shutdown
      Setpoint = Input - 10.0f;
      CyborgActive = false;
    }

    if (!CyborgTurbo) {
      // INPUT_PULLUP so off is high, on is low
      CyborgActive = false;
      Setpoint = CyborgSecondLimit;
    }

    // pass in  Input - which is the measured current or speed
    // pass in  Setpoint is either 1) first limit 2) second limit (turbo)
    // pass out Output which is the PWM signal (800 to 4096), CYBORG_LOWER_LIMIT to EXADC_BIT_CONVERSION

    // do we want to control AMPS or SPEED (for rare testing only and NOT racees)
    if (CyborgInput == CYBORG_CONTROL_AMPS) {
      Input = CyborgAmps;
    } else {
      Input = CarSpeed;
    }

    // fyi the PID object uses pointers for input, output and setpoint
    // PID CyborgPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

    CyborgPID.Compute();

    // now we have output
    // we only allow cyborg to work above a certian throttle value, otherwise reducing throttle
    // will not slow car since cyborg is managing esc input to maintain x amps
    // min throttle is around 0.6 volts
    if (CyborgActive) {
      CyborgOutputPWM = Output;
      CyborgInSignal = (ThrottleInputVolts * 100.0) / ThrottleMaxRange;
    } else if (ThrottleInputVolts < ESCVoltsOff) {
      CyborgActive = false;
      CyborgOutputPWM = 0;
      CyborgInSignal = 0.0f;
    }

    // bounds check
    if (CyborgOutputPWM > BIT_CONVERSION) {
      CyborgOutputPWM = BIT_CONVERSION;
    }
    ThrottleInputBits = 0;
    CyborgAmpBits = 0;
    CyborgFLBits = 0.0f;
    CyborgInSignal = FloatMap(CyborgInSignal, 0.0f, 100.0f, 0.0f, 100.0f);

    // send a PWM signal to the ESC, could be direct from the throttle, or a cyborg calculation
    analogWrite(OUTPUT_PIN, CyborgOutputPWM);

    if ((CyborgScreenUpdate > 200)) {
      // now get percentages for display and SSD storage and for saving to the SSD
      // weird mapping just--namely because the min voltage from the throttle is 0.6 or so
      CyborgScreenUpdate = 0;
      // since output PWM can be 0 we scale from 0
      CyborgOutSignal = map(CyborgOutputPWM, 0, EXADC_BIT_CONVERSION, 0, 100);

      // add check if old is more than some difference
      // show DisplayID 8

      if (abs(CyborgFirstLimit - OldCyborgFirstLimit) > 0.15) {
        OldCyborgFirstLimit = CyborgFirstLimit;
        DriverChangeScreen = 0;
        if (DisplayID != 8) {

          RedrawHeader = true;
          if (RestartDisplayAlways) {
            RestartDisplay();
          }
          Display.fillScreen(BackColor);
        }
        DisplayID = 8;
      }

      if (DisplayID == 8) {
        if (RedrawDisplay) {
          Display.fillScreen(BackColor);
        }

        CYBORGView();

        RedrawDisplay = false;
      }

      if ((DriverChangeScreen > 3000)) {
        if (OldDisplayID != DisplayID) {

          DisplayID = OldDisplayID;
          RedrawDisplay = true;
          DrawGraph = true;
        }
      }
    }
  }

  // end CYBORG calculations
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Check if we can update the display and them compute, send, and save
  if (DisplayUpdateTimer >= UPDATE_LIMIT) {

    WatchDogTimer(RESET_WDT);

    DisplayUpdateTimer = 0;

    // check external switch if high change to the other theme
    if (digitalRead(EXTADC_ENABLE_AME_PIN)) {
      Theme = 1;
    } else {
      Theme = 0;
    }
    if (OldTheme != Theme) {
      OldTheme = Theme;
      SetScreenParameters();
      RedrawDisplay = true;
      DrawGraph = true;
    }

    // check the accuracy of the RTC
    // if the RTC battey died, we should get a date/time but will out of sync with GPS data
    // if so, get the GPS time (when available and update the RTC)

    if ((NeedToUpdateTime) && (RaceStatus == RACE_NOTSTARTED)) {

      if (GPSSensor.location.isValid()) {
        // pick some values that will surely mean our time is wrong
        // not considering hour since day light savings can throw things off
        if ((abs(GPSSensor.date.day() - day()) > 3) || (abs(GPSSensor.time.minute() - minute()) > 5)) {
          SyncTimeFromGPS();
          NeedToUpdateTime = false;
          NeedToReplaceBattery = true;
          EEPROM.put(65, NeedToReplaceBattery);
        } else {
          NeedToUpdateTime = false;
          NeedToReplaceBattery = false;
          EEPROM.put(65, NeedToReplaceBattery);
        }
      }
    }

    ComputeData();

    ComputeSpeed();

    // stop race if longer than pre choosen race time
    // add some time to account for if GPUSA stops clock (red flag?).
    if ((RaceStatus == RACE_INPROGRESS) && ((CarRaceTimer / 1000) >= (RACE_TIME_SECONDS + RACE_EXTENSION))) {
      // set restoring data to 0
      RedrawHeader = true;
      // Cyborg First limit will have been biased, let's reset  it
      EEPROM.get(485, CyborgFirstLimit);
      ResetRaceDate();
      SaveStartGPS(false);
      RaceStatus = RACE_FINISHED;
      DriverTimeOK = false;
    }

    if (EXTADC.analogRead(EXTADC_KEY_PIN) > DIGITAL_ON_LIMIT) {
      KeyState = HIGH;
    } else {
      KeyState = LOW;
    }

    GetDriverPitTime();

    // call this before determining key states
    CheckIfStarting();

    // key states for banner color
    if ((KeyState == HIGH) && (DisplayOldKeyState == LOW)) {
      // key just turned on
      DisplayOldKeyState = HIGH;
      RedrawHeader = true;
      banner_back = C_DKRED;
    } else if ((KeyState == LOW) && (DisplayOldKeyState == HIGH)) {
      // key turned off
      DisplayOldKeyState = LOW;
      RedrawHeader = true;
      banner_back = C_DKGREEN;
    }

    // key states for determining if driver change is warranted
    if ((RaceStatus == RACE_INPROGRESS) && (DriverTimeOK)) {

      if (Driver == 0) {
        // special case for first driver--states already set
        if (KeyState == LOW) {
          OldKeyState = LOW;
          RestoreType = STATUS_PITSTOP1;
        }
        if ((KeyState == HIGH) && (!OldKeyState)) {
          KeyStatus = KEYSTATUS_TURNED_ON;
        }
      }

      if (Driver == 1) {
        if ((KeyState == HIGH) && (OldKeyState)) {
          RestoreType = STATUS_OK;
        }
        if (KeyState == LOW) {
          OldKeyState = LOW;
          RestoreType = STATUS_PITSTOP2;
        }
        if ((KeyState == HIGH) && (!OldKeyState)) {
          KeyStatus = KEYSTATUS_TURNED_ON;
        }
      }

      if (Driver == 2) {
        if ((KeyState == HIGH) && (!OldKeyState)) {
          RestoreType = STATUS_OK;
        }
      }
    }

    if (GPSLEDTimer > 2000) {
      digitalWrite(LAPLED_PIN, LOW);
    }

    if (RaceStatus == RACE_INPROGRESS) {
      if (!StartGPSFound) {
        GetStartGPS();
      }
    }

    RealClockTime = (hour() * 3600) + (minute() * 60) + second();

    if (RaceStatus == RACE_INPROGRESS) {

      if (GraphStoreTimer >= 60000l) {
        GraphStoreTimer = 0;
        EnergyXPoint = CarRaceTimer / 60000l;
        if (EnergyXPoint > 95) {
          EnergyXPoint = 95;
        }
        EnergyPoints[EnergyXPoint] = Energy;
      }

      // compute energy prediction but only if key is on
      if (KeyState == HIGH) {
        ComputePrediction();
      } else {
        PredictedEnergy = Energy;
      }

      // store the driver time
      // need to do this before display so race time and driver type synch up
      DriverTime[Driver] = DriverTimer;

      // see if we have passed the start point and if so trigger a lap
      if (GPSTolerance > 0) {
        CheckIfLap();
      }
      RecordType = RT_DATA;

      if (SSD.addRecord()) {
        SSD.saveRecord();
      }
    }

    if (KeyState == LOW) {
      Warnings = Warnings | KEY_OFF;
    }
    if (!GPSStatus) {
      Warnings = Warnings | GPS_WARNING;
    }
    if (RaceStatus == RACE_INPROGRESS) {
      Warnings = Warnings | RACE_START;
    }
    if (EnableCyborg) {
      if (!CyborgTurbo) {
        Warnings = Warnings | TURBO_STATUS;
      }
    }
    if (!SSDStatus) {
      Warnings = Warnings | SSD_FAIL;
    }
    if (!EXTADCStatus) {
      Warnings = Warnings | EXTADC_WARNING;
    }
    if (Volts < WARNING_BATTERY) {
      Warnings = Warnings | BAT_WARNING;
    }
    if (Amps > 25.0f) {
      Warnings = Warnings | AMP_WARNING;
    }
    if (LapAmps > 20.0f) {
      Warnings = Warnings | LAPAMP_WARNING;
    }
    if ((GForceY > 0.6f) || (!GForceStatus)) {
      Warnings = Warnings | GFORCE_WARNING;
    }
    if ((MotorTemp > WARNING_MTEMP) || (MotorTemp < 10.0f)) {
      Warnings = Warnings | TEMP_WARNING;
    }
    if ((AuxTemp > WARNING_MTEMP) || (AuxTemp < 10.0f)) {
      Warnings = Warnings | TEMP_WARNING;
    }

    if ((CyborgOutSignal >= 95) && (CyborgInSignal >= 99)) {
      Warnings = Warnings | TUNE_WARNING;
    }

    if ((EnableAirFlowSensor && !AirFlowSensorStatus) || !SpeedSensorStatus) {
      Warnings = Warnings | SPEED_WARNING;
    }

    // the DriverChange screen is only for when we have a new driver, show the welcome screen.
    if ((DriverChangeScreen > 3000)) {
      if (RedrawDisplay) {

        RedrawHeader = true;
        RedrawDisplay = false;
        // user option to restart the display every time or on screen change
        // may need every draw as display can show artifacts due to high electrical noise
        // note since we don't have MISO connected we can't read display status--hence restarting may be needed
        // works...but takes 120 ms to restart
        // if the display pinouts are ever changed on the PCB- you must send MOSI and GND on a twisted pair,
        // and Vcc and SCK on a twisted pair--otherwise the display will freak out
        // also using 100 ohm series resistors to offset wire capacitance

        if (RestartDisplayAlways) {
          RestartDisplay();
        }
        Display.fillScreen(BackColor);
      }

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
        case 8:
          //CYBORGView(); // we need faster updates so put call in main loop with a different timer
          break;
      }
      if (RaceStatus == RACE_INPROGRESS) {
        DrawPitMessage();
      } else {
        DrawWarnings();
      }
    }

    if (RadioUpdate != 0) {
      if (RadioUpdateTimer >= (RadioUpdate * 1000)) {
        RadioUpdateTimer = 0;
        SendData();
      }
    }


    // reset the counters
    vVolts = 0.0f;
    aVolts = 0.0f;
    thmVolts = 0.0f;
    thxVolts = 0.0f;
    Counter = 0;
    GPSStatus = false;

    Warnings = 0;
    GForceX = 0.0;
    GForceY = 0.0;
    GForceZ = 0.0;
  }
}

/*---------------------------------------------------------*/
//PRIMARY FUNCTIONS
/*---------------------------------------------------------*/

/*
  PURPOSE : If the RTC backup battery fails and Datalogger does not have current time, get time from GPS
  PARAMS :  -
  RETURNS : None
  NOTES : Called in loop() inside the display update section
*/


void SyncTimeFromGPS() {

  uint8_t TempH, TempD;
  // If the datalogger RTC is incorrect (if the CR2032 is dead), we wll get the time from the
  // GPS and adjust according to CST non daylight savings time
  // if the backup battery is dead, this will happen every boot up--which is fine but replace CR2032
  // I'll admit this is lame...guessing at the timezone
  // but we never travel outside CST and EST and let's ignore daylight savings
  // GPS will return GMT and we're 5 hours behind

  int cstHour = GPSSensor.time.hour() - 5;

  if (cstHour < 0) {
    TempH = 24 + cstHour;
    TempD = GPSSensor.date.day() - 1;
  } else {
    TempH = cstHour;
    TempD = GPSSensor.date.day();
  }

  // update datalogger RTC
  setTime(TempH, GPSSensor.time.minute(), GPSSensor.time.second(), TempD, GPSSensor.date.month(), GPSSensor.date.year());

  Teensy3Clock.set(now());
}

/*
  PURPOSE : Computes data for the car
  PARAMS :  -
  RETURNS : None
  NOTES : Called by loop()
*/

void ComputeSpeed() {

  if (RPMCount < MINIMUM_PULSES) {

    WRPM = 0;
    CarSpeed = 0.0f;
  } else {

#if defined(__MK20DX256__)  // Teensy 3.2
    WRPM = (60.0f / (float)Pickups) * RPMSensor.countToFrequency(RPMSum / RPMCount);
#else
    WRPM = 15.0f * FreqMeasure.countToFrequency(RPMSum / RPMCount);
#endif

    // compute the car speed
    // v = omega * r
    // v = velocity
    // omega is radians per time
    // r is tire radius
    // use formula and convert units of measure and convert rpm to radians
    CarSpeed = (WRPM * TireRad * 2.0f * 3.14159f * 60.0f / (12.0f * 5280.0f));
  }

  if ((WRPM > 4000.0f) || (WRPM < 0.0f)) {
    WRPM = 0.0f;
  }

  mRPM = WRPM * GearRatio;

  SpeedSensorStatus = true;
  if ((CarSpeed > 39.0f) || (CarSpeed < 0.0f)) {
    SpeedSensorStatus = false;
    CarSpeed = 0.0f;
  }

  // get the driven Distance in miles
  Distance = (Revolutions * TireRad * 2.0f * 3.1416f) / (12.0f * 5280.0f);

  RPMSum = 0.0f;
  RPMCount = 0.0f;

  // kris
  // some hardware / software issue creates jittery speed reads +/- 0.2 MPH, begin seems to address this
  // RPMStatus = RPMSensor.begin(RPM_PIN);

#if defined(__IMXRT1062__)  // Teensy 3.2
// RPMStatus = RPMSensor.begin(RPM_PIN);
#endif
}

void ComputeData() {

  Point++;

  // get the battey voltage
  vVolts = vVolts / Counter;

  vVolts = vVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);

  Volts = (vVolts * VoltageSlope) + VoltageOffset;

  if ((Volts > 99.0f) || (Volts < 0.0f)) {
    Volts = 0.0f;
  }

  // get current draw
  aVolts = aVolts / Counter;
  aVolts = aVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);

  Amps = ((aVolts - VMid) * 1000.0f) / mVPerAmp;

  if ((Amps > 199.0f) || (Amps < -99.0f)) {
    Amps = 0.0f;
  }

  thmVolts = thmVolts / Counter;
  thxVolts = thxVolts / Counter;

  // compute motor casing temperature
  // no need to average, just one read is fine
  thmVolts = thmVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);

  // voltage divider calculation
  // vo = 5 * r2 /(r1+r2)
  // solve for r2
  // get the exact value for voltage divider r2
  ThermistorResistence = (thmVolts * ThermResMotor) / (REFERENCE_VOLTAGE - thmVolts);
  //equation from data sheet
  TempK = 1.0f / (NTC_A + (NTC_B * (log(ThermistorResistence / 10000.0f))) + (NTC_C * pow(log(ThermistorResistence / 10000.0f), 2)) + (NTC_D * pow(log(ThermistorResistence / 10000.0f), 3)));
  MotorTemp = (TempK * 1.8f) - 459.67f;
  if ((MotorTemp > 299.0f) || (MotorTemp < 0.0f)) {
    MotorTemp = 0.0f;
  }

  // compute motor exhaust temperature
  // no need to average, just one read is fine
  thxVolts = thxVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);

  // voltage divider calculation
  // vo = 5 * r2 /(r1+r2)
  // solve for r2
  // get the exact value for voltage divider r1
  ThermistorResistence = (thxVolts * ThermResAux) / (REFERENCE_VOLTAGE - thxVolts);
  // equation from data sheet
  TempK = 1.0f / (NTC_A + (NTC_B * (log(ThermistorResistence / 10000.0f))) + (NTC_C * pow(log(ThermistorResistence / 10000.0f), 2)) + (NTC_D * pow(log(ThermistorResistence / 10000.0f), 3)));
  AuxTemp = (TempK * 1.8f) - 459.67f;
  if ((AuxTemp > 499.0f) || (AuxTemp < 0.0f)) {
    AuxTemp = 0.0f;
  }

  // compute Power
  Power = Volts * Amps;

  // compute Energy
  Energy = Energy + (Power * (UPDATE_LIMIT / 3600000.0f));

  // compute AmpHours
  AmpHours = AmpHours + (Amps * (UPDATE_LIMIT / 3600000.0f));

  if (Energy < 0) {
    Energy = 0;
  }

  // compute remaining Energy note total Energy is based on battery tests
  ERem = ((TotalEnergy - Energy) / TotalEnergy) * 100.0f;
  if (ERem < 0.0f) {
    ERem = 0.0f;
  }
  if (ERem > 100.0f) {
    ERem = 100.0f;
  }


  // altimiter

  // read time is 42 ms
  Altitude = 0.0f;
  AtmPressure = 0.0f;
  AmbTemp = 0.0f;
  AltimiterStatus = PressureSensor.isConnected();
  if (AltimiterStatus) {
    PressureSensor.read();
    Altitude = (PressureSensor.getAltitude() * METERS_TO_FEET) + AltCorrection;
    AtmPressure = PressureSensor.getPressure();
    AmbTemp = PressureSensor.getTemperature();
    AmbTemp = (AmbTemp * 1.8) + 32.0 + AmbTempCF;
  }

  // gps
  // do we have GPS?
  // is it connected (buffer filling) AND is the GPSLat & GPSLon valid
  GPSStatus = GPSSensor.location.isValid();
  GPSDistance = 0.0f;
  if (GPSStatus) {
    if (GPSTolerance > 0) {
      GPSLat = GPSSensor.location.lat();
      GPSLon = GPSSensor.location.lng();
      GPSSpeed = GPSSensor.speed.mph();
      GPSSatellites = GPSSensor.satellites.value();
      GPSAltitude = (GPSSensor.altitude.meters() * METERS_TO_FEET) + GPSAltCorrection;
      if (StartGPSFound) {
        GPSDistance = GPSSensor.distanceBetween(GPSStartLat, GPSStartLon, GPSLat, GPSLon);  // in meters
      }
    }
  } else {
    GPSLEDTimer = 0;
    digitalWrite(LAPLED_PIN, HIGH);
  }

  // accelerometer
  // read time is 1ms
  GForceStatus = AccelSensor.isConnected();
  GForceX = 0.0f;
  GForceY = 0.0f;
  GForceZ = 0.0f;
  GForce = 0.0f;

  if (GForceStatus) {
    // we consider Y + to be forward
    if (ASensorDirection == 0) {
      // +X
      GForceY = -(AccelSensor.getAccelerationX()) / ASensorBits;
      GForceX = AccelSensor.getAccelerationY() / ASensorBits;
      GForceZ = AccelSensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 1) {
      // -X
      GForceY = (AccelSensor.getAccelerationX()) / ASensorBits;
      GForceX = AccelSensor.getAccelerationY() / ASensorBits;
      GForceZ = AccelSensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 2) {
      // +Y
      GForceX = AccelSensor.getAccelerationX() / ASensorBits;
      GForceY = -(AccelSensor.getAccelerationY()) / ASensorBits;
      GForceZ = AccelSensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 3) {
      // -Y
      GForceX = AccelSensor.getAccelerationX() / ASensorBits;
      GForceY = (AccelSensor.getAccelerationY()) / ASensorBits;
      GForceZ = AccelSensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 4) {
      // +Z
      GForceX = AccelSensor.getAccelerationX() / ASensorBits;
      GForceY = AccelSensor.getAccelerationY() / ASensorBits;
      GForceZ = AccelSensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 5) {
      // -Z
      GForceX = AccelSensor.getAccelerationX() / ASensorBits;
      GForceY = -(AccelSensor.getAccelerationY()) / ASensorBits;
      GForceZ = AccelSensor.getAccelerationZ() / ASensorBits;
    }
    GForceX = ((int16_t)(GForceX * 100.0f)) / 100.0f;
    GForceY = ((int16_t)(GForceY * 100.0f)) / 100.0f;
    GForceZ = ((int16_t)(GForceZ * 100.0f)) / 100.0f;
    GForce = sqrt((GForceX * GForceX) + (GForceY * GForceY) + (GForceZ * GForceZ));
    GForce = ((int16_t)(GForce * 100.0f)) / 100.0f;
    if (abs(GForceY) > TempGForceYLapMaxL) {
      TempGForceYLapMaxL = abs(GForceY);
    }

    if (abs(GForceY) > TempGForceYLapMaxR) {
      TempGForceYLapMaxR = abs(GForceY);
    }
  }

  AirFlowSensorStatus = false;
  // reads are 8ms
  if (EnableAirFlowSensor) {
    AirFlowSensorStatus = AirFlowSensor.readSensor(ASTemp, ASPressure);
    if (AirFlowSensorStatus) {
      // sensor is giving pascals
      // need to convert to inches of water
      ASPressure = ASPressure * 0.00401865f;
      if (ASPressure < 0.0f) {
        ASPressure = 0;
      }
      // this equation needs inches of water
      fpm = sqrt(ASPressure) * 4005.0f;
      // convert feet per minute to miles per hour
      AirSpeed = (fpm * 0.0113636f) + AirSpeedOffset;
      if (AirSpeed < 0.0f) {
        AirSpeed = 0.0f;
      }
    }
  }

  // build averages
  AverageCount++;
  AverageAmps = AverageAmps + Amps;
  AverageVolts = AverageVolts + Volts;
  AverageCarSpeed = AverageCarSpeed + CarSpeed;

  if (RaceStatus == RACE_INPROGRESS) {
    TRem = ((RACE_TIME_SECONDS - (CarRaceTimer / 1000.0f)) / RACE_TIME_SECONDS) * 100.0f;
    if (TRem < 0.0f) {
      // End of race
      TRem = 0.0f;
    }
  }
}

/*
  PURPOSE : ComputePrediction
  PARAMS :  -
  RETURNS : None
  NOTES : uses linear regression to predict total energy consumption
*/
void ComputePrediction() {
  if (PredictionArrayCounter < PREDICT_SAMPLES) {
    EnergyArray[PredictionArrayCounter] = Energy;
    PointArray[PredictionArrayCounter] = Point;
    PredictionArrayCounter++;
  } else {
    // shift the array
    memmove(&EnergyArray[0], &EnergyArray[1], sizeof(float) * (PREDICT_SAMPLES - 1));
    memmove(&PointArray[0], &PointArray[1], sizeof(float) * (PREDICT_SAMPLES - 1));
    EnergyArray[PREDICT_SAMPLES - 1] = Energy;
    PointArray[PREDICT_SAMPLES - 1] = Point;

    sum_y = 0;
    sum_xx = 0;
    sum_x = 0;
    sum_xy = 0;

    for (iCP = 0; iCP < PREDICT_SAMPLES; iCP++) {
      sum_y += EnergyArray[iCP];
      sum_xx += (PointArray[iCP] * PointArray[iCP]);
      sum_x += PointArray[iCP];
      sum_xy += (EnergyArray[iCP] * PointArray[iCP]);
    }

    den = (((PREDICT_SAMPLES)*sum_xx) - (sum_x * sum_x));
    c1 = (sum_y * sum_xx);
    c2 = (sum_x * sum_xy);
    c3 = (PREDICT_SAMPLES)*sum_xy;
    c4 = sum_x * sum_y;

    b_PE = (c1 - c2) / den;
    m_PE = (c3 - c4) / den;

    // use offset and slope to compute energy at 90 min (90min * 2 points/sec * 60 sec)
    PredictedEnergy = m_PE * 10800.0f + b_PE;

    // this compensator will attempt to compensate for the non linear nature of the energy curve
    // the farther we are from 90 min the more we deduct. Race data shows about 70 wHr at t=90
    PredictedEnergy = PredictedEnergy - (PredictionCompensation * ((float)Point / 10800.0f));

    //if (PredictedEnergy < Energy) {
    //  PredictedEnergy = Energy;
    // }
  }
#ifdef LR_DEBUG
  Serial.print("Pnt ");
  Serial.print(Point);
  Serial.print(", NRG ");
  Serial.print(Energy);
  Serial.print(", sum_y ");
  Serial.print(sum_y, 2);
  Serial.print(", sum_xx ");
  Serial.print(sum_xx);
  Serial.print(", sum_x ");
  Serial.print(sum_x);
  Serial.print(", sum_xy ");
  Serial.print(sum_xy);
  Serial.print(", den ");
  Serial.print(den, 0);
  Serial.print(", c1 ");
  Serial.print(c1);
  Serial.print(", c2 ");
  Serial.print(c2);
  Serial.print(", c3 ");
  Serial.print(c3);
  Serial.print(", c4 ");
  Serial.print(c4);
  Serial.print(", b_PE ");
  Serial.print(b_PE, 4);
  Serial.print(", m_PE ");
  Serial.print(m_PE, 4);
  Serial.print(", PE ");
  Serial.println(PredictedEnergy);
#endif
}

/*
   PURPOSE : get start GPS but only if we have data
   PARAMS :  -
   RETURNS : None
   NOTES : checked every update
*/

void GetStartGPS() {
  if (StartGPSDelayTimer > StartGPSDelay) {
    if (GPSSensor.location.isValid()) {
      GPSStatus = true;
      SaveStartGPS(true);
    }
  }
}

/*
   PURPOSE : check if race has started or driver change
   PARAMS :  -
   RETURNS : None
   NOTES : checked every boot up
*/

void CheckIfStarting() {
  if (RaceStatus == RACE_FINISHED) {
    return;
  }

  if (RaceStatus == RACE_NOTSTARTED) {
    // we don't look for key here as the car will not start if key is off
    // and we don't care what they key state is befor a race
    // basically, if amps go up. race has started
    if (Amps >= TriggerAmps) {
      // test if we are starting from beginning no matter what if amps goes high start the race
      // this handles driver 0
      ShowNewDriverScreen();
      RaceStatus = RACE_INPROGRESS;
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
      delay(10);
      // here is where we add a new record, write header data, called only once when race starts
      AddNewRecordset();

      // "zero" out all key data fields
      Point = 0;
      Driver = 0;
      PitTime = 0;
      GPSLapTimer = 0;
      AverageCount = 0;
      AverageAmps = 0.0f;
      AverageCarSpeed = 0.0f;
      AverageVolts = 0.0f;
      Distance = 0.0f;
      LapCount = 0;
      Energy = 0.0f;
      LapEnergy = 0.0f;
      StartLapEnergy = 0.0f;
      DriverTimer = 0;
      CarRaceTimer = 0;
      LapTimer = 0;

      DriverLaps[0] = 0;
      DriverLaps[1] = 0;
      DriverLaps[2] = 0;

      DriverTime[0] = 0;
      DriverTime[1] = 0;
      DriverTime[2] = 0;

      StartGPSDelayTimer = 0;
      RedrawHeader = true;
      DriverTimeOK = false;
      RestoreType = STATUS_OK;
      OldKeyState = HIGH;
      KeyStatus = KEYSTATUS_NO_CHANGE;
    }
  } else {
    // race has started
    // now we're more diligent if a driver change
    // currrent will be all over the map so can't rely on just that
    // this handles driver 1 and 2

    if ((Amps >= TriggerAmps) && (KeyStatus == KEYSTATUS_TURNED_ON) && (DriverTimeOK)) {
      RestoreType = STATUS_OK;
      RaceStatus = RACE_INPROGRESS;
      KeyStatus = KEYSTATUS_NO_CHANGE;
      OldKeyState = HIGH;
      // based on rule check in DrawPitMessage()
      // if race is running and driver time is within limit and key just turned on

      if (Driver < 2) {
        ChangeDriver();
        ShowNewDriverScreen();
      }
    }
  }
}

/*
   PURPOSE : cute screen to let driver/pit know when driver change
   PARAMS :  -
   RETURNS : None
   NOTES : called in racehasstarted or when driver change is detected
*/

void ShowNewDriverScreen() {
  // show a cute splash screen
  //Draw new driver welcome screen
  Display.fillScreen(BackColor);

  Display.setCursor(30, 30);
  Display.setFont(FONT_24BI);
  Display.setTextColor(ForeColor, BackColor);
  Display.print(F("Driver:"));
  Display.setFont(FONT_100BINO);
  Display.setTextColor(ForeColor, BackColor);
  Display.setCursor(140, 100);
  Display.print(Driver + 1);

  // start the screen timer
  RedrawHeader = true;
  RedrawDisplay = true;
  DriverChangeScreen = 0;
  DrawGraph = true;
}

/*
   PURPOSE : resets a race, to a new race
   PARAMS :  -
   RETURNS : None
   NOTES : called manually at boot up (press l or r button) or end of race
*/
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

/*
   PURPOSE : we have valid start GPS coordinates
   PARAMS :  -
   RETURNS : None
   NOTES : called if race starte and we don't have start gps
*/
void SaveStartGPS(bool Action) {
  if (Action) {
    if (GPSStatus) {
      GPSStartLat = GPSLat;
      GPSStartLon = GPSLon;
      StartGPSFound = true;
      // need to get
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

/*
   PURPOSE : add a special header record
   PARAMS :  -
   RETURNS : None
   NOTES : called once at start of race
*/

void AddNewRecordset() {
  Tyear = year();
  Tmonth = month();
  Tday = day();
  Thour = hour();
  Tminute = minute();
  RecordType = RT_HEADER;

  // set the next RecordsetID
  RecordSETID++;

  AverageCounter = Counter;

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
  PARAMS :  None
  RETURNS : None
  NOTES : Data logger, repeaters, and wifi server must match these bit shifts
  to save packet size we play some games... we don't need 6 decimal places for volts
  we only need one, so multiply volts * 10 and cast to uint16_t then we clamp max (we'll never
  above 28 volts (280) so max is 512 or 9 bits
  this scheme allows us to compress tons of data into a very small packet

  future: better handle rounding errors 23.999999 volts should be 24, (just add 0.05 to 1 decimal places,
  0.5 to no decimals, etc.) 
*/
void SendData() {
  // datalogger Device ID is 0, for all cars, repeaters are 1-3 hence LSB 0 and 1 are both 0
  Data.ENERGY_DNO_DID = (((uint16_t)Energy & 0b0000001111111111) << 4) | ((Driver & 0b0000000000000011) << 2);
  Data.ENERGY_DNO_DID = Data.ENERGY_DNO_DID & 0b1111111111111100;
  // we're only sending 6 bits of the warnings, hence we send the most important
  Data.WARNINGS_PE = ((uint16_t)(Warnings) << 10) | ((uint16_t)(PredictedEnergy)&0b0000001111111111);
  Data.TEMPF_TEMPX = ((uint16_t)MotorTemp << 8) | (((uint16_t)AuxTemp) & 0b0000000011111111);
  Data.VOLTS_LAPS = ((uint16_t)(Volts * 10.0f)) << 7 | (((uint16_t)LapCount) & 0b0000000001111111);
  Data.SPEED_EREM = ((uint16_t)(CarSpeed * 10.0f)) << 7 | (((uint16_t)ERem) & 0b0000000001111111);
  Data.DISTANCE_TREM = ((uint16_t)(Distance * 10.0f)) << 7 | (((uint16_t)TRem) & 0b0000000001111111);
  // total energy will never be less than 550 with a max of 800 this means we have to send 250 divide by 2 will fit 7 bits
  Data.TWHR_LAPAMPS = (uint16_t)(((TotalEnergy - 550) / 2) << 9) | ((uint16_t)(LapAmps * 10.0f) & 0b0000000111111111);
  Data.AMPS_D0TIME = ((uint16_t)(abs(Amps) * 10.0f)) << 5 | (((uint16_t)(DriverTime[0] / 1000)) & 0b0000111111111111) >> 7;
  if (Amps < 0) {
    Data.AMPS_D0TIME = Data.AMPS_D0TIME | 0b1000000000000000;
  } else {
    Data.AMPS_D0TIME = Data.AMPS_D0TIME & 0b0111111111111111;
  }
  Data.LAP2AMPS_D0TIME = ((uint16_t)(abs(TargetAmps) * 10.0f)) << 7 | (((uint16_t)(DriverTime[0] / 1000) & 0b0000000001111111));

  Data.RPM_CYBORGIN = ((uint16_t)(mRPM / 20.0f)) << 7 | (((uint16_t)CyborgInSignal) & 0b0000000001111111);
  Data.D1TIME_GFORCEY = (((uint16_t)(DriverTime[1] / 1000) & 0b0000111111111111) << 4) | ((uint16_t)(abs(GForceY) * 100.0f) & 0b0000000001111111) >> 4;
  if (GForceY < 0) {
    Data.D1TIME_GFORCEY = Data.D1TIME_GFORCEY | 0b0000000000001000;
  }
  Data.D2TIME_GFORCEY = ((uint16_t)(DriverTime[2] / 1000) & 0b0000111111111111) << 4 | ((uint16_t)(abs(GForceY) * 100.0f) & 0b0000000000001111) >> 4;
  Data.ALTITUDE_SID = ((uint16_t)(Altitude)&0b0000111111111111) << 2;
  //repeaters will use source id and sender id, data logger is 0 for both
  Data.ALTITUDE_SID = Data.ALTITUDE_SID & 0b1111111111111100;
  if (RaceStatus == RACE_NOTSTARTED) {
    Data.RACETIME_LAPENERGY = (uint16_t)(0);  // data stored in s
  } else {
    Data.RACETIME_LAPENERGY = (uint16_t)(CarRaceTimer / 1000) << 3;
  }
  Data.RACETIME_LAPENERGY = Data.RACETIME_LAPENERGY | (((uint16_t)(LapEnergy * 10.0f) & 0b0000000111111111) >> 6);  // data stored in ms
  Data.LT_LAPENERGY = ((uint16_t)(LapTime) << 6) | (((uint16_t)(LapEnergy * 10.0f) & 0b0000000000111111));          // data stored in ms
  Data.CYBORGOUT_CYBORGLO = (((uint16_t)(CyborgOutSignal)&0b0000000001111111) << 9) | (((uint16_t)(CyborgFirstLimit * 10.0f) & 0b0000000111111111));
  Data.LAT = GPSLat;
  Data.LON = GPSLon;
  DataPacket.sendData();

  // this radio seems to drop the voltage or adds enough noise to mess up the ADC VREF--should have put an external 3.0 v ref.
  // small delay may help system stabilize
  // SmartDelay(50);
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
  bool RestoreEEPROM = false;

  ButtonPressed = WhatButtonWasPressed();

  if ((ButtonPressed == L_BUTTON) || (ButtonPressed == R_BUTTON)) {

    Display.setRotation(1);
    Display.fillScreen(C_RED);
    Display.setFont(FONT_16B);
    Display.setTextColor(C_WHITE, BackColor);
    Display.setCursor(10, 30);
    Display.print(F("Resetting Race"));

    WaitForRelease();

    ResetRaceDate();
    SaveStartGPS(false);
  }
  // if unprogrammed or user want's to reset
  // could be due to corrupted eeprom data or added parameter and eeprom has some old data
  else if ((ButtonPressed == C_BUTTON_LONG)) {
    Display.setRotation(1);
    Display.fillScreen(C_RED);
    Display.setFont(FONT_16B);
    Display.setTextColor(C_WHITE, BackColor);
    Display.setCursor(10, 30);
    Display.print(F("Are you sure you want"));
    Display.setCursor(10, 60);
    Display.print(F("to restore all settings?"));
    Display.setCursor(10, 110);
    Display.print(F("Press Left or Right to cancel"));
    Display.setCursor(10, 140);
    Display.print(F("Press Center to continue"));

    WaitForRelease();

    ButtonPressed = NO_BUTTON;
    while (1) {
      ButtonPressed = WhatButtonWasPressed();
      if (ButtonPressed == C_BUTTON) {
        RestoreEEPROM = true;
        break;
      } else if ((ButtonPressed == L_BUTTON) || (ButtonPressed == R_BUTTON)) {
        RestoreEEPROM = false;
        break;
      }
    }
  }
  if (RestoreEEPROM) {
    Display.setCursor(10, 170);
    Display.print(F("Restoring settings"));

    // new programmer reset the whole eeprom
    for (i = 0; i < 600; i++) {
      EEPROM.put(i, 0);
      delay(10);
    }

#ifdef DO_DEBUG
    Serial.println(F("Resetting EEPROM data"));
#endif

    // now set some defaults
    /* we do not store these variables in memory as they are only used when calibrating sensors
        or when downloading the settings to a text file
        ...hence no need to consume memory for rare usage
        adding here as a reminder to NOT use these addresses
        VoltSensorCalibrationDate = 0;
        EEPROM.put(2, VoltSensorCalibrationDate);
        AmpSensorCalibrationDate = 0;
        EEPROM.put(4, AmpSensorCalibrationDate);
        TempSensorCalibrationDate = 0;
        EEPROM.put(6, TempSensorCalibrationDate);
    */

    MotorSprocket = 15;
    EEPROM.put(10, MotorSprocket);
    WheelSprocket = 70;
    EEPROM.put(20, WheelSprocket);
    CyborgInput = CYBORG_CONTROL_AMPS;
    EEPROM.put(25, CyborgInput);
    TireID = 0;
    EEPROM.put(30, TireID);
    TirePressureFront = 115;
    EEPROM.put(35, TirePressureFront);
    TirePressureRear = 115;
    EEPROM.put(36, TirePressureRear);
    Theme = 0;
    EEPROM.put(40, Theme);
    AltCorrection = 0;
    EEPROM.put(45, AltCorrection);
    GPSAltCorrection = 0;
    EEPROM.put(47, GPSAltCorrection);
    Orientation = 1;
    EEPROM.put(50, Orientation);
    AutoCurrentCal = true;
    EEPROM.put(55, AutoCurrentCal);
    RadioUpdate = 1;
    EEPROM.put(60, RadioUpdate);
    NeedToReplaceBattery = false;
    EEPROM.put(65, NeedToReplaceBattery);
    TotalEnergy = 600;
    EEPROM.put(70, TotalEnergy);
    ThrottleMinRange = 0.6f;
    EEPROM.put(75, ThrottleMinRange);
    ThrottleMaxRange = 3.25f;
    EEPROM.put(80, ThrottleMaxRange);
    VoltageSlope = 11.0;
    EEPROM.put(110, VoltageSlope);
    VoltageOffset = 0.30;
    EEPROM.put(120, VoltageOffset);
    MotorID = 0;
    EEPROM.put(130, MotorID);
    LapThreshold = 30;
    EEPROM.put(160, LapThreshold);
    Battery1 = 0;
    EEPROM.put(170, Battery1);
    Battery2 = 0;
    EEPROM.put(175, Battery2);
    AddLapInPit = true;
    EEPROM.put(185, AddLapInPit);
    StartGPSDelayID = 0;
    EEPROM.put(190, StartGPSDelayID);
    AccelLPFilter = 0;
    EEPROM.put(200, AccelLPFilter);
    AccelHPFilter = 0;
    EEPROM.put(205, AccelHPFilter);
    Pickups = 4;
    EEPROM.put(210, Pickups);
    mVPerAmp = 20.0f;
    EEPROM.put(220, mVPerAmp);
    VMid = .5f;
    EEPROM.put(230, VMid);
    RestartDisplayAlways = false;
    EEPROM.put(240, RestartDisplayAlways);
    GForceRange = 0;
    EEPROM.put(250, GForceRange);
    ASensorDirection = 0;
    EEPROM.put(275, ASensorDirection);
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
    TriggerAmps = 15.0f;
    EEPROM.put(380, TriggerAmps);
    ESCVoltsOn = 3.2f;
    EEPROM.put(385, ESCVoltsOn);
    ESCVoltsOff = 0.8f;
    EEPROM.put(390, ESCVoltsOff);
    RaceMonth = 0;
    EEPROM.put(400, RaceMonth);
    EnableAirFlowSensor = false;
    EEPROM.put(410, EnableAirFlowSensor);
    AirSpeedOffset = false;
    EEPROM.put(412, AirSpeedOffset);
    AccelCalX = -1800;
    EEPROM.put(450, AccelCalX);
    AccelCalY = -4400;
    EEPROM.put(460, AccelCalY);
    AccelCalZ = 720;
    EEPROM.put(470, AccelCalZ);
    AmbTempCF = 0.0f;
    EEPROM.put(480, AmbTempCF);

    //CYBORG
    CyborgFirstLimit = 18.0f;
    EEPROM.put(485, CyborgFirstLimit);
    EnableCyborg = false;
    EEPROM.put(490, EnableCyborg);
    CyborgUpdateTime = 100;
    EEPROM.put(495, CyborgUpdateTime);
    Kp = 5.0f;
    EEPROM.put(500, Kp);
    Ki = 500.0f;
    EEPROM.put(510, Ki);
    Kd = 0.0f;
    EEPROM.put(520, Kd);
    PredictionCompensation = 86.0f;
    EEPROM.put(530, PredictionCompensation);
    CyborgSecondLimit = 25.0;
    EEPROM.put(535, CyborgSecondLimit);

    Display.setCursor(10, 200);
    Display.print(F("Restoring complete."));
    delay(1000);
    Display.fillScreen(C_BLACK);
  }

#ifdef DO_DEBUG
  Serial.println(F("Getting EEPROM data"));
#endif
  /* reminder to not use these we only set in calibration get when writing settings file
  EEPROM.get(2, VoltCalibrationDate);
  EEPROM.get(4, CurrentCalibrationDate);
  EEPROM.get(6, TemperatureCalibrationDate);
  */
  EEPROM.get(10, MotorSprocket);
  EEPROM.get(20, WheelSprocket);
  EEPROM.get(25, CyborgInput);
  EEPROM.get(30, TireID);
  EEPROM.get(35, TirePressureFront);
  EEPROM.get(36, TirePressureRear);
  EEPROM.get(40, Theme);
  EEPROM.get(45, AltCorrection);
  EEPROM.get(47, GPSAltCorrection);
  EEPROM.get(50, Orientation);
  EEPROM.get(55, AutoCurrentCal);
  EEPROM.get(60, RadioUpdate);
  EEPROM.get(65, NeedToReplaceBattery);
  EEPROM.get(70, TotalEnergy);
  EEPROM.get(75, ThrottleMinRange);
  EEPROM.get(80, ThrottleMaxRange);
  EEPROM.get(110, VoltageSlope);
  EEPROM.get(120, VoltageOffset);
  EEPROM.get(130, MotorID);
  EEPROM.get(160, LapThreshold);
  EEPROM.get(170, Battery1);
  EEPROM.get(175, Battery2);
  EEPROM.get(185, AddLapInPit);
  EEPROM.get(190, StartGPSDelayID);
  EEPROM.get(200, AccelLPFilter);
  EEPROM.get(205, AccelHPFilter);
  EEPROM.get(210, Pickups);
  EEPROM.get(220, mVPerAmp);
  EEPROM.get(230, VMid);
  EEPROM.get(240, RestartDisplayAlways);
  EEPROM.get(250, GForceRange);
  EEPROM.get(275, ASensorDirection);
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
  EEPROM.get(385, ESCVoltsOn);
  EEPROM.get(390, ESCVoltsOff);
  EEPROM.get(400, RaceMonth);
  EEPROM.get(410, EnableAirFlowSensor);
  EEPROM.get(412, AirSpeedOffset);
  EEPROM.get(450, AccelCalX);
  EEPROM.get(460, AccelCalY);
  EEPROM.get(470, AccelCalZ);
  EEPROM.get(480, AmbTempCF);
  EEPROM.get(485, CyborgFirstLimit);
  EEPROM.get(490, EnableCyborg);
  EEPROM.get(495, CyborgUpdateTime);
  EEPROM.get(500, Kp);
  EEPROM.get(510, Ki);
  EEPROM.get(520, Kd);
  EEPROM.get(530, PredictionCompensation);
  EEPROM.get(535, CyborgSecondLimit);

  OldCyborgFirstLimit = CyborgFirstLimit;
  OldTheme = Theme;
  OldDisplayID = DisplayID;

  // bounds check the ID to ensure it's not past array bounds
  if (ASensorDirection > (sizeof(ASensorDirectionText) / sizeof(ASensorDirectionText[0]))) {
    ASensorDirection = 0;
  }

  SetCyborgSetpointLimits();

  GetGearParameters();

  SetupAccelerometer();

  SetupGPS();

#ifdef DO_DEBUG
  Serial.println(F("******* EEPROM Parameters *******"));
  Serial.print(F("Voltage Slope: "));
  Serial.println(VoltageSlope);
  Serial.print(F("Voltage Offset: "));
  Serial.println(VoltageOffset);
  Serial.print(F("VMid: "));
  Serial.println(VMid);
  Serial.print(F("mVPerAmp: "));
  Serial.println(mVPerAmp);
  Serial.print(F("Gear Ratio: "));
  Serial.println(GearRatio);
  Serial.print(F("Motor Sprocket: "));
  Serial.println(MotorSprocket);
  Serial.print(F("Wheel Sprocket: "));
  Serial.println(WheelSprocket);
  Serial.print(F("Motor ID: "));
  Serial.println(MotorID);
  Serial.print(F("Tire ID: "));
  Serial.println(TireID);
  Serial.print(F("Tire type: "));
  Serial.println(TireText[TireID]);
  Serial.print(F("Tire Rad: "));
  Serial.println(TireRadius[TireID]);
  Serial.print(F("Theme: "));
  Serial.println(Theme);
  Serial.print(F("Orientation: "));
  Serial.println(Orientation);
  Serial.print(F("Update: "));
  Serial.println(DisplayUpdateTimer);
  Serial.print(F("Total Energy: "));
  Serial.println(TotalEnergy);
  Serial.print(F("Battery1: "));
  Serial.println(Battery1);
  Serial.print(F("Battery2: "));
  Serial.println(Battery2);
  Serial.print(F("Pickups: "));
  Serial.println(Pickups);
  Serial.print(F("TriggerAmps: "));
  Serial.println(TriggerAmps);
  Serial.print(F("Lap Threshold: "));
  Serial.println(LapThreshold);
  Serial.print(F("CarID: "));
  Serial.println(CarID);
  Serial.print(F("GPSTolerance: "));
  Serial.println(GPSTolerance);
  Serial.print(F("Temp Motor Res: "));
  Serial.println(ThermResMotor);
  Serial.print(F("Temp Aux Res: "));
  Serial.println(ThermResAux);
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
  Serial.print(F("AutoCurrentCal: "));
  Serial.println(AutoCurrentCal);
  Serial.print(F("RadioUpdate: "));
  Serial.println(RadioUpdate);
  Serial.print(F("AddLapInPit: "));
  Serial.println(AddLapInPit);
  Serial.print(F("StartGPSDelayID: "));
  Serial.println(StartGPSDelayID);
  Serial.print(F("AccelLPFilter: "));
  Serial.println(AccelLPFilter);
  Serial.print(F("AccelHPFilter: "));
  Serial.println(AccelHPFilter);
  Serial.print(F("RestartDisplayAlways: "));
  Serial.println(RestartDisplayAlways);
  Serial.print(F("GForceRange: "));
  Serial.println(GForceRange);
  Serial.print(F("ASensorDirection: "));
  Serial.println(ASensorDirection);
  Serial.print(F("DisplayID: "));
  Serial.println(DisplayID);
  Serial.print(F("AccelCalX: "));
  Serial.println(AccelCalX);
  Serial.print(F("AccelCalY: "));
  Serial.println(AccelCalY);
  Serial.print(F("AccelCalZ: "));
  Serial.println(AccelCalZ);
  Serial.print(F("AmbTempCF: "));
  Serial.println(AmbTempCF);
  Serial.print(F("EnableCyborg: "));
  Serial.println(EnableCyborg);
  Serial.print(F("ESCVoltsOn: "));
  Serial.println(ESCVoltsOn);
  Serial.print(F("CyborgFirstLimit: "));
  Serial.println(CyborgFirstLimit);
  Serial.print(F("CyborgSecondLimit: "));
  Serial.println(CyborgSecondLimit);
  Serial.print(F("CyborgUpdateTime: "));
  Serial.println(CyborgUpdateTime);
  Serial.print(F("Kp: "));
  Serial.println(Kp);
  Serial.print(F("Ki: "));
  Serial.println(Ki);
  Serial.print(F("Kd: "));
  Serial.println(Kd);

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
  uint16_t StatusBarWidth = 0;
  uint32_t DataRecord = 0;
  uint32_t TempTime;
  uint32_t TempDriver0 = 0, TempDriver1 = 0, TempDriver2 = 0, ii = 0;
  uint32_t StartRecord = 0, LastRecord = 0;
  uint32_t EnergyPointCounter = 0;
  uint8_t RecordsToBackup = 1;
  uint32_t TempDriver = 0;
  float InitalEnergy = 0.0f;
  uint32_t StatusBarCounter = 0;

  // get the last known record and read the data

  Display.setTextColor(C_CYAN);
  Display.setCursor(STATUS_RESULT, 200);
  Display.print(F("Reading: "));

  // go to the last record and get the data
  // note that records are storded 1 based so record 10 records is stored and recalled as 10
  // if power was lost during a record write, we will not have a full record so be save and back up 2 records

  LastRecord = SSD.getLastRecord();
  if ((LastRecord == 0) || (LastRecord == NULL_RECORD)) {
    return RR_ERROR;
  }

  StartRecord = SSD.getFirstRecord(RecordSETID, hrID);
  if ((StartRecord == 0) || (StartRecord == NULL_RECORD)) {
    return RR_ERROR;
  }

  DataRecord = LastRecord - RecordsToBackup;
  SSD.gotoRecord(DataRecord);
  RecordType = SSD.getField(RecordType, frType);
  if ((RecordType == RT_HEADER) || (RecordType == NULL_RECORD)) {
    return RR_ERROR;
  }

  RecordSETID = SSD.getField(RecordSETID, frID);
  // now rip through and get driver times
  // first get first record for the desire recordsetID
  // cant' really have 30 races, even if simple tests
  if ((RecordSETID > 30) || (RecordSETID == NULL_RECORD)) {
    return RR_ERROR;
  }

  // get last known race time
  TempTime = SSD.getField(RealClockTime, frRT);

#ifdef DO_DEBUG
  Serial.println("Line 2269 _________________________");
  Serial.print("StartRecord ");
  Serial.println(StartRecord);
  Serial.print("LastRecord ");
  Serial.println(LastRecord);
  Serial.print("RecordSETID ");
  Serial.println(RecordSETID);
  Serial.print("RecordType ");
  Serial.println(RecordType);
  Serial.print("Last known race time ");
  Serial.println(TempTime);
#endif

  // now go to the data record and get race data (this data will be used during restore)
  SSD.gotoRecord(DataRecord);
  RecordType = SSD.getField(RecordType, frType);
  Point = SSD.getField(Point, frPoint);
  LapCount = SSD.getField(LapCount, frLap);
  Driver = SSD.getField(Driver, frDriver);
  Energy = SSD.getField(Energy, frEnergy);
  AmpHours = SSD.getField(AmpHours, frAmpHours);
  Distance = SSD.getField(Distance, frDist);
  TempTime = SSD.getField(RealClockTime, frRT);
  GPSLon = SSD.getField(GPSLon, frLon);
  GPSLat = SSD.getField(GPSLat, frLat);
  Altitude = SSD.getField(Altitude, frAltitude);
  AmbTemp = SSD.getField(AmbTemp, frAmbTemp);

  // get Revolutions so distance calculations will be restored
  Revolutions = (Distance * 12.0f * 5280.0f) / (TireRad * 2.0f * 3.1416f);

  // we zero out these
  Volts = 0.0;
  Amps = 0.0;
  MotorTemp = 0.0;
  AuxTemp = 0.0;
  mRPM = 0;
  CarSpeed = 0.0;
  GPSSpeed = 0.0;
  GForceX = 0.0;
  GForceY = 0.0;
  GForceZ = 0.0;
  CyborgInSignal = 0;
  CyborgOutSignal = 0;

  // now get the start lat and long
  EEPROM.get(340, GPSStartLat);
  EEPROM.get(350, GPSStartLon);

  TempDriver0 = 0;
  TempDriver1 = 0;
  TempDriver2 = 0;

  EnergyPointCounter = 0;
  InitalEnergy = Energy;

  for (ii = StartRecord; ii <= LastRecord; ii++) {
    SSD.gotoRecord(ii);
    Point = SSD.getField(Point, frPoint);

    // this will build know data from the record
    if (Point % ((60 * 1000) / UPDATE_LIMIT) == 0) {
      if (EnergyPointCounter < 100) {
        Energy = SSD.getField(Energy, frEnergy);
        EnergyPoints[EnergyPointCounter] = Energy;
        EnergyPointCounter++;
      }
    }

    TempDriver = SSD.getField(Driver, frDriver);
    if (SSD.getField(RecordSETID, frID) == RecordSETID) {
      if (0 == TempDriver) {
        TempDriver0++;
      }
      if (1 == TempDriver) {
        TempDriver1++;
      }
      if (2 == TempDriver) {
        TempDriver2++;
      }
    }
  }
  // in case our energy trend graph has an issue,
  // restore energy with last saved
  Energy = InitalEnergy;

  SSD.gotoRecord(DataRecord);
  Driver = SSD.getField(Driver, frDriver);

  // we need to account for how many points per second
  // then convert to millis
  DriverTime[0] = TempDriver0 * UPDATE_LIMIT;
  DriverTime[1] = TempDriver1 * UPDATE_LIMIT;
  DriverTime[2] = TempDriver2 * UPDATE_LIMIT;

#ifdef DO_DEBUG
  Serial.println("Line 2464 computed driver times _________________________");
  Serial.print("Current Driver ");
  Serial.println(Driver);
  Serial.print("Driver 0 Time ");
  Serial.println(DriverTime[0]);
  Serial.print("Driver 1 Time ");
  Serial.println(DriverTime[1]);
  Serial.print("Driver 2 Time ");
  Serial.println(DriverTime[2]);
#endif
  // restore record to last so we can contine writing new records for current race
  // since we backed up some records, advance time and point
  // we will increment in loop below
  RealClockTime += RecordsToBackup;
  Point += RecordsToBackup;
  CurrentTime = (hour() * 3600) + (minute() * 60) + second();
  TotalDownTime = CurrentTime - TempTime;
  CarRaceTimer = ((Point * UPDATE_LIMIT) / 1000) * 1000;

  // last known driver gets the down time
  // I think its this (uint8_t so it can't be negative)
  if (Driver < 3) {
    // account for 15 sec boot up time and time estimated time for the rest of this...
    // really aint' worth figuring out how to get it exact
    DriverTimer = DriverTime[Driver] + ((TotalDownTime + 15) * 1000l);
  }

#ifdef DO_DEBUG
  Serial.println("Line 2492 Last record data _________________________");
  Serial.print("Start Record: ");
  Serial.println(StartRecord);
  Serial.print("Last Record: ");
  Serial.println(LastRecord);
  Serial.print("RecordType ");
  Serial.println(RecordType);
  Serial.print("RecordSETID ");
  Serial.println(RecordSETID);
  Serial.print("Point ");
  Serial.println(Point);
  Serial.print("LapCount ");
  Serial.println(LapCount);
  Serial.print("Current Driver ");
  Serial.println(Driver);
  Serial.print("Energy ");
  Serial.println(Energy);
  Serial.print("Distance ");
  Serial.println(Distance);
  Serial.print("LastRaceTime ");
  Serial.println(TempTime);
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
  Serial.print("Last known Point ");
  Serial.println(Point);
  Serial.print("Current Driver Timer ");
  Serial.println(DriverTimer);
  Serial.print("CurrentTime ");
  Serial.println(CurrentTime);
  Serial.print("RealClockTime ");
  Serial.println(RealClockTime);
  Serial.print("TotalDownTime ");
  Serial.println(TotalDownTime);
  Serial.print("Duration ");
  Serial.println(Duration);
  Serial.print("CurrentTime          [s] ");
  Serial.println(CurrentTime);
  Serial.print("Last known race Time [s] ");
  Serial.println(TempTime);
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

  RestoreType = STATUS_RESTORE;

  // compensate for if we backed up any records
  TempTime = TempTime + 1 + RecordsToBackup;

  // add missing records to the database, this will fill in the downtime with default data

  Display.fillRect(STATUS_RESULT, 220, 160, 18, C_BLACK);

  Display.setTextColor(C_CYAN);
  Display.setCursor(STATUS_RESULT, 200);
  Display.print(F("Recreating: "));

  // now we are on the first writable record
  SSD.gotoRecord(LastRecord);

  // Get start time
  uint32_t StartTime = millis();

  for (i = 0; i <= RecordsToRestore; i++) {

    StatusBarCounter++;
    StatusBarWidth = ((float)(i * 160.0) / RecordsToRestore) + 2;
    Display.fillRoundRect(STATUS_RESULT, 200, StatusBarWidth, 18, 2, C_GREEN);

    // clock must increment by UPDATE_LIMIT
    RealClockTime = TempTime + (int)((i * UPDATE_LIMIT) / 1000);
    if (Point > ((float)((1000.0 / UPDATE_LIMIT)) * (RACE_TIME_SECONDS + RACE_EXTENSION))) {
      break;
    }

    if (SSD.addRecord()) {
      SSD.saveRecord();
    }
    delay(5);
    // this will create  know data from the record starting with last known EnergyPointCounter
    if (i % ((60 * 1000) / UPDATE_LIMIT) == 0) {
      if (EnergyPointCounter < 100) {
        EnergyPointCounter++;
        EnergyPoints[EnergyPointCounter] = Energy;
      }
    }

    Point++;
  }

  // we have incremented a point but not added a record
  // so back out that point
  Point--;

  Duration = ((hour() * 3600) + (minute() * 60) + second()) - ((RaceHour * 3600) + (RaceMinute * 60) + RaceSecond);

  Duration = Duration + 7 + ((millis() - StartTime) / 1000);

  // todo maybe keep this simple and use (Point * 1000 / 2)
  CarRaceTimer = (Duration * 1000l);

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

void StartDisplay() {
  Display.begin();
  Display.setTextWrap(false);
  Display.setClock(10000000);
}

void RestartDisplay() {
  StartDisplay();
  SetScreenParameters();
  RedrawHeader = true;
  RedrawDisplay = true;
  DrawGraph = true;
}

void SpeedView() {
  if (RedrawHeader) {
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(DisplayIDText[DisplayID]);
    Display.fillRect(0, 170, 320, 70, DetailsColor);
    Display.fillRect(0, 170, 320, 4, ForeColor);
    Display.setFont(FONT_16B);
    Display.setCursor(5, 180);
    Display.setTextColor(ForeColor);
    Display.print(F("Lap Average"));
    LapAverageBar.setSectionColors(C_GREEN, C_GREEN, C_GREEN, InactiveColor);
    LapAverageBar.setSectionSize(15.0f, 20.0f);
    LapAverageBar.setScale(0.0f, 32.0f, 1);
    LapAverageBar.refresh();

    RedrawHeader = false;
  }

  Display.setCursor(DATA_X, DATA_Y);
  Display.setFont(FONT_100BINO);
  ffMainData.setTextColor(ForeColor, BackColor);
  ffMainData.print(CarSpeed, 1);

  if (LapSpeed < 15.0f) {  // GPUSA rules if a car is less 15 mph for 3 laps, it gets pulled off the track
    LapAverageBar.setSectionColors(C_RED, C_RED, C_RED, InactiveColor);
  } else if (LapSpeed < 20.0f) {  // heck, if were doing this, something is wrong
    LapAverageBar.setSectionColors(C_YELLOW, C_YELLOW, C_YELLOW, InactiveColor);
  } else {
    LapAverageBar.setSectionColors(C_GREEN, C_GREEN, C_GREEN, InactiveColor);
  }

  LapAverageBar.draw(LapSpeed);
  Display.setFont(FONT_48BINO);
  Display.setCursor(LADATA_X, LADATA_Y);
  ffLapData.setTextColor(ForeColor, DetailsColor);
  ffLapData.print(LapSpeed, 1);
}

/*
   PURPOSE : Draws cyborg View
   PARAMS :  -
   RETURNS : None
   NOTES : Draws signal going to the esc as well as energy (both in %)
*/

void CYBORGView() {
  if (RedrawHeader) {
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(DisplayIDText[DisplayID]);

    Display.setTextColor(ForeColor);
    Display.setFont(FONT_14);

    Display.fillRect(0, 165, 320, 75, DetailsColor);

    Display.fillRect(0, 167, 320, 3, ForeColor);
    Display.fillRect(160, 167, 3, 73, ForeColor);

    Display.setFont(FONT_16B);
    Display.setTextColor(ForeColor);
    Display.setCursor(10, 43);
    if (CyborgInput == CYBORG_CONTROL_AMPS) {
      Display.print("Max Amps");
    } else {
      Display.print("Max Speed");
    }

    Display.setCursor(6, 171);
    if (CyborgInput == CYBORG_CONTROL_AMPS) {
      Display.print("Target");
    } else {
      Display.print("Target");
    }

    Display.setCursor(185, 171);
    if (CyborgInput == CYBORG_CONTROL_AMPS) {
      Display.print("Actual");
    } else {
      Display.print("Actual");
    }

    RedrawHeader = false;
  }

  Display.setCursor(DATA_X, DATA_Y + 10);
  Display.setFont(FONT_100BINO);
  ffCyborgSetpoint.setTextColor(ForeColor, BackColor);
  ffCyborgSetpoint.print(CyborgFirstLimit, 1);

  Display.setFont(FONT_48BINO);

  if (EnablePIDTuning) {
    Display.setCursor(130, 190);
    ffCyborgThrottle.setTextColor(C_RED, DetailsColor);
    ffCyborgThrottle.print(Kp, 0);

    Display.setCursor(310, 190);
    ffCyborgInput.setTextColor(C_RED, DetailsColor);
    ffCyborgInput.print(Ki, 0);

  } else {

    // stuff in center bottom (setpoint)

    Display.setCursor(130, 190);
    ffCyborgThrottle.setTextColor(ForeColor, DetailsColor);
    ffCyborgThrottle.print(CyborgSetpoint, 1);

    //right side is actual current
    Display.setCursor(310, 190);
    ffCyborgInput.setTextColor(ForeColor, DetailsColor);

    if (CyborgInput == CYBORG_CONTROL_AMPS) {
      ffCyborgInput.print(Amps, 1);
    } else {
      ffCyborgInput.print(CarSpeed, 1);
    }
  }

  // give some feedback to signals--mainly for verification
  // cute bar graph for throttle %


  Display.fillRect(139, 170, 20, 70.1 - (CyborgInSignal * 70) / 100, DetailsColor);
  Display.fillRect(139, 240.1 - (CyborgInSignal * 70) / 100, 20, 0.1 + (CyborgInSignal * 70) / 100, C_ORANGE);
  // cute bar graph for signal to ESC %
  Display.fillRect(164, 170, 20, 71 - (CyborgOutSignal * 70) / 100, DetailsColor);
  Display.fillRect(164, 241 - (CyborgOutSignal * 70) / 100, 20, (CyborgOutSignal * 70) / 100, C_GREEN);
}

/*
   PURPOSE : Draws amps View
   PARAMS :  -
   RETURNS : None
   NOTES : Draws amperage and a few max parameters)
*/
void AmpsView() {
  if (RedrawHeader) {
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(DisplayIDText[DisplayID]);
    Display.fillRect(0, 170, 320, 70, DetailsColor);
    Display.fillRect(0, 170, 320, 4, ForeColor);
    Display.setFont(FONT_16B);
    Display.setCursor(5, 180);
    Display.setTextColor(ForeColor);
    Display.print(F("Lap Average"));
    LapAverageBar.setSectionColors(C_GREEN, C_YELLOW, C_RED, InactiveColor);
    LapAverageBar.setSectionSize(WARNING_LAPAMP - 2.0f, WARNING_LAPAMP + 0.5f);
    LapAverageBar.setScale(0, WARNING_LAPAMP + 5.0f, 1);
    LapAverageBar.refresh();

    RedrawHeader = false;
  }

  ffMainData.setTextColor(ForeColor, BackColor);
  Display.setFont(FONT_100BINO);
  Display.setCursor(DATA_X, DATA_Y);

  if (Amps > 99) {
    ffMainData.print(Amps, 0);
  } else if (Amps >= 1.0f) {
    ffMainData.print(Amps, 1);
  } else if (Amps >= -1.0) {
    ffMainData.print(Amps, 2);
  } else {
    ffMainData.print(Amps, 1);
  }

  //Bottom Info

  LapAverageBar.draw(LapAmps);
  Display.setFont(FONT_48BINO);
  Display.setCursor(LADATA_X, LADATA_Y);
  ffLapData.setTextColor(ForeColor, DetailsColor);
  ffLapData.print(LapAmps, 1);
}

/*
  PURPOSE : Draws Volts View
  PARAMS :  -
  RETURNS : None
  NOTES : Draws Volts view in the Display Data Function when called in the switch
 */

void VoltsView() {
  if (RedrawHeader) {
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(DisplayIDText[DisplayID]);
    Display.fillRect(0, 170, 320, 70, DetailsColor);
    Display.fillRect(0, 170, 320, 4, ForeColor);
    Display.setTextColor(ForeColor);
    Display.setFont(FONT_16B);
    Display.setCursor(5, 180);
    Display.print(F("Lap Average"));

    LapAverageBar.setSectionColors(C_RED, C_YELLOW, C_GREEN, InactiveColor);
    LapAverageBar.setSectionSize(WARNING_BATTERY - 2.0f, WARNING_BATTERY + 1.0f);
    LapAverageBar.setScale(12, 26, 1);
    LapAverageBar.refresh();
    RedrawHeader = false;
  }

  Display.setCursor(DATA_X, DATA_Y);
  Display.setFont(FONT_100BINO);
  ffMainData.setTextColor(ForeColor, BackColor);
  ffMainData.print(Volts, 1);

  //Bottom Info

  if ((LapVolts + 2.0f) < WARNING_BATTERY) {
    LapAverageBar.setSectionColors(C_RED, C_RED, C_RED, InactiveColor);
  } else if (LapVolts < WARNING_BATTERY) {
    LapAverageBar.setSectionColors(C_YELLOW, C_YELLOW, C_YELLOW, InactiveColor);
  } else {
    LapAverageBar.setSectionColors(C_GREEN, C_GREEN, C_GREEN, InactiveColor);
  }

  LapAverageBar.draw(LapVolts);
  Display.setFont(FONT_48BINO);
  Display.setCursor(LADATA_X, LADATA_Y);
  ffLapData.setTextColor(ForeColor, DetailsColor);
  ffLapData.print(LapVolts, 1);
}

void EnergyView() {
  if (RedrawHeader) {
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(DisplayIDText[DisplayID]);
    EnergyG.setXAxis(0, 90.0, 15.0);
    EnergyG.setYAxis(0, (((int)((TotalEnergy + 99.0f) / 100)) * 100), 100);
    EnergyG.setXAxis(0, 90, 15);
    RedrawHeader = false;
  }

  if (DrawGraph) {
    // draw the graph
    DrawGraph = false;
    EnergyG.resetStart(bEnergyID);
    EnergyG.resetStart(EnergyID);

    EnergyG.setLineColor(bEnergyID, C_RED);
    EnergyG.setLineThickness(bEnergyID, 1);

    EnergyG.drawGraph();

    // force data to be drawn
    // once a graph is drawn, the counter in
    // compute will take care of this
    GraphDrawTimer = 60000;
    // plot base line curve
    // if plotted data is above curve we dont finish
    // if plotted data is below we undertuned
    for (i = 0; i < 527; i++) {
      GraphPointX = i * 0.17110f;  // 90 min / 527 data points
      if (GraphPointX > 89) {
        GraphPointX = 89;
      }

      EnergyG.setX(GraphPointX);
      EnergyG.plot(bEnergyID, BLEnergy[i] * (TotalEnergy / (float)BLEnergy[526]));  // last data point is max battery capacity
    }
  }

  // if a min has passed plot all data
  // reason we redraw all points is upon a screen change and we don't need to try
  // to contine where we left off
  // update every minute
  if (GraphDrawTimer >= 60000) {
    EnergyG.resetStart(EnergyID);
    EnergyG.setLineColor(EnergyID, C_CYAN);
    EnergyG.setLineThickness(EnergyID, 4);
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

  EPointer = 50 + map((int)PredictedEnergy, 0, (((int)((TotalEnergy + 99.0f) / 100)) * 100), 160, 0);
  if (EPointer < 50) {
    EPointer = 50;
  }
  if (EPointer > 210) {
    EPointer = 210;
  }
  Display.fillTriangle(302, oEPointer, 302 + 15, oEPointer - 5, 302 + 15, oEPointer + 5, BackColor);
  Display.fillTriangle(302, EPointer, 302 + 15, EPointer - 5, 302 + 15, EPointer + 5, C_GREEN);
  oEPointer = EPointer;

  Display.setFont(FONT_24BI);
  Display.setCursor(280, 180);
  ffPredict.setTextColor(ForeColor, InactiveColor);
  ffPredict.print(PredictedEnergy, 0);
}

void GForceView() {
  if (RedrawHeader) {
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(DisplayIDText[DisplayID]);
    Display.fillRect(0, 170, 320, 70, DetailsColor);
    Display.fillRect(0, 170, 320, 4, ForeColor);
    Display.setFont(FONT_16B);
    Display.setCursor(83, 180);
    Display.setTextColor(ForeColor);
    Display.print(F("Lap Maximums"));
    Display.fillRect(158, 110, 4, 40, ForeColor);  // draw 0 G bar
    RedrawHeader = false;
  }

  Display.setFont(FONT_48BINO);

  Display.setCursor(220, 45);
  ffGForceY.setTextColor(ForeColor, BackColor);
  ffGForceY.print(abs(GForceY), 2);

  if (PeakTimer >= PEAK_TIMER) {  // time-based peak decay
    PeakTimer = 0;
    PeakGValue = 0;
  }

  if (((OldGForceY <= 0) && (GForceY >= 0)) || ((OldGForceY >= 0) && (GForceY <= 0))) {
    PeakGValue = 0;  // reset peak on direction change
    PeakTimer = 0;
    PeakColor = InactiveColor;
  }

  if (GForceY > 0) {
    // clear left-side bars (negative side)
    for (i = 0; i <= 25; i++) {
      Display.fillRect(151 - (i * 6), 110 - (i / 2.0), 5, 40 + (i), InactiveColor);
    }
    // draw right-side bars (positive G)
    for (i = 0; i <= 25; i++) {
      if ((GForceY) > (i * 0.04f)) {
        if (i <= 12) {
          BarColor = C_GREEN;
        } else if (i <= 16) {
          BarColor = C_YELLOW;
        } else if (i <= 25) {
          BarColor = C_RED;
        }
        MaxG = i;
        if (MaxG > PeakGValue) {
          PeakGValue = MaxG;
          PeakTimer = 0;
          PeakColor = BarColor;
        }
        Display.fillRect(164 + (i * 6), 110 - (i / 2.0), 5, 40 + (i), BarColor);

      } else {
        if (i != PeakGValue) {
          Display.fillRect(164 + (i * 6), 110 - (i / 2.0), 5, 40 + (i), InactiveColor);
        }
      }
    }
    if (PeakTimer < PEAK_TIMER) {
      Display.fillRect(164 + (PeakGValue * 6), 110 - (PeakGValue / 2.0), 5, 40 + (PeakGValue), PeakColor);
    }

  } else {
    // clear right-side bars (positive side)
    for (i = 0; i <= 25; i++) {
      Display.fillRect(164 + (i * 6), 110 - (i / 2.0), 5, 40 + (i), InactiveColor);
    }

    // draw left-side bars (negative G)
    for (i = 0; i <= 25; i++) {
      if ((abs(GForceY)) > (i * 0.04f)) {
        if (i <= 12) {
          BarColor = C_GREEN;
        } else if (i <= 16) {
          BarColor = C_YELLOW;
        } else if (i <= 25) {
          BarColor = C_RED;
        }
        MaxG = i;
        if (MaxG > PeakGValue) {
          PeakGValue = MaxG;
          PeakTimer = 0;
          PeakColor = BarColor;
        }
        Display.fillRect(151 - (i * 6), 110 - (i / 2.0), 5, 40 + (i), BarColor);
      } else {
        if (i != PeakGValue) {
          Display.fillRect(151 - (i * 6), 110 - (i / 2.0), 5, 40 + (i), InactiveColor);
        }
      }
    }
    if (PeakTimer < PEAK_TIMER) {
      Display.fillRect(151 - (PeakGValue * 6), 110 - (PeakGValue / 2.0), 5, 40 + (PeakGValue), PeakColor);
    }
  }

  Display.setFont(FONT_24BI);
  Display.setCursor(80, 205);
  ffGForceYLapMaxL.setTextColor(ForeColor, BackColor);
  ffGForceYLapMaxL.print(GForceYLapMaxL, 2);

  Display.setCursor(305, 205);
  ffGForceYLapMaxR.setTextColor(ForeColor, BackColor);
  ffGForceYLapMaxR.print(GForceYLapMaxR, 2);

  OldGForceY = GForceY;  // always update for next-cycle crossing detection
}

/*
   PURPOSE : Draws Temp View
    PARAMS :  -
   RETURNS : None
     NOTES : Draws Temp view in the Display Data Function when called in the switch
*/

void TempView() {
  if (RedrawHeader) {
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(DisplayIDText[DisplayID]);
    Display.fillRect(0, 200, 320, 40, ForeColor);
    RedrawHeader = false;
  }
  if (DrawGraph) {
    DrawGraph = false;
    MotorTempG.refresh();
    AuxTempG.refresh();
    AmbTempG.refresh();
  }

  MotorTempG.draw(MotorTemp);
  AuxTempG.draw(AuxTemp);
  AmbTempG.draw(AmbTemp);

  // show current time and date
  if (hour() > 12) {
    sprintf(buf, "%d:%02d:%02d  %d/%d/%d", hour() & 12, minute(), second(), month(), day(), year());
  } else {
    sprintf(buf, "%d:%02d:%02d  %d/%d/%d", hour(), minute(), second(), month(), day(), year());
  }
  Display.setFont(FONT_24BI);
  Display.setCursor(10, 208);
  ffTime.setTextColor(BackColor, ForeColor);
  ffTime.print(buf);
}

/*
   PURPOSE : Draws Time View
    PARAMS :  -
   RETURNS : None
     NOTES : Draws Time view in the Display Data Function when called in the switch
*/

void TimeView() {
  if (RedrawHeader) {
    //Display.fillScreen(BackColor);
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(DisplayIDText[DisplayID]);
    Display.fillRect(0, 170, 320, 70, DetailsColor);
    Display.fillRect(0, 170, 320, 4, ForeColor);
    Display.setFont(FONT_16B);
    Display.setTextColor(ForeColor, DetailsColor);
    Display.setCursor(5, 180);
    Display.print(F("Seat Time"));
    Display.setCursor(215, 180);
    Display.print(F("Last Lap"));
    Display.fillRect(160 - 40, 170, 80, 65, ForeColor);

    RedrawHeader = false;
  }

  // split time
  if (RaceStatus == RACE_INPROGRESS) {
    if (TimeDelta > 2) {
      ffMainData.setTextColor(C_RED, BackColor);
      sprintf(buf, "+%d", TimeDelta);
    } else if (TimeDelta < -2) {
      sprintf(buf, "-%d", abs(TimeDelta));
      ffMainData.setTextColor(C_GREEN, BackColor);
    } else {
      sprintf(buf, " %d", TimeDelta);
      ffMainData.setTextColor(C_YELLOW, BackColor);
    }
  } else if ((RaceStatus == RACE_FINISHED) || (RaceStatus == RACE_NOTSTARTED)) {
    ffMainData.setTextColor(ForeColor, BackColor);
    strcpy(buf, "0");
  }

  Display.setFont(FONT_100BINO);
  Display.setCursor(DATA_X, 57);
  ffMainData.print(buf);

  // driver seat time
  if (RaceStatus == RACE_INPROGRESS) {
    mn = (DriverTime[Driver] / 1000) / 60;
    sc = (DriverTime[Driver] / 1000) % 60;
    sprintf(buf, "%01d:%02d", mn, sc);
  } else if (RaceStatus == RACE_FINISHED) {
    strcpy(buf, "DONE");
  } else if (RaceStatus == RACE_NOTSTARTED) {
    strcpy(buf, "0:00");
  }
  Display.setFont(FONT_24BI);
  Display.setCursor(105, 205);
  ffDriverTime.setTextColor(ForeColor, DetailsColor);
  ffDriverTime.print(buf);

  // print lap time
  if (RaceStatus == RACE_INPROGRESS) {
    mn = abs(LapTime) / 60;
    sc = abs(LapTime) % 60;
    sprintf(buf, "%01d:%02d", mn, sc);
  } else if (RaceStatus == RACE_FINISHED) {
    strcpy(buf, "DONE");
  } else if (RaceStatus == RACE_NOTSTARTED) {
    strcpy(buf, "0:00");
  }
  Display.setFont(FONT_24BI);
  Display.setCursor(305, 205);
  ffDriverLapTime.setTextColor(ForeColor, DetailsColor);
  ffDriverLapTime.print(buf);

  Display.setFont(FONT_48BINO);
  Display.setCursor(195, 180);
  ffLap.setTextColor(BackColor, ForeColor);
  ffLap.print(LapCount);
}

/*
   PURPOSE : Draws Usage View
    PARAMS :  -
   RETURNS : None
     NOTES : Draws Usage view in the Display Data Function when called in the switch
*/

void UsageView() {
  if (RedrawHeader) {
    //Display.fillScreen(BackColor);
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(DisplayIDText[DisplayID]);
    Display.setFont(FONT_14);
    Display.setCursor(140, 50);
    Display.setTextColor(ForeColor);
    Display.print(F("Consumption ("));
    Display.print(Driver + 1);
    Display.print(F(")"));
    //Amps
    Display.fillRect(140, 72, 85, 49, DetailsColor);
    Display.drawRect(140, 72, 179, 49, ForeColor);
    Display.drawFastVLine(225, 72, 49, ForeColor);
    Display.setCursor(150, 92);
    Display.print(F("AMPS/L"));

    Display.fillRect(140, 127, 85, 49, DetailsColor);
    Display.drawRect(140, 127, 179, 49, ForeColor);
    Display.drawFastVLine(225, 127, 49, ForeColor);
    Display.setCursor(148, 147);
    Display.print(F("ENER/L"));

    Display.fillRect(140, 182, 85, 49, DetailsColor);
    Display.drawRect(140, 182, 179, 49, ForeColor);
    Display.drawFastVLine(225, 182, 49, ForeColor);
    Display.setCursor(145, 202);
    Display.print(F("ENERGY"));
    RedrawHeader = false;
    TRemG.refresh();
    ERemG.refresh();
  }

  //Draw Time
  Display.setTextColor(ForeColor);


  // draw the Energy and time remaining, use red if it gets below time by 3%
  if ((TRem - ERem) > 10) {
    ERemG.setSectionColors(C_RED, C_RED, C_RED, InactiveColor);
  } else if ((TRem - ERem) > 5) {
    ERemG.setSectionColors(C_YELLOW, C_YELLOW, C_YELLOW, InactiveColor);
  } else {
    ERemG.setSectionColors(C_GREEN, C_GREEN, C_GREEN, InactiveColor);
  }
  ERemG.draw(ERem);
  TRemG.draw(TRem);

  Display.setFont(FONT_24BI);

  Display.setCursor(305, 85);
  ffAmpsPerLap.setTextColor(ForeColor, BackColor);
  ffAmpsPerLap.print(LapAmps, 1);

  Display.setCursor(305, 140);
  ffEnergyPerLap.setTextColor(ForeColor, BackColor);
  ffEnergyPerLap.print(LapEnergy, 1);

  Display.setCursor(305, 195);
  ffEnergy.setTextColor(ForeColor, BackColor);
  ffEnergy.print(Energy, 0);
}

/*
   PURPOSE : Generates warnings based on car data
    PARAMS :  -
   RETURNS : None
     NOTES : Warnings are displayed in the form of an icon
*/

void GetDriverPitTime() {
  DriverTimeOK = false;
  // math and logic by Delmont Goins 6/3/2025
  // test driver
  if (Driver == 0) {
    if ((DriverTime[0] < (15 * 60 * 1000))) {
      PitTime = (15 * 60 * 1000) - (DriverTime[0]);  // rule is 15 min but +1 second to address round off
      PitTime = (PitTime / 1000) + 1;
      mn = PitTime / 60;
      sc = abs(PitTime) % 60;
      sprintf(pitbuf, "Can pit in: % 01d:%02d", mn, sc);
    } else if ((DriverTime[0] >= (15 * 60 * 1000)) && (DriverTime[0] < (45 * 60 * 1000))) {
      DriverTimeOK = true;
      PitTime = (45 * 60 * 1000) - DriverTime[0];
      PitTime = (PitTime / 1000) + 1;
      mn = PitTime / 60;
      sc = abs(PitTime) % 60;
      sprintf(pitbuf, "Pit within: % 01d:%02d", mn, sc);
    } else if (DriverTime[0] >= (45 * 60 * 1000)) {
      DriverTimeOK = true;
      PitTime = (45 * 60 * 1000) - DriverTime[0];
      PitTime = (PitTime / 1000) + 1;
      if (PitTime < 0) {
        PitTime = abs(PitTime);
        mn = PitTime / 60;
        sc = PitTime % 60;
        sprintf(pitbuf, "PIT NOW!-%01d:%02d", mn, sc);
      } else {
        mn = PitTime / 60;
        sc = (PitTime) % 60;
        sprintf(pitbuf, "PIT NOW! %01d:%02d", mn, sc);
      }
    }
  } else if (Driver == 1) {
    if (DriverTime[0] > (45 * 60 * 1000)) {
      // special case if driver 0 exceeds 45 min
      // driver 1 get out at 15 min
      MinDriveTime = max((15 * 60 * 1000), (45 * 60 * 1000) - DriverTime[0]);  // ms
      MaxDriveTime = min((25 * 60 * 1000), (75 * 60 * 1000) - DriverTime[0]);  //ms
    } else {
      MinDriveTime = max((15 * 60 * 1000), (45 * 60 * 1000) - DriverTime[0]);  // ms
      MaxDriveTime = min((45 * 60 * 1000), (75 * 60 * 1000) - DriverTime[0]);  //ms
    }
    if (DriverTime[1] < MinDriveTime) {
      PitTime = (MinDriveTime / 1000) - (DriverTime[1] / 1000) + 1;
      mn = PitTime / 60;
      sc = abs(PitTime) % 60;
      sprintf(pitbuf, "Can pit in: % 01d:%02d", mn, sc);
    } else if ((DriverTime[1] >= MinDriveTime) && (DriverTime[1] <= (MaxDriveTime - (0 * 60 * 1000)))) {
      DriverTimeOK = true;
      PitTime = (MaxDriveTime / 1000) - (DriverTime[1] / 1000) + 1;  // in min
      mn = PitTime / 60;
      sc = abs(PitTime) % 60;
      sprintf(pitbuf, "Pit within: % 01d:%02d", mn, sc);
    } else {
      DriverTimeOK = true;
      PitTime = MaxDriveTime - DriverTime[1];
      PitTime = (PitTime / 1000) + 1;
      if (PitTime < 0) {
        PitTime = abs(PitTime);
        mn = PitTime / 60;
        sc = PitTime % 60;
        sprintf(pitbuf, "PIT NOW!-%01d:%02d", mn, sc);
      } else {
        mn = PitTime / 60;
        sc = (PitTime) % 60;
        sprintf(pitbuf, "PIT NOW! %01d:%02d", mn, sc);
      }
    }
  } else if (Driver == 2) {
    if (((DriverTime[0] + DriverTime[1] + DriverTime[2]) / 1000) > 5400) {
      PitTime = 0;
    } else {
      PitTime = 5400 - (DriverTime[0] / 1000) - (DriverTime[1] / 1000) - (DriverTime[2] / 1000) + 1;  // in sec
    }
    if (PitTime < 0) {
      PitTime = 0;
    }
    mn = PitTime / 60;
    sc = PitTime % 60;
    sprintf(pitbuf, "Time left:% 01d:%02d", mn, sc);
  }
}

void DrawPitMessage() {
  ffPitMessage.setTextColor(C_WHITE, banner_back);
  Display.setFont(FONT_16B);
  Display.setCursor(PITMESSAGEX, 10);
  ffPitMessage.print(pitbuf);
}

void DrawWarnings() {
  // force fails to test icons
  // Warnings = 0b1111111111111111;
  // show racing status

  // battery
  if (Warnings & BAT_WARNING) {
    drawBitmap(93, 3, battery_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(93, 3, battery_icon, 32, 32, banner_back);
  }

  // SSD chip--this is bad...
  if (Warnings & SSD_FAIL) {
    drawBitmap(125, 3, ssd_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(125, 3, ssd_icon, 32, 32, banner_back);
  }

  if (Warnings & SPEED_WARNING) {
    drawBitmap(157, 3, speed_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(157, 3, speed_icon, 32, 32, banner_back);
  }

  // amps
  if (Warnings & AMP_WARNING) {
    // over 70 amps
    drawBitmap(189, 3, amps_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(189, 3, amps_icon, 32, 32, banner_back);
  }

  // GPS
  if (Warnings & GPS_WARNING) {
    drawBitmap(221, 3, gps_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(221, 3, gps_icon, 32, 32, banner_back);
  }

  // temp
  if (Warnings & TEMP_WARNING) {
    drawBitmap(253, 3, temp_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(253, 3, temp_icon, 32, 32, banner_back);
  }

  // g-force
  if (Warnings & GFORCE_WARNING) {
    drawBitmap(285, 3, gforce_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(285, 3, gforce_icon, 32, 32, banner_back);
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
  if ((StartGPSFound) && (GPSDistance <= GPSTolerance) && (RaceStatus == RACE_INPROGRESS) && (GPSLapTimer >= (1000l * LapThreshold))) {

    // we just tiggered get averages
    LapCount++;

    LapAmps = AverageAmps / AverageCount;
    LapVolts = AverageVolts / AverageCount;
    LapEnergy = Energy - StartLapEnergy;
    LapSpeed = AverageCarSpeed / AverageCount;

    // to get the target amps get the average of laps 2 and 3 laps
    // target amps are the "initial" amp draw
    // batteries can support 19.5 amps for 90 min in a test environment
    // flat track is about 18.5, hilly around 16
    // this only works if GPS works and finds lap

    if ((LapCount > 1) && (LapCount < 4)) {
      // using laps 2 and 3
      TempTargetAmps += LapAmps;
    }

    if (LapCount == 5) {
      TargetAmps = TempTargetAmps / 2.0f;
    }

    LastLapTime = LapTime;
    LapTime = LapTimer / 1000;
    TimeDelta = LapTime - LastLapTime;

    if (abs(TimeDelta) > 999) {
      TimeDelta = 0;
    }

    EndDistance = Distance;
    TrackLength = EndDistance - StartDistance;
    StartDistance = EndDistance;

    // reset GForce Max calcluations
    GForceYLapMaxL = TempGForceYLapMaxL;
    GForceYLapMaxR = TempGForceYLapMaxR;
    TempGForceYLapMaxL = 0.0;
    TempGForceYLapMaxR = 0.0;

    digitalWrite(LAPLED_PIN, HIGH);

    GPSLEDTimer = 0;
    AverageCount = 0;
    AverageAmps = 0.0f;
    AverageVolts = 0.0f;
    AverageCarSpeed = 0.0f;
    LapTimer = 0;
    GPSLapTimer = 0;
    StartLapEnergy = Energy;
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
    sprintf(buf, "%d:%02d:%02d, %d/%02d/%02d", hour() % 12, minute(), second(), month(), day(), year());
  } else {
    sprintf(buf, "%d:%02d:%02d, %d/%02d/%02d", hour(), minute(), second(), month(), day(), year());
  }

  Serial.print(F("RTC Time: "));
  Serial.print(buf);
  BuildDateStringMS(millis());
  Serial.print(F(", Datalogger Time: "));
  Serial.print(buf);
  BuildDateStringMS(CarRaceTimer);
  Serial.print(F(", CarRaceTimer: "));
  Serial.println(buf);
  Serial.print(F("Data point: "));
  Serial.print(Point);
  Serial.print(F(", Averages: "));
  Serial.println(Counter);
  Serial.print(F("Volts: "));
  Serial.print(Volts, 2);
  Serial.print(F(", pin: "));
  Serial.print(vVolts, 3);
  Serial.print(F(", Amps: "));
  Serial.print(Amps, 2);
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

  Serial.print(F("Time: \t\t"));
  BuildDateStringS(DriverTime[0] / 1000);
  Serial.print(buf);
  Serial.print(F("\t"));
  BuildDateStringS(DriverTime[1] / 1000);
  Serial.print(buf);
  Serial.print(F("\t"));
  BuildDateStringS(DriverTime[2] / 1000);
  Serial.println(buf);

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
  if (RadioUpdate != 0) {
    return false;
  }
  // if this get's called, EBYTE fails to connect

#ifdef DO_DEBUG
  Serial.println("resetting the EBYTE");
#endif
  Radio.restoreDefaults();

  // set some basic defaults
  Radio.setRSSIAmbientNoise(false);
  Radio.setRSSISignalStrength(false);
  Radio.setPacketSize(SUB_64BYTES);
  Radio.setTransmitPower(TRP_22DB);
  Radio.saveParameters(EBYTE_WRITE_PERMANENT);

#ifdef DO_DEBUG
  Serial.println("TRANSCEIVER RESET");
#endif

  RadioStatus = Radio.init();
  RadioChannel = Radio.getChannel();
  AirDataRate = Radio.getAirDataRate();
  RadioPower = Radio.getTransmitPower();

#ifdef DO_DEBUG
  Serial.println(F("******* EBYTE Parameters *******"));
  Radio.printParameters();
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
    L_PIN = LEFT_PIN;
    R_PIN = RIGHT_PIN;
  } else {
    L_PIN = RIGHT_PIN;
    R_PIN = LEFT_PIN;
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
  DelayAmount = millis();

  while ((millis() - DelayAmount) < msDelay) {

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
  // hence let's not reset if disabled--not sure what will happen
  GPSMaxReadTimer = 0;
  while (GPSSerial.available()) {
    WatchDogTimer(RESET_WDT);
    GPSStatus = true;
    //c = GPSSerial.read();
    GPSSensor.encode(GPSSerial.read());
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
   PARAMS  : int16_t x - x-value
             int16_t y - y-value
             const uint8_t *bitmap - icon
             int16_t w - width
             int16_t h - height
             uint16_t color - C_COLOR
   RETURNS : None
   NOTES   : Icons must be bitmap images and converted to uint8_t form using image2.cpp, a 3rd party program; must be stored as extern uint16_t variables[]
*/

void drawBitmap(int16_t x, int16_t y, const unsigned char *bitmap, int16_t w, int16_t h, uint16_t color) {
  int16_t ByteWidth = 0;
  uint8_t sByte = 0;
  uint16_t i = 0, j = 0;

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
  TopMainMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, 35, 5, "Main Menu", FONT_16B, FONT_16B);
  MainMenuID1 = TopMainMenu.add565("Race", race_icon565, 32, 32);
  MainMenuID9 = TopMainMenu.add565("Race Playback", graph_icon565, 32, 32);
  MainMenuID6 = TopMainMenu.add565("Data Storage", SSD_icon565, 32, 32);
  MainMenuID8 = TopMainMenu.add565("Cyborg", cyborg_icon565, 32, 32);
  MainMenuID2 = TopMainMenu.add565("Settings", car_icon565, 32, 32);
  MainMenuID3 = TopMainMenu.add565("Wireless", transceiver_icon565, 32, 32);
  MainMenuID4 = TopMainMenu.add565("Sensors", calibrate_icon565, 32, 32);
  MainMenuID7 = TopMainMenu.add565("Accelerometer", GForce_icon565, 32, 32);
  MainMenuID5 = TopMainMenu.add565("Clock", clock_icon565, 32, 32);
  TopMainMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  TopMainMenu.setMenuBarMargins(10, 319, 6, 2);
  TopMainMenu.setItemTextMargins(10, 9, 5);
  TopMainMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  TopMainMenu.setTitleTextMargins(50, 13);

  RaceMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 230, 22, 8, "Driver Setup", FONT_14, FONT_16B);
  RaceMenuID2 = RaceMenu.addNI("Tires", TireID, 0, sizeof(TireText) / sizeof(TireText[0]), 1, 0, TireText);
  RaceMenuID9 = RaceMenu.addNI("Front tire pressure [psi]", TirePressureFront, 70, 200, 5);
  RaceMenuID14 = RaceMenu.addNI("Rear tire pressure [psi]", TirePressureRear, 70, 200, 5);
  sprintf(buf, "Motor sprocket (%.2f)", GearRatio);
  RaceMenuID3 = RaceMenu.addNI(buf, MotorSprocket, 10, 20, 1);
  sprintf(buf, "Wheel sprocket (%.2f)", GearRatio);
  RaceMenuID4 = RaceMenu.addNI(buf, WheelSprocket, 20, 90, 1);
  RaceMenuID5 = RaceMenu.addNI("Battery energy [whr]", TotalEnergy, 550, 750, 1);
  RaceMenuID6 = RaceMenu.addNI("Battery 1", Battery1, 1, 99, 1);
  RaceMenuID7 = RaceMenu.addNI("Battery 2", Battery2, 1, 99, 1);
  RaceMenuID8 = RaceMenu.addNI("Add lap when pitting", AddLapInPit, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  RaceMenuID10 = RaceMenu.addNI("Delay GPS start read", StartGPSDelayID, 0, sizeof(GPSReadTimeText) / sizeof(GPSReadTimeText[0]), 1, 0, GPSReadTimeText);
  RaceMenuID12 = RaceMenu.addNI("GPS trigger range", GPSTolerance, 0, sizeof(GPSToleranceText) / sizeof(GPSToleranceText[0]), 1, 0, GPSToleranceText);
  RaceMenuID13 = RaceMenu.addNI("GPS lap threshold [s]", LapThreshold, 10, 120, 5);
  RaceMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  RaceMenu.setItemTextMargins(2, 3, 5);
  RaceMenu.setMenuBarMargins(1, 319, 3, 1);
  RaceMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  RaceMenu.setTitleTextMargins(50, 13);

  SettingsMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 230, 22, 8, "Settings", FONT_14, FONT_16B);
  SettingsMenuID5 = SettingsMenu.addNI("Start/Driver trigger [a]", TriggerAmps, 10, 90, 1, 0);
  SettingsMenuID1 = SettingsMenu.addNI("Motor", MotorID, 0, sizeof(MotorText) / sizeof(MotorText[0]), 1, 0, MotorText);
  SettingsMenuID2 = SettingsMenu.addNI("Button location", Orientation, 0, sizeof(OrientationText) / sizeof(OrientationText[0]), 1, 0, OrientationText);
  SettingsMenuID3 = SettingsMenu.addNI("Theme", Theme, 0, sizeof(ThemeText) / sizeof(ThemeText[0]), 1, 0, ThemeText);
  SettingsMenuID4 = SettingsMenu.addNI("Car", CarID, 0, sizeof(CarText) / sizeof(CarText[0]), 1, 0, CarText);
  SettingsMenuID6 = SettingsMenu.addNI("Restart display each draw", RestartDisplayAlways, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);

  SettingsMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  SettingsMenu.setItemTextMargins(2, 3, 5);
  SettingsMenu.setMenuBarMargins(1, 319, 3, 1);
  SettingsMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  SettingsMenu.setTitleTextMargins(50, 13);

  WirelessMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 220, 22, 5, "Wireless Setup", FONT_14, FONT_16B);
  WirelessMenuID1 = WirelessMenu.addNI("Send time [s]", RadioUpdate, 0, sizeof(SendTimeText) / sizeof(SendTimeText[0]), 1, 0, SendTimeText);
  WirelessMenuID2 = WirelessMenu.addNI("Channel", RadioChannel, 0, 69, 1);
  WirelessMenuID3 = WirelessMenu.addNI("Data rate", AirDataRate, 0, sizeof(AirRateText) / sizeof(AirRateText[0]), 1, 0, AirRateText);
  WirelessMenuID4 = WirelessMenu.addNI("Radio power [dB]", RadioPower, 0, sizeof(PowerText) / sizeof(PowerText[0]), 1, 0, PowerText);
  WirelessMenuID8 = WirelessMenu.addNI("Address key 1", RadioAddressL, 0, 255, 1, 0);
  WirelessMenuID9 = WirelessMenu.addNI("Address key 2", RadioAddressH, 0, 255, 1, 0);
  WirelessMenuID5 = WirelessMenu.addNI("Alt. cor. (700) [ft]", AltCorrection, -400, 400, 5, 0);
  WirelessMenuID6 = WirelessMenu.addNI("GPS Altitude corr. [ft]", GPSAltCorrection, -400, 400, 5, 0);
  WirelessMenuID7 = WirelessMenu.addNI("RESET WIRELESS", ResetEBYTE, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  WirelessMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  WirelessMenu.setItemTextMargins(2, 3, 5);
  WirelessMenu.setMenuBarMargins(1, 319, 3, 1);
  WirelessMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  WirelessMenu.setTitleTextMargins(50, 13);

  SensorMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 230, 22, 5, "Sensor Setup", FONT_14, FONT_16B);
  SensorMenuID1 = SensorMenu.addNI("Volt slope (11.0)", VoltageSlope, 8.0, 12.0, 0.001, 3, NULL);
  SensorMenuID2 = SensorMenu.addNI("Volt offset (0.32 V)", VoltageOffset, 0.20, 0.40, 0.001, 3, NULL);
  SensorMenuID3 = SensorMenu.addNI("Amp sensitiv. (20 mV/A)", mVPerAmp, 10, 30.0, 0.01, 2);
  SensorMenuID4 = SensorMenu.addNI("Amp quies. (0.5 V)", VMid, 0.4, 0.8, 0.001, 3);
  SensorMenuID5 = SensorMenu.addNI("Temp. motor [ohm]", ThermResMotor, 7000.0, 15000.0, 10, 0);
  SensorMenuID6 = SensorMenu.addNI("Temp. aux. [ohm]", ThermResAux, 7000.0, 15000.0, 10, 0);
  SensorMenuID7 = SensorMenu.addNI("Temp. amb. (offset)", AmbTempCF, -20.0, 20.0, .1, 1);
  SensorMenuID9 = SensorMenu.addNI("Speed sensor pickups", Pickups, 1, 50, 1);
  SensorMenuID10 = SensorMenu.addNI("Zero current at startup", AutoCurrentCal, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  SensorMenuID11 = SensorMenu.addNI("Enable air sensor", EnableAirFlowSensor, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  SensorMenuID12 = SensorMenu.addNI("Air speed offset", AirSpeedOffset, -10, 10, .05, 2);
  SensorMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  SensorMenu.setItemTextMargins(2, 3, 5);
  SensorMenu.setMenuBarMargins(1, 319, 3, 1);
  SensorMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  SensorMenu.setTitleTextMargins(50, 13);

  GForceMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 200, 22, 4, "Accelerometer Setup", FONT_14, FONT_16B);
  GForceMenuID7 = GForceMenu.addNI("Auto calibrate", 0, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  GForceMenuID2 = GForceMenu.addNI("X calibration", AccelCalX, -8000, 8000, 50, 0);
  GForceMenuID3 = GForceMenu.addNI("Y calibration", AccelCalY, -8000, 8000, 50, 0);
  GForceMenuID4 = GForceMenu.addNI("Z calibration", AccelCalZ, -8000, 8000, 50, 0);
  GForceMenuID8 = GForceMenu.addNI("Axis point forward", ASensorDirection, 0, sizeof(ASensorDirectionText) / sizeof(ASensorDirectionText[0]), 1, 0, ASensorDirectionText);
  GForceMenuID1 = GForceMenu.addNI("G-Force range", GForceRange, 0, sizeof(AccelFSRange) / sizeof(AccelFSRange[0]), 1, 0, AccelFSRange);
  GForceMenuID5 = GForceMenu.addNI("Low-pass filter", AccelLPFilter, 0, sizeof(AccelLPFilterText) / sizeof(AccelLPFilterText[0]), 1, 0, AccelLPFilterText);
  GForceMenuID6 = GForceMenu.addNI("High-pass filter", AccelHPFilter, 0, sizeof(AccelHPFilterText) / sizeof(AccelHPFilterText[0]), 1, 0, AccelHPFilterText);
  GForceMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  GForceMenu.setItemTextMargins(2, 3, 5);
  GForceMenu.setMenuBarMargins(1, 319, 3, 1);
  GForceMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  GForceMenu.setTitleTextMargins(50, 13);

  ClockMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 230, 22, 4, "Clock Setup", FONT_14, FONT_16B);
  ClockMenuID1 = ClockMenu.addNI("Year", years, 2020, 2040, 1);
  ClockMenuID2 = ClockMenu.addNI("Month", months, 1, 12, 1);
  ClockMenuID3 = ClockMenu.addNI("Day", days, 1, 31, 1);
  ClockMenuID4 = ClockMenu.addNI("Hour", hours, 0, 23, 1);
  ClockMenuID5 = ClockMenu.addNI("Minute", minutes, 0, 60, 1);
  ClockMenuID6 = ClockMenu.addNI("Disable menu auto exit", EnableAutoExit, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  ClockMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  ClockMenu.setItemTextMargins(2, 3, 5);
  ClockMenu.setMenuBarMargins(1, 319, 3, 1);
  ClockMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  ClockMenu.setTitleTextMargins(50, 13);

  CyborgMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 215, 22, 8, "Cyborg Setup", FONT_14, FONT_16B);
  CyborgMenuID2 = CyborgMenu.addNI("Enable Cyborg", EnableCyborg, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  CyborgMenuID1 = CyborgMenu.addNI("Race current [A]", CyborgFirstLimit, CyborgMinRange, CyborgMaxRange, 0.1, 1);
  CyborgMenuID8 = CyborgMenu.addNI("Turbo current [A]", CyborgSecondLimit, 10.0, 40.0, 0.1, 1);
  CyborgMenuID10 = CyborgMenu.addNI("ESC off at [V]", ESCVoltsOff, 0, 2.0, 0.1, 1);
  CyborgMenuID7 = CyborgMenu.addNI("ESC on at [V]", ESCVoltsOn, 2.5, 3.3, 0.1, 1);
  CyborgMenuID3 = CyborgMenu.addNI("Kp (rise time)", Kp, 0, MAX_KP, 5, 0);
  CyborgMenuID4 = CyborgMenu.addNI("Ki (converge rate)", Ki, 0, MAX_KI, 5, 0);
  CyborgMenuID5 = CyborgMenu.addNI("Kd (future prediction)", Kd, 0, MAX_KD, .01, 2);
  CyborgMenuID6 = CyborgMenu.addNI("PID update time [ms]", CyborgUpdateTime, 20, 200, 20, 0);
  CyborgMenuID11 = CyborgMenu.addNI("Cyborg to manage", CyborgInput, 0, sizeof(CyborgInputText) / sizeof(CyborgInputText[0]), 1, 0, CyborgInputText);
  CyborgMenuID12 = CyborgMenu.addNI("CODE", 0, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);  // throttle range
  CyborgMenuID9 = CyborgMenu.addNI("Predict comp. [A]", PredictionCompensation, 10, 180, 1, 0);

  CyborgMenuID13 = CyborgMenu.addNI("Enable PID Tuning", EnablePIDTuning, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);

  CyborgMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  CyborgMenu.setItemTextMargins(2, 3, 5);
  CyborgMenu.setMenuBarMargins(1, 319, 3, 1);
  CyborgMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  CyborgMenu.setTitleTextMargins(50, 13);

  SSDMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, 35, 3, "Data Storage Options", FONT_16B, FONT_16B);
  SSDMenuID3 = SSDMenu.addNI("Download data");
  SSDMenuID1 = SSDMenu.addNI("ERASE SSD chip");
  SSDMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  SSDMenu.setMenuBarMargins(10, 319, 6, 2);
  SSDMenu.setItemTextMargins(10, 9, 5);
  SSDMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  SSDMenu.setTitleTextMargins(50, 13);

  PlaybackMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 250, 22, 8, "Playback Options", FONT_14, FONT_16B);
  // default number of recordsets to 1 and well update when we draw the menu
  PlayBackID9 = PlaybackMenu.addNI("Plot Lines (select heat)", RPBRaceLines, 0, 1, 1, 0);
  PlayBackID1 = PlaybackMenu.addNI("Draw graphs", RPBDrawGraphs, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackID2 = PlaybackMenu.addNI("Plot volts", RPBPlotVolts, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackID3 = PlaybackMenu.addNI("Plot amps", RPBPlotAmps, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackID8 = PlaybackMenu.addNI("Plot lap amps", RPBPlotLapAmps, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackID4 = PlaybackMenu.addNI("Plot speed", RPBPlotSpeed, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackID5 = PlaybackMenu.addNI("Plot motor temp", RPBPlotMTemp, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackID10 = PlaybackMenu.addNI("Plot altitude", RPBPlotAltitude, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackID6 = PlaybackMenu.addNI("Plot throttle signal", RPBCyborgIn, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackID7 = PlaybackMenu.addNI("Plot ESC signal", RPBCyborgOut, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackID11 = PlaybackMenu.addNI("Plot predicted energy", RPBPlotMPEnergy, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);

  PlaybackMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  PlaybackMenu.setItemTextMargins(2, 3, 5);
  PlaybackMenu.setMenuBarMargins(1, 319, 3, 1);
  PlaybackMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  PlaybackMenu.setTitleTextMargins(50, 13);

  MotorTempG.init(10, 181, 40, 105, 40, 160, 20, "Motor", C_WHITE, BackColor, C_MAGENTA, C_DKMAGENTA, BackColor, FONT_14, FONT_14);
  MotorTempG.useSegmentBars(true);
  MotorTempG.setBars(18, 5, 1);
  MotorTempG.setSectionColors(C_GREEN, C_YELLOW, C_RED, C_VDKGREY);
  MotorTempG.setSectionSize(WARNING_MTEMP - 20.0f, WARNING_MTEMP);
  MotorTempG.setScale(40, 160, 20);

  AuxTempG.init(110, 181, 40, 105, 40, 140, 20, "Auxiliary", C_WHITE, BackColor, C_YELLOW, C_DKYELLOW, BackColor, FONT_14, FONT_14);
  AuxTempG.useSegmentBars(true);
  AuxTempG.setBars(18, 5, 1);
  AuxTempG.setSectionColors(C_GREEN, C_YELLOW, C_RED, C_VDKGREY);
  AuxTempG.setSectionSize(90, 110);
  AuxTempG.setScale(40, 140, 20);

  AmbTempG.init(220, 181, 40, 105, 40, 100, 10, "Ambient", C_WHITE, BackColor, C_CYAN, C_DKCYAN, BackColor, FONT_14, FONT_14);
  AmbTempG.useSegmentBars(true);
  AmbTempG.setBars(18, 5, 1);
  AmbTempG.setSectionColors(C_GREEN, C_YELLOW, C_RED, C_VDKGREY);
  AmbTempG.setSectionSize(75, 90);
  AmbTempG.setScale(40, 100, 10);

  LapAverageBar.init(2, 205, 150, 30, 25, 40, 3, "", C_WHITE, C_RED, C_YELLOW, C_GREEN, C_VDKGREY, FONT_16B, FONT_16B);
  LapAverageBar.useSegmentBars(true);
  LapAverageBar.showScale(false);
  LapAverageBar.showTitle(false);
  LapAverageBar.useSegmentBars(true);
  LapAverageBar.setBars(28, 5, 1);

  TRemG.init(10, 235, 45, 160, 0, 100, 1, "Time", C_WHITE, BackColor, C_CYAN, C_DKCYAN, BackColor, FONT_14, FONT_14);
  TRemG.useSegmentBars(true);
  TRemG.showScale(false);
  TRemG.useSegmentBars(true);
  TRemG.setBars(27, 5, 1);
  TRemG.setSectionColors(C_LTBLUE, C_LTBLUE, C_LTBLUE, C_VDKGREY);
  TRemG.setSectionSize(50, 70);
  TRemG.setScale(0, 100, 1);

  ERemG.init(70, 235, 45, 160, 0, 100, 1, "Energy", C_WHITE, BackColor, C_CYAN, C_DKCYAN, BackColor, FONT_14, FONT_14);
  ERemG.useSegmentBars(true);
  ERemG.showScale(false);
  ERemG.useSegmentBars(true);
  ERemG.setBars(27, 5, 1);
  ERemG.setSectionColors(C_GREEN, C_GREEN, C_GREEN, C_VDKGREY);
  ERemG.setSectionSize(50, 70);
  ERemG.setScale(0, 100, 1);
  EnergyG.init("x", "x", "x", C_WHITE, C_DKGREY, C_BLUE, C_BLACK, C_BLACK, FONT_16B, FONT_14);

  EnergyID = EnergyG.add("E", C_CYAN);
  bEnergyID = EnergyG.add("B", C_RED);

  GraphVoltsID = EnergyG.add("V", GCOLOR_VOLTS);
  GraphAmpsID = EnergyG.add("A", GCOLOR_AMPS);
  GraphLapAmpsID = EnergyG.add("LA", GCOLOR_LAMPS);
  GraphSpeedID = EnergyG.add("Sp", GCOLOR_SPEED);
  GraphMTempID = EnergyG.add("Tm", GCOLOR_TEMP);
  GraphAltitudeID = EnergyG.add("Al", GCOLOR_ALT);
  GraphCyborgInID = EnergyG.add("CyI", GCOLOR_CBGIN);
  GraphCyborgOutID = EnergyG.add("CyO", GCOLOR_CBGOUT);
  GraphPredictedID = EnergyG.add("PE", GCOLOR_PNRG);

  EnergyG.setMarkerSize(GraphLapAmpsID, 2);
  EnergyG.setLineThickness(EnergyID, 4);
  EnergyG.setLineThickness(bEnergyID, 2);
  EnergyG.showLegend(false);
  EnergyG.showTitle(false);
  EnergyG.showAxisLabels(false);

  EnergyG.setXTextOffset(5);

  // Parameters May have changed from getParameters and above first creates menu item colors
  SetScreenParameters();
}

/*
   PURPOSE : draws a cute progress bar in menus to show how long before menu auto exit
   PARAMS :  -
   RETURNS : None
   NOTES :
*/
void DrawExitTimerProgress(int Val) {
  WatchDogTimer(RESET_WDT);

  if (!EnableAutoExit) {
    return;
  }
  // draw progress
  Display.fillRect(0, 0, 320 - (320 * (millis() - ExitStartTime) / MENU_EXIT_TIMEOUT), 4, C_RED);

  // draw blankout
  if (Val == 0) {
    Display.fillRect(320 - (320 * (millis() - ExitStartTime) / MENU_EXIT_TIMEOUT), 0, (320 * (millis() - ExitStartTime) / MENU_EXIT_TIMEOUT), 4, MENU_HIGHLIGHT);
  } else {
    Display.fillRect(320 - (320 * (millis() - ExitStartTime) / MENU_EXIT_TIMEOUT), 0, (320 * (millis() - ExitStartTime) / MENU_EXIT_TIMEOUT), 4, MENU_TITLEBACK);
  }
}

/*
   PURPOSE : Setup main menu
   PARAMS :  -
   RETURNS : None
   NOTES :
*/

void ProcessMainMenu() {
  int i = 0;

  WatchDogTimer(DISABLE_WDT);  // on T4.0 still need to call RESET_WDT as you can't fully disable WDT

  MainMenuOption = 1;

  if (EnableCyborg && (CyborgOutputPWM > CYBORG_LOWER_LIMIT)) {
    Display.fillScreen(C_BLACK);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE, C_BLACK);
    Display.setCursor(30, 50);
    Display.print(F("Shutting down"));
    Display.setCursor(30, 100);
    Display.print(F("Cyborg."));

    Display.drawRect(19, 179, 282, 42, C_DKRED);
    // cyborg could be on so shut the ESC off, slowly...

    for (i = CyborgOutputPWM; i >= 0; i -= 20) {
      analogWrite(OUTPUT_PIN, i);
      delay(1);
      // draw progress
      Display.fillRect(20, 180, 280.0f * ((float)i / EXADC_BIT_CONVERSION), 40, C_DKRED);
      // draw blankout
      Display.fillRect((280.0f * ((float)i / EXADC_BIT_CONVERSION)) + 20, 180, 280 - (280.0f * ((float)i / EXADC_BIT_CONVERSION)), 40, C_BLACK);
    }
  }

  CyborgOutputPWM = 0;

  Display.fillScreen(C_BLACK);
  TopMainMenu.draw();

  WaitForRelease();

  ExitStartTime = millis();

  while (MainMenuOption > 0) {

    delay(50);

    if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
      MainMenuOption = 0;
    }

    DrawExitTimerProgress(TopMainMenu.item);

    ButtonPressed = WhatButtonWasPressed();

    if (ButtonPressed == L_BUTTON) {
      TopMainMenu.MoveUp();
      WaitForRelease();
      ExitStartTime = millis();
    } else if (ButtonPressed == R_BUTTON) {
      TopMainMenu.MoveDown();
      WaitForRelease();
      ExitStartTime = millis();
    } else if (ButtonPressed == C_BUTTON) {
      MainMenuOption = TopMainMenu.selectRow();
      WaitForRelease();

      if (MainMenuOption == MainMenuID1) {
        Display.fillScreen(C_BLACK);
        ProcessRaceMenu();

      } else if (MainMenuOption == MainMenuID2) {
        Display.fillScreen(C_BLACK);
        ProcessSettingsMenu();
      } else if (MainMenuOption == MainMenuID3) {
        Display.fillScreen(C_BLACK);
        ProcessWirelessMenu();
      } else if (MainMenuOption == MainMenuID4) {
        Display.fillScreen(C_BLACK);
        ProcessSensorMenu();

      } else if (MainMenuOption == MainMenuID5) {
        Display.fillScreen(C_BLACK);
        ProcessClockMenu();

      } else if (MainMenuOption == MainMenuID6) {
        Display.fillScreen(C_BLACK);
        ProcessSSDMenu();

      } else if (MainMenuOption == MainMenuID7) {
        Display.fillScreen(C_BLACK);
        ProcessGForceMenu();

      } else if (MainMenuOption == MainMenuID8) {
        Display.fillScreen(C_BLACK);
        ProcessCyborgMenu();

      } else if (MainMenuOption == MainMenuID9) {
        Display.fillScreen(C_BLACK);
        ProcessPlaybackMenu();
      }

      if ((millis() - ExitStartTime) < MENU_EXIT_TIMEOUT) {
        Display.fillScreen(C_BLACK);
        TopMainMenu.draw();
      }
    }
    // during menu access, GPSSerial is still getting data
    // and can take time to clear out

    GPSSerial.read();
  }

  EnableAutoExit = true;
  ClockMenu.SetItemValue(ClockMenuID6, EnableAutoExit);
  WatchDogTimer(ENABLE_WDT);
}


/*
  PURPOSE : Setup driver function
  PARAMS : -
  RETURNS : None
  NOTES : Allows user to set driver id's for later analysis
*/

void ProcessRaceMenu() {
  float OldGearRatio = GearRatio;

  MenuOption = 1;

  RaceMenu.draw();

  Display.setFont(FONT_14);
  Display.setTextColor(C_BLACK, C_GREY);

  WaitForRelease();
  ExitStartTime = millis();

  while (MenuOption > 0) {

    delay(5);

    if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
      MenuOption = 0;
    }

    DrawExitTimerProgress(RaceMenu.item);

    ButtonPressed = WhatButtonWasPressed();
    if (ButtonPressed == NO_BUTTON) {
      PressTimer = 0;
    }
    if (ButtonPressed == L_BUTTON) {
      ExitStartTime = millis();
      RaceMenu.MoveUp();
      if (!RaceMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      MotorSprocket = RaceMenu.value[RaceMenuID3];
      WheelSprocket = RaceMenu.value[RaceMenuID4];
      TireID = RaceMenu.value[RaceMenuID2];
      GetGearParameters();


      if (OldGearRatio != GearRatio) {
        OldGearRatio = GearRatio;
        sprintf(buf, "Motor sprocket (%.2f)", GearRatio);
        RaceMenu.setItemText(RaceMenuID3, buf);
        sprintf(buf, "Wheel sprocket (%.2f)", GearRatio);
        RaceMenu.setItemText(RaceMenuID4, buf);
      }
      ExitStartTime = millis();
    } else if (ButtonPressed == R_BUTTON) {

      RaceMenu.MoveDown();
      if (!RaceMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      MotorSprocket = RaceMenu.value[RaceMenuID3];
      WheelSprocket = RaceMenu.value[RaceMenuID4];
      TireID = RaceMenu.value[RaceMenuID2];
      GetGearParameters();

      if (OldGearRatio != GearRatio) {
        OldGearRatio = GearRatio;
        sprintf(buf, "Motor sprocket (%.2f)", GearRatio);
        RaceMenu.setItemText(RaceMenuID3, buf);
        sprintf(buf, "Wheel sprocket (%.2f)", GearRatio);
        RaceMenu.setItemText(RaceMenuID4, buf);
      }
      ExitStartTime = millis();
    } else if (ButtonPressed == C_BUTTON) {
      PressTimer = 0;
      MenuOption = RaceMenu.selectRow();
      WaitForRelease();
      ExitStartTime = millis();
    }
  }

  TireID = (uint8_t)RaceMenu.value[RaceMenuID2];
  MotorSprocket = (int)RaceMenu.value[RaceMenuID3];
  WheelSprocket = (int)RaceMenu.value[RaceMenuID4];
  TotalEnergy = RaceMenu.value[RaceMenuID5];
  Battery1 = (uint8_t)RaceMenu.value[RaceMenuID6];
  Battery2 = (uint8_t)RaceMenu.value[RaceMenuID7];
  AddLapInPit = (bool)RaceMenu.value[RaceMenuID8];
  TirePressureFront = (uint8_t)RaceMenu.value[RaceMenuID9];
  TirePressureRear = (uint8_t)RaceMenu.value[RaceMenuID14];
  StartGPSDelayID = (uint8_t)RaceMenu.value[RaceMenuID10];
  GPSTolerance = (uint8_t)RaceMenu.value[RaceMenuID12];
  LapThreshold = (uint8_t)RaceMenu.value[RaceMenuID13];  // LapThreshold, seconds GPS considers a lap

  EEPROM.put(10, MotorSprocket);
  EEPROM.put(20, WheelSprocket);
  EEPROM.put(30, TireID);
  EEPROM.put(35, TirePressureFront);
  EEPROM.put(36, TirePressureRear);
  EEPROM.put(70, TotalEnergy);
  EEPROM.put(160, LapThreshold);
  EEPROM.put(170, Battery1);
  EEPROM.put(175, Battery2);
  EEPROM.put(185, AddLapInPit);
  EEPROM.put(190, StartGPSDelayID);
  EEPROM.put(280, GPSTolerance);
}

/*
  PURPOSE : Setup car function
  PARAMS : -
  RETURNS : None
  NOTES :
*/

void ProcessSettingsMenu() {
  MenuOption = 1;
  SettingsMenu.draw();

  WaitForRelease();
  ExitStartTime = millis();
  while (MenuOption > 0) {

    delay(5);
    if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
      MenuOption = 0;
    }
    DrawExitTimerProgress(SettingsMenu.item);
    ButtonPressed = WhatButtonWasPressed();

    if (ButtonPressed == NO_BUTTON) {
      PressTimer = 0;
    }

    if (ButtonPressed == L_BUTTON) {
      SettingsMenu.MoveUp();
      delay(100);
      if (!SettingsMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    } else if (ButtonPressed == R_BUTTON) {
      SettingsMenu.MoveDown();
      if (!SettingsMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    } else if (ButtonPressed == C_BUTTON) {
      MenuOption = SettingsMenu.selectRow();
      WaitForRelease();
      ExitStartTime = millis();
    }
  }

  MotorID = (int)SettingsMenu.value[SettingsMenuID1];
  Orientation = (uint8_t)SettingsMenu.value[SettingsMenuID2];
  Theme = (uint8_t)SettingsMenu.value[SettingsMenuID3];
  CarID = (uint8_t)SettingsMenu.value[SettingsMenuID4];
  TriggerAmps = (uint8_t)SettingsMenu.value[SettingsMenuID5];
  RestartDisplayAlways = (bool)SettingsMenu.value[SettingsMenuID6];

  EEPROM.put(40, Theme);
  EEPROM.put(50, Orientation);
  EEPROM.put(130, MotorID);
  EEPROM.put(240, RestartDisplayAlways);
  EEPROM.put(300, CarID);
  EEPROM.put(380, TriggerAmps);

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
  ResetEBYTE = 0;

  // reset the menu data
  WirelessMenu.SetItemValue(WirelessMenuID2, Radio.getChannel());
  WirelessMenu.SetItemValue(WirelessMenuID3, Radio.getAirDataRate());
  WirelessMenu.SetItemValue(WirelessMenuID4, Radio.getTransmitPower());
  WirelessMenu.SetItemValue(WirelessMenuID8, Radio.getAddressL());
  WirelessMenu.SetItemValue(WirelessMenuID9, Radio.getAddressH());

  Display.fillRect(0, 158, 319, 101, C_DKGREY);
  // now we are ready to draw
  WirelessMenu.draw();
  WaitForRelease();
  ExitStartTime = millis();

  Display.setTextColor(C_WHITE, C_BLACK);
  Display.setCursor(5, 170);
  Display.print(F("Coordinates"));
  Display.setCursor(5, 190);
  Display.print(F("Onboard Alt."));
  Display.setCursor(5, 210);
  Display.print(F("GPSAlt/Sat"));


  while (MenuOption > 0) {

    delay(5);
    if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
      MenuOption = 0;
    }
    DrawExitTimerProgress(WirelessMenu.item);
    if (GPSTolerance != 0) {
      GPSRead();
    }

    ButtonPressed = WhatButtonWasPressed();

    if (ButtonPressed == NO_BUTTON) {
      PressTimer = 0;
    }

    if (ButtonPressed == L_BUTTON) {
      WirelessMenu.MoveUp();
      if (!WirelessMenu.isEditing()) {
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    }

    if (ButtonPressed == R_BUTTON) {
      WirelessMenu.MoveDown();
      if (!WirelessMenu.isEditing()) {
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    }

    if (ButtonPressed == C_BUTTON) {
      MenuOption = WirelessMenu.selectRow();
      WaitForRelease();
      ExitStartTime = millis();
    }

    if ((millis() - caltime) > 1000) {

      caltime = millis();

      GPSAltCorrection = (int16_t)WirelessMenu.value[WirelessMenuID6];
      AltCorrection = (int16_t)WirelessMenu.value[WirelessMenuID5];

      PressureSensor.read();
      Altitude = (PressureSensor.getAltitude() * METERS_TO_FEET) + AltCorrection;
      AtmPressure = PressureSensor.getPressure();
      AmbTemp = PressureSensor.getTemperature();
      AmbTemp = (AmbTemp * 1.8) + 32.0 + AmbTempCF;

      GPSAltitude = (GPSSensor.altitude.meters() * METERS_TO_FEET) + GPSAltCorrection;

      GPSLat = GPSSensor.location.lat();
      GPSLon = GPSSensor.location.lng();

      GPSSatellites = GPSSensor.satellites.value();
      GPSStatus = GPSSensor.location.isValid();

      if (!GPSStatus) {
        GPSLon = 0.0;
        GPSLat = 0.0;
        GPSSatellites = 0;
      }

      Display.fillRect(128, 168, 240, 62, C_DKGREY);

      Display.setTextColor(C_YELLOW, C_DKGREY);
      Display.setCursor(140, 170);
      Display.print(GPSLon, 4);
      Display.print(F(" / "));
      Display.print(GPSLat, 4);

      Display.setCursor(140, 190);
      Display.print(Altitude, 0);

      Display.setCursor(140, 210);
      Display.print(GPSAltitude, 0);
      Display.print(F(" / "));
      Display.print(GPSSatellites);

      // force a retest of valid GPS
      GPSStatus = false;
    }
  }

  AltCorrection = (int16_t)WirelessMenu.value[WirelessMenuID5];
  GPSAltCorrection = (int16_t)WirelessMenu.value[WirelessMenuID6];
  RadioUpdate = (uint8_t)WirelessMenu.value[WirelessMenuID1];
  RadioChannel = (uint8_t)WirelessMenu.value[WirelessMenuID2];
  AirDataRate = (uint8_t)WirelessMenu.value[WirelessMenuID3];
  RadioPower = (uint8_t)WirelessMenu.value[WirelessMenuID4];
  ResetEBYTE = (uint8_t)WirelessMenu.value[WirelessMenuID7];
  RadioAddressL = (uint8_t)WirelessMenu.value[WirelessMenuID8];
  RadioAddressH = (uint8_t)WirelessMenu.value[WirelessMenuID9];

  SetupGPS();

  Radio.setRSSIAmbientNoise(false);
  Radio.setRSSISignalStrength(false);
  Radio.setPacketSize(SUB_64BYTES);
  Radio.saveParameters(EBYTE_WRITE_PERMANENT);
  Radio.setChannel(RadioChannel);
  Radio.setTransmitPower(RadioPower);
  Radio.setAirDataRate(AirDataRate);
  Radio.setAddressL(RadioAddressL);
  Radio.setAddressH(RadioAddressH);
  Radio.saveParameters(EBYTE_WRITE_PERMANENT);

  // save stuff to eeprom
  EEPROM.put(45, AltCorrection);
  EEPROM.put(47, GPSAltCorrection);
  EEPROM.put(60, RadioUpdate);

  if (RadioUpdate > 0) {
    if (!Radio.getModel() || ResetEBYTE == 1) {
      WirelessMenu.SetItemValue(WirelessMenuID7, 0);
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
        if (Radio.init()) {
          Display.setCursor(20, 200);
          Display.print(F("ResetEBYTE OK"));
          break;
        }
      }
    }
  }
#ifdef DO_DEBUG
  Serial.println(F("******* EBYTE Parameters *******"));
  Radio.printParameters();
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

  uint16_t SensorCalibrationDate = 0;
  bool VoltSensorCalibration = false, AmpSensorCalibration = false, TempSensorCalibration = false;
  // format mmmmdddddyyyyyyy and offset year by 2026 to reduce size
  SensorCalibrationDate = ((((uint16_t)month() & 0b1111) << 12)) | ((((uint16_t)day() & 0b11111) << 7)) | (((((uint16_t)year() - 2026)) & 0b1111111));

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

  Display.fillRect(0, 158, 319, 101, C_DKGREY);
  Display.setTextColor(C_WHITE);

  Display.setCursor(5, 160);
  Display.print(F("Volts: "));

  Display.setCursor(5, 180);
  Display.print(F("Amps: "));

  Display.setCursor(5, 200);
  Display.print(F("T[f] M / X / A"));

  Display.setCursor(5, 220);
  Display.print(F("WRPM / Air"));

  WaitForRelease();
  ExitStartTime = millis();
  PressTimer = 0;

  while (MenuOption > 0) {

    if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
      MenuOption = 0;
    }

    DrawExitTimerProgress(SensorMenu.item);

    vVolts = vVolts + EXTADC.analogRead(EXTADC_VM_PIN);
    aVolts = aVolts + EXTADC.analogRead(EXTADC_AM_PIN);
    thmVolts = thmVolts + EXTADC.analogRead(EXTADC_THM_PIN);
    thxVolts = thxVolts + EXTADC.analogRead(EXTADC_THX_PIN);

    VoltageSlope = SensorMenu.value[SensorMenuID1];   // volt slope
    VoltageOffset = SensorMenu.value[SensorMenuID2];  // volt offset
    mVPerAmp = SensorMenu.value[SensorMenuID3];       // amp slope
    VMid = SensorMenu.value[SensorMenuID4];           // amp offset
    ThermResMotor = SensorMenu.value[SensorMenuID5];  // temp offset
    ThermResAux = SensorMenu.value[SensorMenuID6];    // temp offset
    AmbTempCF = SensorMenu.value[SensorMenuID7];      // temp offset
    AirSpeedOffset = SensorMenu.value[SensorMenuID12];
    Counter++;

#if defined(__MK20DX256__)  // Teensy 3.2
    if (RPMSensor.available()) {
      RPMSum = RPMSum + RPMSensor.read();
      RPMCount = RPMCount + 1;
      Revolutions = Revolutions + (1.0f / (float)Pickups);
    }
#else
    if (FreqMeasure.available()) {
      RPMSum = RPMSum + FreqMeasure.read();
      RPMCount = RPMCount + 1;
      Revolutions = Revolutions + (1.0f / (float)Pickups);
    }
#endif

    if ((millis() - caltime) > 1000) {
      caltime = millis();
      ComputeSpeed();
      ComputeData();

      PressureSensor.read();
      Altitude = (PressureSensor.getAltitude() * METERS_TO_FEET) + AltCorrection;
      AtmPressure = PressureSensor.getPressure();
      AmbTemp = PressureSensor.getTemperature();
      AmbTemp = (AmbTemp * 1.8) + 32.0 + AmbTempCF;

      Display.fillRect(133, 158, 185, 103, C_DKGREY);

      Display.setTextColor(C_YELLOW, C_DKGREY);

      Display.setCursor(140, 160);
      Display.print(Volts, 2);
      Display.setCursor(240, 160);
      Display.print(vVolts, 3);

      Display.setCursor(140, 180);
      Display.print(Amps, 3);
      Display.setCursor(240, 180);
      Display.print(aVolts, 3);

      Display.setCursor(140, 200);
      Display.print(MotorTemp, 1);
      Display.print(F(" / "));

      Display.print(AuxTemp, 1);
      Display.print(F(" / "));
      Display.print(AmbTemp, 1);

      Display.setCursor(140, 220);
      Display.print(WRPM, 0);
      Display.setCursor(240, 220);
      Display.print(AirSpeed, 1);

      // need to get volts / amps and display
      Counter = 0;
      vVolts = 0.0f;
      aVolts = 0.0f;
      thmVolts = 0.0f;
      thxVolts = 0.0f;
      WRPM = 0;
    }

    ButtonPressed = WhatButtonWasPressed();

    if (ButtonPressed == NO_BUTTON) {
      PressTimer = 0;
    }

    if (ButtonPressed == L_BUTTON) {
      SensorMenu.MoveUp();
      if (!SensorMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    }
    if (ButtonPressed == R_BUTTON) {
      SensorMenu.MoveDown();
      if (!SensorMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    }
    if (ButtonPressed == C_BUTTON) {
      PressTimer = 0;
      MenuOption = SensorMenu.selectRow();
      WaitForRelease();
      ExitStartTime = millis();
      Pickups = (uint8_t)SensorMenu.value[SensorMenuID9];  // pickups

      if ((MenuOption == SensorMenuID1) || (MenuOption == SensorMenuID2)) {
        VoltSensorCalibration = true;
      }
      if ((MenuOption == SensorMenuID3) || (MenuOption == SensorMenuID4)) {
        AmpSensorCalibration = true;
      }
      if ((MenuOption == SensorMenuID5) || (MenuOption == SensorMenuID6) || (MenuOption == SensorMenuID7)) {
        TempSensorCalibration = true;
      }
    }
  }

  RPMSum = 0;
  RPMCount = 0;

  VoltageSlope = SensorMenu.value[SensorMenuID1];      // volt slope
  VoltageOffset = SensorMenu.value[SensorMenuID2];     // volt offset
  mVPerAmp = SensorMenu.value[SensorMenuID3];          // amp slope
  VMid = SensorMenu.value[SensorMenuID4];              // amp offset
  ThermResMotor = SensorMenu.value[SensorMenuID5];     // temp thermistor voltage divider ext
  ThermResAux = SensorMenu.value[SensorMenuID6];       // temp thermistor voltage divider int
  AmbTempCF = SensorMenu.value[SensorMenuID7];         // temp offset
  Pickups = (uint8_t)SensorMenu.value[SensorMenuID9];  // pickups
  AutoCurrentCal = (bool)SensorMenu.value[SensorMenuID10];
  EnableAirFlowSensor = (bool)SensorMenu.value[SensorMenuID11];
  AirSpeedOffset = SensorMenu.value[SensorMenuID12];

  if (VoltSensorCalibration) {
    EEPROM.put(2, SensorCalibrationDate);
  }
  if (AmpSensorCalibration) {
    EEPROM.put(4, SensorCalibrationDate);
  }
  if (TempSensorCalibration) {
    EEPROM.put(6, SensorCalibrationDate);
  }


  EEPROM.put(55, AutoCurrentCal);
  EEPROM.put(110, VoltageSlope);
  EEPROM.put(120, VoltageOffset);
  EEPROM.put(210, Pickups);
  EEPROM.put(220, mVPerAmp);
  EEPROM.put(230, VMid);
  EEPROM.put(310, ThermResMotor);
  EEPROM.put(315, ThermResAux);
  EEPROM.put(410, EnableAirFlowSensor);
  EEPROM.put(412, AirSpeedOffset);
  EEPROM.put(480, AmbTempCF);
}

/*
  PURPOSE : Setup clock menu
  PARAMS: -
  RETURNS : None
  NOTES:
*/

void ProcessClockMenu() {
  uint16_t SensorCalibrationDate = 0;
  years = year();
  months = month();
  days = day();
  hours = hour();
  minutes = minute();
  seconds = second();

  ClockMenu.SetItemValue(ClockMenuID1, years);
  ClockMenu.SetItemValue(ClockMenuID2, months);
  ClockMenu.SetItemValue(ClockMenuID3, days);
  ClockMenu.SetItemValue(ClockMenuID4, hours);
  ClockMenu.SetItemValue(ClockMenuID5, minutes);

  // get the last recordset

  MenuOption = 1;
  ClockMenu.draw();

  Display.fillRect(0, 140, 320, 100, C_GREY);

  Display.setTextColor(C_BLACK, C_GREY);

  Display.setCursor(10, 146);
  Display.print(F("Code: "));
  Display.print(CODE_VERSION);

  Display.setCursor(10, 164);
  EEPROM.get(2, SensorCalibrationDate);
  Display.print(F("Volt Cal: "));
  Display.print(SensorCalibrationDate >> 12);
  Display.print(F("/"));
  Display.print((SensorCalibrationDate >> 7 & 0b0000000000011111));
  Display.print(F("/"));
  Display.println(2026 + (SensorCalibrationDate & 0b00000000001111111));

  Display.setCursor(10, 182);
  EEPROM.get(4, SensorCalibrationDate);
  Display.print(F("Amp Cal: "));

  Display.print(SensorCalibrationDate >> 12);
  Display.print(F("/"));
  Display.print((SensorCalibrationDate >> 7 & 0b0000000000011111));
  Display.print(F("/"));
  Display.println(2026 + (SensorCalibrationDate & 0b00000000001111111));

  Display.setCursor(10, 200);
  EEPROM.get(6, SensorCalibrationDate);
  Display.print(F("Temp Cal: "));
  Display.print(SensorCalibrationDate >> 12);
  Display.print(F("/"));
  Display.print((SensorCalibrationDate >> 7 & 0b0000000000011111));
  Display.print(F("/"));
  Display.println(2026 + (SensorCalibrationDate & 0b00000000001111111));

  Display.setCursor(10, 218);
  EEPROM.get(8, SensorCalibrationDate);
  Display.print(F("Accel. Cal: "));
  Display.print(SensorCalibrationDate >> 12);
  Display.print(F("/"));
  Display.print((SensorCalibrationDate >> 7 & 0b0000000000011111));
  Display.print(F("/"));
  Display.println(2026 + (SensorCalibrationDate & 0b00000000001111111));

  WaitForRelease();
  ExitStartTime = millis();
  while (MenuOption > 0) {

    delay(5);
    if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
      MenuOption = 0;
    }
    DrawExitTimerProgress(ClockMenu.item);
    ButtonPressed = WhatButtonWasPressed();

    if (ButtonPressed == NO_BUTTON) {
      PressTimer = 0;
    }

    if (ButtonPressed == L_BUTTON) {
      ClockMenu.MoveUp();
      if (!ClockMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    }
    if (ButtonPressed == R_BUTTON) {
      ClockMenu.MoveDown();
      if (!ClockMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    }
    if (ButtonPressed == C_BUTTON) {
      MenuOption = ClockMenu.selectRow();
      PressTimer = 0;
      WaitForRelease();
      EnableAutoExit = (int)ClockMenu.value[ClockMenuID6];
      ExitStartTime = millis();
    }
  }

  years = (int)ClockMenu.value[ClockMenuID1];
  months = (int)ClockMenu.value[ClockMenuID2];
  days = (int)ClockMenu.value[ClockMenuID3];
  hours = (int)ClockMenu.value[ClockMenuID4];
  minutes = (int)ClockMenu.value[ClockMenuID5];
  EnableAutoExit = (int)ClockMenu.value[ClockMenuID6];

  seconds = 1;

  setTime(hours, minutes, seconds, days, months, years);

  Teensy3Clock.set(now());

  NeedToUpdateTime = false;
}

void ProcessGForceMenu() {
  uint16_t SensorCalibrationDate = 0;
  bool AccelSensorCalibration = false;

  SensorCalibrationDate = ((((uint16_t)month() & 0b1111) << 12)) | ((((uint16_t)day() & 0b11111) << 7)) | (((((uint16_t)year() - 2026)) & 0b1111111));

  MenuOption = 1;

  GForceMenu.draw();

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
  WaitForRelease();
  ExitStartTime = millis();
  while (MenuOption > 0) {

    delay(5);
    if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
      MenuOption = 0;
    }
    DrawExitTimerProgress(GForceMenu.item);
    ButtonPressed = WhatButtonWasPressed();
    if (ButtonPressed == NO_BUTTON) {
      PressTimer = 0;
    }
    if (ButtonPressed == L_BUTTON) {
      GForceMenu.MoveUp();
      if (!GForceMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    }
    if (ButtonPressed == R_BUTTON) {
      GForceMenu.MoveDown();
      if (!GForceMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    }
    if (ButtonPressed == C_BUTTON) {
      MenuOption = GForceMenu.selectRow();
      WaitForRelease();
      PressTimer = 0;
      ASensorDirection = (uint8_t)GForceMenu.value[GForceMenuID8];  //direction ID
      if (GForceMenu.value[GForceMenuID7] == 1) {
        AccelSensorCalibration = true;
        CalibrateAccererometer();
        GForceMenu.SetItemValue(GForceMenuID7, 0);
        GForceMenu.drawRow(GForceMenuID7);
        GForceMenu.value[GForceMenuID2] = AccelCalX;
        GForceMenu.value[GForceMenuID3] = AccelCalY;
        GForceMenu.value[GForceMenuID4] = AccelCalZ;
        GForceMenu.drawRow(GForceMenuID2);
        GForceMenu.drawRow(GForceMenuID3);
        GForceMenu.drawRow(GForceMenuID4);
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
      } else if (MenuOption == GForceMenuID1) {
        GForceRange = (int16_t)GForceMenu.value[GForceMenuID1];
        SetupAccelerometer();
      } else if (MenuOption == GForceMenuID2) {
        AccelCalX = (int16_t)GForceMenu.value[GForceMenuID2];
        SetupAccelerometer();
      } else if (MenuOption == GForceMenuID3) {
        AccelCalY = (int16_t)GForceMenu.value[GForceMenuID3];
        SetupAccelerometer();
      } else if (MenuOption == GForceMenuID4) {
        AccelCalZ = (int16_t)GForceMenu.value[GForceMenuID4];
        SetupAccelerometer();
      }
      ExitStartTime = millis();
    }
    if (ASensorDirection == 0) {
      // +X
      ay = AccelSensor.getAccelerationX() / ASensorBits;
      ax = AccelSensor.getAccelerationY() / ASensorBits;
      az = AccelSensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 1) {
      // -X
      ay = -(AccelSensor.getAccelerationX()) / ASensorBits;
      ax = AccelSensor.getAccelerationY() / ASensorBits;
      az = AccelSensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 2) {
      // +Y
      ax = AccelSensor.getAccelerationX() / ASensorBits;
      ay = -(AccelSensor.getAccelerationY()) / ASensorBits;
      az = AccelSensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 3) {
      // -Y
      ax = AccelSensor.getAccelerationX() / ASensorBits;
      ay = AccelSensor.getAccelerationY() / ASensorBits;
      az = AccelSensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 4) {
      // +Z
      ax = AccelSensor.getAccelerationX() / ASensorBits;
      az = AccelSensor.getAccelerationY() / ASensorBits;
      ay = AccelSensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 5) {
      // -Z
      ax = AccelSensor.getAccelerationX() / ASensorBits;
      az = -(AccelSensor.getAccelerationY()) / ASensorBits;
      ay = AccelSensor.getAccelerationZ() / ASensorBits;
    }

    Display.fillRect(20, 210, 280, 30, C_DKGREY);
    Display.setFont(FONT_24BI);
    Display.setCursor(20, 210);
    Display.setTextColor(C_RED, C_DKGREY);
    Display.print(ax, 2);

    Display.setCursor(120, 210);
    Display.setTextColor(C_GREEN, C_DKGREY);
    Display.print(ay, 2);

    Display.setCursor(220, 210);
    Display.setTextColor(C_BLUE, C_DKGREY);
    Display.print(az, 2);
  }

  AccelCalX = (int16_t)GForceMenu.value[GForceMenuID2];         // x cal
  AccelCalY = (int16_t)GForceMenu.value[GForceMenuID3];         // y cal
  AccelCalZ = (int16_t)GForceMenu.value[GForceMenuID4];         // z cal
  AccelLPFilter = (uint8_t)GForceMenu.value[GForceMenuID5];     //LP filter DLPF_CFG
  AccelHPFilter = (uint8_t)GForceMenu.value[GForceMenuID6];     //HP Filtern DHPF_CFG
  ASensorDirection = (uint8_t)GForceMenu.value[GForceMenuID8];  //direction ID

  if (AccelSensorCalibration) {
    EEPROM.put(8, SensorCalibrationDate);
  }
  EEPROM.put(200, AccelLPFilter);
  EEPROM.put(205, AccelHPFilter);
  EEPROM.put(250, GForceRange);
  EEPROM.put(275, ASensorDirection);
  EEPROM.put(450, AccelCalX);
  EEPROM.put(460, AccelCalY);
  EEPROM.put(470, AccelCalZ);
}

void ProcessCyborgMenu() {
  MenuOption = 1;

  CyborgMenu.SetItemValue(CyborgMenuID1, CyborgFirstLimit);
  CyborgMenu.SetItemValue(CyborgMenuID8, CyborgSecondLimit);
  CyborgMenu.SetItemValue(CyborgMenuID3, Kp);
  CyborgMenu.SetItemValue(CyborgMenuID4, Ki);
  CyborgMenu.SetItemValue(CyborgMenuID5, Kd);

  sprintf(buf, "Throttle range (%.2f/%.2f)", ThrottleMinRange, ThrottleMaxRange);
  CyborgMenu.setItemText(CyborgMenuID12, buf);

  CyborgMenu.draw();

  WaitForRelease();
  ExitStartTime = millis();
  while (MenuOption > 0) {

    delay(5);
    if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
      MenuOption = 0;
    }
    DrawExitTimerProgress(CyborgMenu.item);
    ButtonPressed = WhatButtonWasPressed();
    if (ButtonPressed == NO_BUTTON) {
      PressTimer = 0;
    }
    if (ButtonPressed == L_BUTTON) {
      CyborgMenu.MoveUp();

      if (!CyborgMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    }
    if (ButtonPressed == R_BUTTON) {
      CyborgMenu.MoveDown();
      if (!CyborgMenu.isEditing()) {
        PressTimer = 0;
        WaitForRelease();
      } else if (PressTimer < 1500) {
        delay(100);
      }
      ExitStartTime = millis();
    }
    if (ButtonPressed == C_BUTTON) {
      PressTimer = 0;
      MenuOption = CyborgMenu.selectRow();
      WaitForRelease();
      if (CyborgMenu.value[CyborgMenuID12] == 1) {

        CalibrateThrottle();

        Display.fillScreen(C_BLACK);
        sprintf(buf, "Cal Throttle (%.2f/%.2f)", ThrottleMinRange, ThrottleMaxRange);
        CyborgMenu.setItemText(CyborgMenuID12, buf);
        CyborgMenu.SetItemValue(CyborgMenuID12, 0);
        CyborgMenu.draw();
      }

      ExitStartTime = millis();
    }
  }

  CyborgFirstLimit = CyborgMenu.value[CyborgMenuID1];
  EnableCyborg = CyborgMenu.value[CyborgMenuID2];
  ESCVoltsOff = CyborgMenu.value[CyborgMenuID10];
  ESCVoltsOn = CyborgMenu.value[CyborgMenuID7];
  PredictionCompensation = CyborgMenu.value[CyborgMenuID9];
  CyborgInput = (uint8_t)CyborgMenu.value[CyborgMenuID11];
  EnablePIDTuning = (bool)CyborgMenu.value[CyborgMenuID13];

  SetCyborgSetpointLimits();

  Kp = (double)CyborgMenu.value[CyborgMenuID3];
  Ki = (double)CyborgMenu.value[CyborgMenuID4];
  Kd = (double)CyborgMenu.value[CyborgMenuID5];

  CyborgUpdateTime = (uint16_t)CyborgMenu.value[CyborgMenuID6];

  CyborgSecondLimit = CyborgMenu.value[CyborgMenuID8];

  CyborgPID.SetTunings(Kp, Ki, Kd);

  Setpoint = CyborgFirstLimit;

  EEPROM.put(25, CyborgInput);
  EEPROM.put(385, ESCVoltsOn);
  EEPROM.put(390, ESCVoltsOff);
  EEPROM.put(485, CyborgFirstLimit);
  EEPROM.put(490, EnableCyborg);
  EEPROM.put(495, CyborgUpdateTime);
  EEPROM.put(500, Kp);
  EEPROM.put(510, Ki);
  EEPROM.put(520, Kd);
  EEPROM.put(530, PredictionCompensation);
  EEPROM.put(535, CyborgSecondLimit);
}

void ProcessSSDMenu() {
  uint32_t CurrentRecord = 0;
  i = 0;
  MenuOption = 1;

  SSDMenu.draw();

  ShowJEDECScreen();

  WaitForRelease();
  ExitStartTime = millis();
  while (MenuOption > 0) {

    delay(50);
    if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
      MenuOption = 0;
    }
    DrawExitTimerProgress(SSDMenu.item);
    ButtonPressed = WhatButtonWasPressed();

    if (ButtonPressed == L_BUTTON) {
      SSDMenu.MoveUp();
      if (SSDMenu.item == 0) {
        ShowJEDECScreen();
      } else if (SSDMenu.item == SSDMenuID1) {
        DrawInfoScreen();
      }
      if (SSDMenu.item > SSDMenuID1) {
        DrawDownloadInfoScreen();
      }
      WaitForRelease();
      ExitStartTime = millis();
    }
    if (ButtonPressed == R_BUTTON) {
      SSDMenu.MoveDown();
      if (SSDMenu.item == 0) {
        ShowJEDECScreen();
      } else if (SSDMenu.item == SSDMenuID1) {
        DrawInfoScreen();
      }
      if (SSDMenu.item > SSDMenuID1) {
        DrawDownloadInfoScreen();
      }
      WaitForRelease();
      ExitStartTime = millis();
    }
    if (ButtonPressed == C_BUTTON) {
      MenuOption = SSDMenu.selectRow();
      WaitForRelease();
      if (MenuOption == SSDMenuID1) {

        Display.fillRect(0, 160, 320, 80, C_WHITE);
        Display.setFont(FONT_14);
        Display.setTextColor(C_RED, BackColor);
        Display.setCursor(10, 167);
        Display.print(F("Erase all race data?"));
        Display.setCursor(10, 192);
        Display.print(F("Press Left / Right to cancel"));
        Display.setCursor(10, 217);
        Display.print(F("Press Center to Continue"));

        while (1) {
          ButtonPressed = WhatButtonWasPressed();
          if ((ButtonPressed == L_BUTTON) || (ButtonPressed == C_BUTTON) || (ButtonPressed == R_BUTTON)) {
            ExitStartTime = millis();
            break;
          }
          if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
            MenuOption = 0;
          }
          DrawExitTimerProgress(SSDMenu.item);
        }
        if (ButtonPressed == C_BUTTON) {
          Display.fillRect(0, 160, 320, 80, C_WHITE);
          Display.setFont(FONT_14);
          Display.setTextColor(C_RED, BackColor);
          Display.setCursor(10, 163);
          Display.print(F("This will take approx 1 min."));
          Display.setCursor(10, 182);
          Display.print(F("Erasing chip..."));
          SSD.eraseAll();
          Display.setCursor(10, 201);
          Display.print(F("Resetting race..."));
          delay(500);
          SaveStartGPS(false);
          ResetRaceDate();

          RecordSETID = 0;

          Display.setCursor(10, 220);
          Display.print(F("Process complete."));
          ExitStartTime = millis();
        }
        delay(500);
        DrawInfoScreen();
      }
      if (MenuOption == SSDMenuID3) {  //all
        CurrentRecord = SSD.getCurrentRecord();
        DownloadRaceData(2);
        DownloadGPSData(1);
        DownloadEEPROM();
        DrawDownloadInfoScreen();
        SSD.gotoRecord(CurrentRecord);
        ExitStartTime = millis();
      }
    }

    if (SSDMenu.item > SSDMenuID1) {
      if (digitalRead(CD_PIN) == HIGH) {
        SDCardStatus = false;  //no card
        Display.setCursor(10, 168);
        Display.setFont(FONT_14);
        Display.setTextColor(C_RED, C_WHITE);
        Display.print(F("NO SD CARD"));
      } else {
        Display.fillRect(10, 167, 134, 19, C_WHITE);
      }
    }
  }
}

void ShowJEDECScreen() {
  Display.fillRect(0, 160, 320, 80, C_WHITE);
  Display.setFont(FONT_16B);
  Display.setTextColor(C_BLACK, C_WHITE);
  Display.setCursor(15, 165);
  Display.print(F("Chip JEDEC: "));
  Display.print(SSD.getChipJEDEC());
  Display.setCursor(15, 190);
  Display.print(F("Free space (kb)"));
  Display.setCursor(200, 190);
  Display.print((SSD.getTotalSpace() - SSD.getUsedSpace()) / 1000);
  Display.setCursor(15, 215);
  Display.print(F("Size (Code/DB): "));
  Display.print(SSD.getRecordLength());
  Display.print(F(" / "));
  Display.print(SSD.getDatabaseRecordLength());
}

void DrawInfoScreen() {
  // draw info screen
  Display.fillRect(0, 160, 320, 80, C_WHITE);
  Display.setFont(FONT_14);
  Display.setTextColor(C_BLACK, C_WHITE);

  Display.setCursor(15, 165);
  Display.print(F("Record (Code/DL)"));
  Display.setCursor(15, 184);
  Display.print(F("Recordsets"));
  Display.setCursor(15, 203);
  Display.print(F("Used space (kb)"));
  Display.setCursor(15, 222);
  Display.print(F("Free space (kb)"));

  Display.setCursor(230, 165);
  Display.print(SSD.getRecordLength());
  Display.print(F(" / "));
  Display.print(SSD.getDatabaseRecordLength());
  Display.setCursor(230, 184);
  Display.print(RecordSETID);
  Display.setCursor(230, 203);
  Display.print(SSD.getUsedSpace() / 1000);
  Display.setCursor(230, 222);
  Display.print((SSD.getTotalSpace() - SSD.getUsedSpace()) / 1000);
}

void DrawDownloadInfoScreen() {
  Display.fillRect(0, 160, 320, 80, C_WHITE);
  Display.drawRoundRect(10, 193, 300, 40, 4, C_BLACK);
  Display.drawRoundRect(11, 194, 298, 38, 3, C_BLACK);
}

void ProcessPlaybackMenu() {
  uint16_t MaxRecordSets = 0;
  uint32_t LastRecord = 0, CurrentRecord = 0;
  MenuOption = 1;

  Display.fillScreen(C_BLACK);
  PlaybackMenu.draw();
  CurrentRecord = SSD.getCurrentRecord();

  LastRecord = SSD.getLastRecord();
  SSD.gotoRecord(LastRecord);
  MaxRecordSets = SSD.getField(RecordSETID, frID);
  PlaybackMenu.setLimits(PlayBackID9, 0, MaxRecordSets, 1, 0);

  if ((LastRecord == 0) || (MaxRecordSets == NULL_RECORD)) {
    MaxRecordSets = 0;
  }

  WaitForRelease();
  ExitStartTime = millis();

  while (MenuOption > 0) {

    delay(50);
    if (((millis() - ExitStartTime) > MENU_EXIT_TIMEOUT) && EnableAutoExit) {
      MenuOption = 0;
    }
    DrawExitTimerProgress(PlaybackMenu.item);
    ButtonPressed = WhatButtonWasPressed();

    if (ButtonPressed == L_BUTTON) {
      PlaybackMenu.MoveUp();
      WaitForRelease();
      delay(100);
      ExitStartTime = millis();
    }
    if (ButtonPressed == R_BUTTON) {
      PlaybackMenu.MoveDown();
      WaitForRelease();
      delay(100);
      ExitStartTime = millis();
    }
    if (ButtonPressed == C_BUTTON) {
      MenuOption = PlaybackMenu.selectRow();
      WaitForRelease();
      RPBPlotVolts = (bool)PlaybackMenu.value[PlayBackID2];
      RPBPlotAmps = (bool)PlaybackMenu.value[PlayBackID3];
      RPBPlotLapAmps = (bool)PlaybackMenu.value[PlayBackID8];
      RPBPlotSpeed = (bool)PlaybackMenu.value[PlayBackID4];
      RPBPlotMTemp = (bool)PlaybackMenu.value[PlayBackID5];
      RPBPlotAltitude = (bool)PlaybackMenu.value[PlayBackID10];
      RPBCyborgIn = (bool)PlaybackMenu.value[PlayBackID6];
      RPBCyborgOut = (bool)PlaybackMenu.value[PlayBackID7];
      RPBPlotMPEnergy = (bool)PlaybackMenu.value[PlayBackID11];

      if (PlaybackMenu.value[PlayBackID1] == 1) {
        PlotRaceData();
        PlaybackMenu.SetItemValue(PlayBackID1, 0);
        PlaybackMenu.drawRow(PlayBackID1);
        PlaybackMenu.draw();
      }

      if (PlaybackMenu.value[PlayBackID9] > 0) {
        PlotRaceLines(PlaybackMenu.value[PlayBackID9]);
        PlaybackMenu.SetItemValue(PlayBackID9, 0);
        PlaybackMenu.drawRow(PlayBackID9);
        PlaybackMenu.draw();
      }
      ExitStartTime = millis();
    }
  }

  SSD.gotoRecord(CurrentRecord);
}

void PlotRaceData() {
  uint32_t CurrentRecord = SSD.getLastRecord();
  bool KeepIn = true;
  uint8_t RecordToPlot = RecordSETID;

  // initial plot
  PlotRaceDataGraphing(RecordSETID);

  while (KeepIn) {

    delay(50);
    ButtonPressed = WhatButtonWasPressed();
    if (ButtonPressed == C_BUTTON) {
      KeepIn = false;
    } else if (ButtonPressed == L_BUTTON) {
      RecordToPlot++;
      if (RecordToPlot > RecordSETID) {
        RecordToPlot = 1;
      }

      PlotRaceDataGraphing(RecordToPlot);

    } else if (ButtonPressed == R_BUTTON) {
      RecordToPlot--;
      if (RecordToPlot < 1) {
        RecordToPlot = RecordSETID;
      };
      PlotRaceDataGraphing(RecordToPlot);
    }
  }
  SSD.gotoRecord(CurrentRecord);
  Display.fillScreen(C_BLACK);
  WaitForRelease();
}


void PlotRaceLines(uint8_t RecID) {
  bool KeepIn = true;
  uint8_t PlotLap = 0, LastLap = 0;
  uint32_t StartRecord = 0, LastRecord = 0, CurrentRecord = 0;

  CurrentRecord = SSD.getLastRecord();

  // get last lap in passed in recordSet
  if (RecID == RecordSETID) {
    // this is the last
    LastRecord = SSD.getLastRecord();
    SSD.gotoRecord(LastRecord);
    LastLap = SSD.getField(LapCount, frLap);
  } else {
    LastRecord = SSD.getFirstRecord(RecID + 1, hrID);
    LastRecord--;
    LastLap = SSD.getField(LapCount, frLap);
  }

  StartRecord = SSD.getFirstRecord(RecID, hrID);
  SSD.gotoRecord(StartRecord + 1);

  PlotRaceLinesGraphing(RecID, PlotLap);

  while (KeepIn) {

    delay(50);

    ButtonPressed = WhatButtonWasPressed();
    if (ButtonPressed == C_BUTTON) {
      KeepIn = false;
    } else if (ButtonPressed == L_BUTTON) {
      PlotLap++;
      if (PlotLap > LastLap) {
        PlotLap = 0;
      }

      PlotRaceLinesGraphing(RecID, PlotLap);

    } else if (ButtonPressed == R_BUTTON) {
      if (PlotLap == 0) {
        PlotLap = LastLap;
      } else {
        PlotLap--;
      }
      PlotRaceLinesGraphing(RecID, PlotLap);
    }
  }

  SSD.gotoRecord(CurrentRecord);
  Display.fillScreen(C_BLACK);
}

void PlotRaceDataGraphing(uint8_t recID) {
  uint8_t rt = 0, div = 0;
  uint32_t scale = 0;
  float data = 0.0f;
  uint16_t ColStart = 10;
  uint32_t StartRecord = 0, EndRecord = 0;
  float LapAmps = 0.0f;
  uint16_t LapAmpsCounter = 0;
  uint8_t PlotLaps = 0, PlotOldLaps = 0;

  StartRecord = SSD.getFirstRecord(recID, hrID);

  if (recID < RecordSETID) {
    EndRecord = SSD.getFirstRecord(recID + 1, hrID);
  } else {
    EndRecord = SSD.getLastRecord();
  }

  if ((EndRecord - StartRecord) > (5400 * 2)) {
    EndRecord = StartRecord + (5400 * 2);
  }

  scale = 5 * ((int)((EndRecord - StartRecord + 599) / 600));

  scale = scale / 15.0f;
  if ((uint32_t)scale != scale) {
    scale++;
  }

  if (scale < 1) {
    scale = 1;
  }

  scale = scale * 15.0;
  div = 15.0;
  if (scale <= 45) {
    div = 5.0;
  }

  EnergyG.setYAxis(0, 30, 5);
  EnergyG.setXAxis(0, scale, div);
  EnergyG.setX(0);

  Display.fillScreen(BackColor);

  Display.setFont(FONT_14);
  Display.setTextColor(ForeColor, BackColor);
  Display.setCursor(5, 10);
  Display.print(F("# "));
  Display.print(recID);

  ColStart = 40;

  if (RPBPlotVolts) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_VOLTS, BackColor);
    Display.print(F("Volts"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Volts");
  }
  if (RPBPlotAmps) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_AMPS, BackColor);
    Display.print(F("Amps"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Amps");
  }
  if (RPBPlotLapAmps) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_LAMPS, BackColor);
    Display.print(F("LAmps"));
    ColStart = 2 + ColStart + Display.measureTextWidth("LAmps");
  }
  if (RPBPlotSpeed) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_SPEED, BackColor);
    Display.print(F("Speed"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Speed");
  }
  if (RPBPlotMTemp) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_TEMP, BackColor);
    Display.print(F("Tmp/5"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Tmp/5");
  }
  if (RPBPlotAltitude) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_ALT, BackColor);
    Display.print(F("Alt[m]/100"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Alt[m]/100");
  }
  if (RPBCyborgIn) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_CBGIN, BackColor);
    Display.print(F("In/4"));
    ColStart = 2 + ColStart + Display.measureTextWidth("In/4");
  }
  if (RPBCyborgOut) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_CBGOUT, BackColor);
    Display.print(F("Out/4"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Out/4");
  }
  if (RPBPlotMPEnergy) {

    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_PNRG, C_BLACK);
    Display.print(F("P. NRG/25"));
  }

  for (i = 1; i < (EndRecord - StartRecord); i++) {

    SSD.gotoRecord(StartRecord + i);

    rt = SSD.getField(RecordType, frType);

    if ((rt == NULL_RECORD) || (rt == RT_HEADER)) {
      break;
    }
    data = (float)i / 120.0;
    if (data > (scale - 2)) {
      break;
    }

    EnergyG.setX(data);

    if (RPBPlotVolts) {
      data = SSD.getField(Volts, frVolts);
      if (i == 1) {
        EnergyG.resetStart(GraphVoltsID);
      }
      EnergyG.plot(GraphVoltsID, data);
    }

    if (RPBPlotLapAmps) {

      LapAmpsCounter++;
      LapAmps += SSD.getField(Amps, frAmps);
      PlotLaps = SSD.getField(LapCount, frLap);
      if (i == 1) {
        EnergyG.resetStart(GraphLapAmpsID);
      }
      if (PlotLaps != PlotOldLaps) {
        PlotOldLaps = PlotLaps;
        LapAmps = LapAmps / LapAmpsCounter;
        EnergyG.plot(GraphLapAmpsID, LapAmps);
        LapAmps = 0;
        LapAmpsCounter = 0;
      }
    }

    if (RPBPlotAmps) {
      data = SSD.getField(Amps, frAmps);
      if (i == 1) {
        EnergyG.resetStart(GraphAmpsID);
      }
      EnergyG.plot(GraphAmpsID, data);
    }

    if (RPBPlotSpeed) {
      data = SSD.getField(CarSpeed, frSpeed);
      if (i == 1) {
        EnergyG.resetStart(GraphSpeedID);
      }
      EnergyG.plot(GraphSpeedID, data);
    }
    if (RPBPlotMTemp) {
      data = SSD.getField(MotorTemp, frMotorTemp);
      if (i == 1) {
        EnergyG.resetStart(GraphMTempID);
      }
      EnergyG.plot(GraphMTempID, data / 5.0f);
    }
    if (RPBPlotAltitude) {
      data = SSD.getField(Altitude, frAltitude);
      if (i == 1) {
        EnergyG.resetStart(GraphAltitudeID);
      }
      EnergyG.plot(GraphAltitudeID, data / 100.0f);
    }
    if (RPBCyborgIn) {
      data = SSD.getField(CyborgInSignal, frCyborgInSignal);
      if (i == 1) {
        EnergyG.resetStart(GraphCyborgInID);
      }
      EnergyG.plot(GraphCyborgInID, data / 4.0f);
    }
    if (RPBCyborgOut) {
      data = SSD.getField(CyborgOutSignal, frCyborgOutSignal);
      if (i == 1) {
        EnergyG.resetStart(GraphCyborgOutID);
      }
      EnergyG.plot(GraphCyborgOutID, data / 4.0f);
    }
    if (RPBPlotMPEnergy) {
      data = SSD.getField(PredictedEnergy, frPredictedEnergy);
      if (i == 1) {
        EnergyG.resetStart(GraphPredictedID);
      }
      EnergyG.plot(GraphPredictedID, data / 25.0f);
    }
  }
}


void PlotRaceLinesGraphing(uint8_t RecID, uint8_t PlotLap) {
  uint8_t rt = 0;
  float x1 = 0.0f, y1 = 0.0f, x2 = 0.0f, y2 = 0.0f;
  uint8_t ReadLap = 0;
  uint32_t TempRecord = 0, NextRecord = 0, StartRecord = 0;
  float TempLat = 0.0f, TempLon = 0.0f, PlotAmps = 0.0f;
  float ScreenTop = 999.0f, ScreenBottom = -999.0f, ScreenLeft = 999.0f, ScreenRight = -999.0f;
  float MinAmps = 0.0f, MaxAmps = 40.0f;
  uint16_t PlotWide = 0, PlotHigh = 0;

  // blank out old graph
  Display.fillRect(50, 25, 270, 240, C_BLACK);

  TempRecord = SSD.getCurrentRecord();
  NextRecord = TempRecord;

  // we need to find the record where the lap starts

  ReadLap = SSD.getField(LapCount, frLap);

  while (PlotLap != ReadLap) {
    NextRecord++;
    SSD.gotoRecord(NextRecord);

    ReadLap = SSD.getField(LapCount, frLap);
    rt = SSD.getField(RecordType, frType);

    if ((rt == NULL_RECORD) || (rt == RT_HEADER)) {
      break;
    }
    if ((PlotLap) == (ReadLap)) {
      break;
    }
  }
  // we should have the start record where laps match
  // now rip through and get boundaries
  StartRecord = NextRecord;
  NextRecord = StartRecord;

  while (PlotLap == ReadLap) {

    NextRecord++;
    SSD.gotoRecord(NextRecord);

    ReadLap = SSD.getField(LapCount, frLap);
    rt = SSD.getField(RecordType, frType);

    if ((rt == NULL_RECORD) || (rt == RT_HEADER)) {
      break;
    }
    if ((PlotLap) != (ReadLap)) {
      break;
    }

    Amps = SSD.getField(Amps, frAmps);

    TempLon = SSD.getField(GPSLat, frLat);
    TempLat = SSD.getField(GPSLon, frLon);
    if (TempLat != 0) {
      if (TempLat < ScreenLeft) {
        ScreenLeft = TempLat;
      }
      if (TempLat > ScreenRight) {
        ScreenRight = TempLat;
      }
    }
    if (TempLon != 0) {
      if (TempLon < ScreenTop) {
        ScreenTop = TempLon;
      }
      if (TempLon > ScreenBottom) {
        ScreenBottom = TempLon;
      }
    }
  }

  // erase screen and plot legend
  Display.fillScreen(C_BLACK);
  Display.setFont(FONT_14);
  Display.setTextColor(C_WHITE);
  Display.fillRect(100, 0, 200, 20, C_BLACK);
  Display.setCursor(40, 1);
  Display.print(F("Amps for Heat: "));
  Display.print(RecID);
  Display.print(F(", Lap: "));
  Display.print(PlotLap);

  for (i = MinAmps; i <= MaxAmps; i++) {
    Display.fillRect(0, 225 - (i * 5), 10, 5, GetPlotColor(i, MinAmps, MaxAmps));
    if ((i % 5) == 0) {
      Display.setCursor(12, 225 - (i * 5));
      Display.print(i);
    }
  }

  SetAspectRatio(&PlotWide, &PlotHigh, ScreenTop, ScreenBottom, ScreenLeft, ScreenRight);

  // now we have screen bounds we can scale and plot
  // start loop all over get the data and plot it
  NextRecord = StartRecord;
  SSD.gotoRecord(NextRecord);
  ReadLap = SSD.getField(LapCount, frLap);
  while (PlotLap == ReadLap) {

    NextRecord++;
    SSD.gotoRecord(NextRecord);
    rt = SSD.getField(RecordType, frType);
    ReadLap = SSD.getField(LapCount, frLap);
    if ((rt == NULL_RECORD) || (rt == RT_HEADER)) {
      break;
    }
    if ((ReadLap) != (PlotLap)) {
      break;
    }

    x1 = SSD.getField(GPSLon, frLon);
    y1 = SSD.getField(GPSLat, frLat);

    x1 = map(x1, ScreenLeft, ScreenRight, 0, PlotWide);
    y1 = map(y1, ScreenTop, ScreenBottom, PlotHigh, 0);

    NextRecord++;
    SSD.gotoRecord(NextRecord);
    // hope this is a valid record
    // todo test
    NextRecord--;

    x2 = SSD.getField(GPSLon, frLon);
    y2 = SSD.getField(GPSLat, frLat);

    if ((x2 == 0) || (y2 == 0)) {
      // Draw2 = false;
    }
    x2 = map(x2, ScreenLeft, ScreenRight, 0, PlotWide);
    y2 = map(y2, ScreenTop, ScreenBottom, PlotHigh, 0);

    PlotAmps = SSD.getField(Amps, frAmps);

    if (PlotAmps < MinAmps) {
      PlotAmps = MinAmps;
    }

    if (PlotAmps > MaxAmps) {
      PlotAmps = MaxAmps;
    }
    Display.drawLine(x1 + 40, y1 + 30, x2 + 40, y2 + 30, GetPlotColor(PlotAmps, MinAmps, MaxAmps));
  }
  SSD.gotoRecord(TempRecord);
}

void SetAspectRatio(uint16_t *PlotWide, uint16_t *PlotHigh, float Top, float Bottom, float Left, float Right) {
  float AR = 0.0f;
  AR = abs(Top - Bottom) / abs(Left - Right);
  if (AR < 1) {
    *PlotWide = 320 - 50;
    *PlotHigh = (240 - 30) * AR;
  } else {
    *PlotWide = (320 - 50) / AR;
    *PlotHigh = (240 - 30);
  }
}

void DownloadRaceData(uint32_t Count) {
  bool SDCardStatus = false;
  uint8_t temp = 0, next = 0, rt = 0, tWS = 0, tMS = 0;
  int16_t OldLap = -1;
  uint32_t i = 0, StatusBarWidth = 0;
  char FileName[27] = "C_RR_YYYY-MM-DD_NN.csv";
  uint16_t tPoint = 0;
  bool OKtoClose = false;
  uint32_t HeaderRecord = 0, CurrentRecord = 0;
  float TempEnergy = 0.0f, EnergyOffset = 0.0f, CurrentEnergy = 0.0f;
  uint32_t SummaryTableLine = 0;
  uint32_t StatusBarCounter = 0;
  CurrentRecord = SSD.getCurrentRecord();

  Display.setFont(FONT_14);
  // Display.fillRect(0, 160, 320, 80, C_WHITE);

  if (digitalRead(CD_PIN) == HIGH) {
    Display.setTextColor(C_RED, C_WHITE);
    Display.fillRect(10, 167, 134, 25, C_WHITE);
    Display.setCursor(10, 168);
    Display.print(F("NO SD CARD"));
    SSD.gotoRecord(CurrentRecord);
    return;
  }

  SDCardStatus = SDCARD.begin(SDCS_PIN, SD_SCK_MHZ(SD_SPI_SPEED));  //SD

  if (!SDCardStatus) {
    Display.setTextColor(C_RED, C_WHITE);
    Display.fillRect(10, 167, 134, 25, C_WHITE);
    Display.setCursor(10, 168);
    Display.print(F("NO SD CARD"));
    SSD.gotoRecord(CurrentRecord);
    return;
  }

  DrawDownloadInfoScreen();

  SSD.gotoRecord(1);

  LastRecord = SSD.getLastRecord();

  for (i = 1; i <= LastRecord; i++) {
    WatchDogTimer(RESET_WDT);
    SSD.gotoRecord(i);
    rt = SSD.getField(RecordType, frType);
    if (rt == NULL_RECORD) {
      SDDataFile.close();
      SSD.gotoRecord(CurrentRecord);
      return;
    }

    // advance progress indicator
    StatusBarCounter++;
    StatusBarWidth = ((float)(StatusBarCounter * 293.0) / (Count * LastRecord)) + 2;
    Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

    if (rt == RT_HEADER) {
      //reset pit stop counter
      SummaryTableLine = 0;
      EnergyOffset = 0.0f,
      CurrentEnergy = 0.0f;

      if (OKtoClose) {
        SDDataFile.close();
        OKtoClose = false;
      }

      HeaderRecord = i;

      sprintf(FileName, "C_%02d_%04d-%02d-%02d_00.csv",
              SSD.getHeaderField(RecordSETID, hrID),
              SSD.getHeaderField(Tyear, hrYear),
              SSD.getHeaderField(Tmonth, hrMonth),
              SSD.getHeaderField(Tday, hrDay));

      if (CarID == BLUE_CAR) {
        FileName[0] = 'B';
      } else if (CarID == RED_CAR) {
        FileName[0] = 'R';
      } else {
        FileName[0] = 'W';
      }
      Display.setTextColor(C_BLACK, C_WHITE);
      Display.fillRect(10, 167, 285, 20, C_WHITE);
      Display.setCursor(10, 168);
      Display.print(FileName);

      next = 0;
      while (SDCARD.exists(FileName)) {

        next++;

        FileName[17] = (int)((next / 10) % 10) + '0';
        FileName[18] = (int)(next % 10) + '0';

        Display.fillRect(10, 167, 285, 20, C_WHITE);
        Display.setTextColor(C_BLACK, C_WHITE);
        Display.setCursor(10, 168);
        Display.print(FileName);

        if (next > 999) {
          break;
        }
      }

      SDCardStatus = SDDataFile.open(FileName, O_WRITE | O_CREAT);

      if (!SDCardStatus) {
        Display.setTextColor(C_RED, C_WHITE);
        Display.setCursor(10, 168);
        Display.print(F("NO SD CARD"));
        delay(1000);
        SSD.gotoRecord(CurrentRecord);
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
      SDDataFile.print(F("Driver #"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Volts"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Amps"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Temp Motor [F]"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Temp Aux [F]"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Power [w]"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Energy [whr]"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Motor RPM"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Wheel RPM"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Speed [MPH]"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Distance [mi]"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Time"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("GPS Speed [MPH]"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Altitude [ft]"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel X"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel Y"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel Z"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("G-Force"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Amb. Temp [f]"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Energy/Lap"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Comments"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Cyborg Input"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Cyborg Output"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Predicted Energy"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Cyborg First Limit"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Air Speed"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("GPS Altitude"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Display"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Amp Hours"));

      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Track Length"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Lap Time"));


      // print another field, remove a space (need to this below as well)

      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.write(DATA_DELIMITER);

      SDDataFile.print(F("PATRIOT RACING RACE RESULTS - CONFIDENTIAL"));
      SDDataFile.println("");
    }

    if (rt == RT_DATA) {

      OKtoClose = true;

      SummaryTableLine++;
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

      // driver number (0-2 but bump 1-3)
      temp = (uint8_t)SSD.getField(Driver, frDriver);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(temp + 1);  // driver starts at 0 make pretty and driver 1 is 1, but name lookup remains 0 based
      // volts
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Volts, frVolts), 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Amps, frAmps), 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(MotorTemp, frMotorTemp), 1);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(AuxTemp, frAuxTemp), 1);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Volts, frVolts) * SSD.getField(Amps, frAmps), 1);
      SDDataFile.write(DATA_DELIMITER);
      CurrentEnergy = (SSD.getField(Energy, frEnergy));
      SDDataFile.print(CurrentEnergy, 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(mRPM, frRPM));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(WRPM, frWRPM));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(CarSpeed, frSpeed), 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Distance, frDist), 4);

      RealClockTime = SSD.getField(RealClockTime, frRT);
      hr = (int)(RealClockTime / 3600);
      mn = (int)((RealClockTime - (hr * 3600)) / 60);
      sc = (int)(RealClockTime % 60);
      sprintf(buf, "%02d:%02d:%02d", hr, mn, sc);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(buf);
      // convert to MPH
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(GPSSpeed, frGPSpeed), 3);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Altitude, frAltitude), 1);
      SDDataFile.write(DATA_DELIMITER);

      GForceX = SSD.getField(GForceX, frMax);
      GForceY = SSD.getField(GForceY, frMay);
      GForceZ = SSD.getField(GForceZ, frMaz);

      SDDataFile.print(GForceX, 3);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(GForceY, 3);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(GForceZ, 3);
      SDDataFile.write(DATA_DELIMITER);
      // compute and print the absolute max g force.
      SDDataFile.print(sqrt((GForceX * GForceX) + (GForceY * GForceY) + (GForceZ * GForceZ)), 3);

      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(AmbTemp, frAmbTemp), 1);  //

      SDDataFile.write(DATA_DELIMITER);
      TempEnergy = CurrentEnergy - EnergyOffset;

      SDDataFile.print(TempEnergy, 2);

      // write comments
      SDDataFile.write(DATA_DELIMITER);

      if (SSD.getField(RestoreType, frRestoreType) == STATUS_OK) {
        // print nothing
      } else if (SSD.getField(RestoreType, frRestoreType) == STATUS_RESTORE) {
        SDDataFile.print(F("Restored"));
      } else if (SSD.getField(RestoreType, frRestoreType) == STATUS_PITSTOP1) {
        SDDataFile.print(F("Pit 1"));  // hard coded text used in Excel query
      } else if (SSD.getField(RestoreType, frRestoreType) == STATUS_PITSTOP2) {
        SDDataFile.print(F("Pit 2"));  // hard coded text used in Excel query
      }

      if (EnableCyborg) {
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(SSD.getField(CyborgInSignal, frCyborgInSignal));  //
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(SSD.getField(CyborgOutSignal, frCyborgOutSignal));  //
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(SSD.getField(PredictedEnergy, frPredictedEnergy), 2);  //
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(SSD.getField(CyborgFirstLimit, frCyborgFirstLimit));  //
      } else {
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(0);
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(0);
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(0);
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.print(0);  //
      }

      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(AirSpeed, frAirSpeed), 2);  //

      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(GPSAltitude, frGPSAltitude), 1);  //

      SDDataFile.write(DATA_DELIMITER);
      if (SSD.getField(DisplayID, frDisplayID) < ((sizeof(DisplayIDText) / sizeof(DisplayIDText[0])))) {
        SDDataFile.print(DisplayIDText[SSD.getField(DisplayID, frDisplayID)]);
      } else {
        SDDataFile.print(F("UNKNOWN"));
      }
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(AmpHours, frAmpHours), 3);

      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(TrackLength, frTrackLength), 3);

      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(LapTime, frLapTime));


      // print the summary block
      if (SummaryTableLine <= 60) {
        SSD.gotoRecord(HeaderRecord);
        // Leave columns between data and header

        // print another field, remove a space (need to this above as well)
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.write(DATA_DELIMITER);

        switch (SummaryTableLine) {
          case 2:
            // car details
            SDDataFile.print(F("CAR SETUP"));
            break;
          case 3:
            // car details
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Car"));
            SDDataFile.write(DATA_DELIMITER);
            if (CarID < ((sizeof(CarText) / sizeof(CarText[0])))) {
              SDDataFile.print(CarText[CarID]);
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
            SDDataFile.print(F("Time"));
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
            SDDataFile.print(F("Diameter [in]"));
            SDDataFile.write(DATA_DELIMITER);
            if (temp < ((sizeof(TireRadius) / sizeof(TireRadius[0])))) {
              SDDataFile.print(TireRadius[temp] * 2.0, 3);
            } else {
              SDDataFile.print(F("UNKNOWN"));
            }
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Pressure [psi] Front / Rear"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(TirePressureFront, hrTirePressureFront));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(TirePressureRear, hrTirePressureRear));
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
            SDDataFile.print(F("Ambient Temp [f]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(AmbTemp, hrTemp));
            break;
          case 11:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Atmospheric Pressure (mbar/Hg)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            AtmPressure = SSD.getHeaderField(AtmPressure, hrStartPressure);
            SDDataFile.print(AtmPressure, 0);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(AtmPressure * 0.02953f, 2);
            break;
          case 12:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Start Altitude [ft] / [m]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(Altitude, hrStartAltitude));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(Altitude, hrStartAltitude) / METERS_TO_FEET);
            break;
          case 15:
            SDDataFile.print(F("PERFORMANCE STATISTICS"));
            break;
          case 16:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Available Energy"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(TotalEnergy, hrEnergy));

            break;
          case 17:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Energy Used"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(J2: J13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=(AL18/AL17)*100"));
            break;
          case 18:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Pit 1 [s]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=COUNTIF(X2:X13000, "));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F("Pit 1"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(") / 2"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Pit 2 [s]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=COUNTIF(X2:X13000, "));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F("Pit 2"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(") / 2"));
            SDDataFile.write(34);
            break;
          case 19:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Amp Hours"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(AF2: AF13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Amp Hours @ 60 min"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=AF7203"));
            break;
          case 20:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Datalogger"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Official (GPUSA)"));
            break;
          case 21:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Laps"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(C2: C13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("<-Enter GPUSA Laps"));
            break;
          case 22:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Track Length [mi]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=AL42"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("<-Enter GPUSA Length"));
            break;
          case 23:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Distance [mi]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(N2: N13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=AN22*AN23"));
            break;
          case 24:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Max V"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(E2:E13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Min V"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(E2:E13000, X2:X13000,"));
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
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", E2:E13000)"));
            SDDataFile.write(34);
            break;
          case 25:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Max A"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(F2:F13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Min A"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(F2:F13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Avg A"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", F2:F13000)"));
            SDDataFile.write(34);
            break;
          case 26:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Motor Temp Max [f]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(G2: G13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Motor Temp Min [f]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(G2:G13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Motor Temp Avg [f]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", G2:G13000)"));
            SDDataFile.write(34);
            break;
          case 27:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Amb Temp Max [f]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(V2: V13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Amb Temp Min [f]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(V2:V13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Amb Temp Avg [f]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", V2:V13000)"));
            SDDataFile.write(34);
            break;
          case 28:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Aux Temp Max [f]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(H2: H13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Aux Temp Min [f]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(H2:H13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Aux Temp Avg [f]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", H2:H13000)"));
            SDDataFile.write(34);
            break;
          case 29:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("RPM Max"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(K2: K13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("RPM Min"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(K2:K13000, X2:X13000,"));
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
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", K2:K13000)"));
            SDDataFile.write(34);
            break;
          case 30:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Speed Max [MPH]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(M2: M13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Speed Min [MPH]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(M2:M13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Speed Avg [MPH]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", M2:M13000)"));
            SDDataFile.write(34);
            break;
          case 31:
            // get just the first 5 minutes as we really only need 1 lap
            // otherwise we get wrong readings as pressure changes over 90 min
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Altitude Max [ft]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(Q2:Q13000, C2:C13000,"));
            SDDataFile.write(34);
            SDDataFile.print(F("1)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Altitude Min [ft]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(Q2:Q13000, C2:C13000,"));
            SDDataFile.write(34);
            SDDataFile.print(F("1)"));
            //SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Altitude Change [ft]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=AL32 - AN32"));
            break;
          case 33:
            SDDataFile.print(F("DRIVER STATISTICS"));
            break;

          case 34:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Driver,1,2,3"));  // driver 1 ID is actually 0, but to make easier for the data analysist driver 1 = 1
            break;

          case 35:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Time [min]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=COUNTIF(D2:D13000,AL35)/120"));  // driver time
            SDDataFile.write(34);                                 // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=COUNTIF(D2:D13000,AM35)/120"));  // driver time
            SDDataFile.write(34);                                 // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=COUNTIF(D2:D13000,AN35)/120"));  // driver time
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Total [min]:"));  // driver time
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=AL36+AM36+AN36"));  // driver time
            break;
          case 36:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Energy [whr]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(J2:J13000, D2:D13000, AL35)"));  // driver time //MAXIFS(A2:A7,B2:B7,1)
            SDDataFile.write(34);                                        // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(J2:J13000, D2:D13000, AM35)-AL37"));  // driver time
            SDDataFile.write(34);                                             // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(J2:J13000, D2:D13000, AN35)-AL37-AM37"));  // driver time
            SDDataFile.write(34);                                                  // write end " to keep comma in formula
            break;
          case 37:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Laps"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(C2:C13000, D2:D13000, AL35)"));  // driver time
            SDDataFile.write(34);                                        // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(C2:C13000, D2:D13000, AM35)-AL38"));  // driver time
            SDDataFile.write(34);                                             // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(C2:C13000, D2:D13000, AN35)-AL38-AM38"));  // driver time
            SDDataFile.write(34);                                                  // write end " to keep comma in formula
            break;
          case 38:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Distance [mi]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(N2:N13000, D2:D13000, AL35)"));  // driver time
            SDDataFile.write(34);                                        // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(N2:N13000, D2:D13000, AM35)-AL39"));  // driver time
            SDDataFile.write(34);                                             // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(N2:N13000, D2:D13000, AN35)-AL39-AM39"));  // driver time
            SDDataFile.write(34);                                                  // write end " to keep comma in formula
            break;
          case 39:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Watts / Mile"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=IF(AL39>0,AL37/AL39,0)"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=IF(AM39>0,AM37/AM39,0)"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=IF(AN39>0,AN37/AN39,0)"));
            SDDataFile.write(34);
            break;
          case 40:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Watts / Minute"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=IF(AL36>0,AL37/AL36,0)"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=IF(AM36>0,AM37/AM36,0)"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=IF(AN36>0,AN37/AN36,0)"));
            SDDataFile.write(34);
            break;
          case 41:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Shortest Lap [mi]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(AG2:AG13000, D2:D13000, AL35, AG2:AG13000, "));  // driver 1
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(">0"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(AG2:AG13000, D2:D13000, AM35, AG2:AG13000, "));  // driver 2
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(">0"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(AG2:AG13000, D2:D13000, AN35, AG2:AG13000, "));  // driver 3
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(">0"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            break;
          case 42:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Fastest Lap [s]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(AH2:AH13000, D2:D13000, AL35, AH2:AH13000, "));  // driver 1
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(">0"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(AH2:AH13000, D2:D13000, AM35, AH2:AH13000, "));  // driver 2
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(">0"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(AH2:AH13000, D2:D13000, AN35, AH2:AH13000, "));  // driver 3
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(">0"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            break;
          case 44:
            SDDataFile.print(F("G-FORCE MEASUREMENTS"));
            break;
          case 45:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Vector"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("X (Medial)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Y (Lateral)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Z (Vertical)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Total"));
            break;
          case 46:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Max"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(R2:R13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(S2:S13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(T2:T13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MAXIFS(U2:U13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            break;
          case 47:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Min"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(R2:R13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(S2:S13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(T2:T13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=MINIFS(U2:U13000, X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")"));
            SDDataFile.write(34);
            break;
          case 48:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Average"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", R2:R13000)"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", S2:S13000)"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", T2:T13000)"));
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=AVERAGEIF(X2:X13000,"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(", U2:U13000)"));
            SDDataFile.write(34);
            break;
          case 50:
            SDDataFile.print(F("DATABASE STATISTICS"));
            break;
          case 51:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Race Time [min]"));
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
            SDDataFile.print(SSD.getHeaderField(AverageCounter, hrCounter) * (1000 / UPDATE_LIMIT));
            break;
          case 54:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Telemetry Downtime [s]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);
            SDDataFile.print(F("=COUNTIF(X2:X13000, "));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F("Restored"));
            SDDataFile.write(34);
            SDDataFile.write(34);
            SDDataFile.print(F(")/2"));
            SDDataFile.write(34);
            break;
          case 55:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Processor"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
#if defined(__MK20DX256__)  // Teensy 3.2
            SDDataFile.print(F("Teensy 3.2"));
#elif defined(__IMXRT1062__)  // Teensy 4.0 or 4.1
            SDDataFile.print(F("Teensy 4.0"));
#else
            SDDataFile.print(F("???"));
#endif
            break;
          case 56:
            EEPROM.get(65, NeedToReplaceBattery);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Backup Battery (CR-2032): "));
            SDDataFile.print(NeedToReplaceBattery ? "REPLACE" : "OK");
            SDDataFile.write(DATA_DELIMITER);
            break;
        }

        // return record back to it's original
        SSD.gotoRecord(i);
      }

      SDDataFile.println();
    }
  }

  SDDataFile.close();

  SSD.gotoRecord(CurrentRecord);
}

void DownloadGPSData(uint32_t Count) {
  bool SDCardStatus = false;
  uint8_t oLapCount = 255;
  uint8_t next = 0, rt = 0;
  uint32_t i = 0, StatusBarWidth = 0;
  char FileName[31] = "C_RR_YYYY-MM-DD_NN_GPS.csv";
  bool OKtoClose = false;
  uint32_t StatusBarCounter = 0;
  uint32_t CurrentRecord = 0;

  CurrentRecord = SSD.getCurrentRecord();

  if (Count == 1) {
    StatusBarCounter = 0;
  }

  Display.setFont(FONT_14);
  Display.fillRect(10, 167, 300, 25, C_WHITE);
  if (digitalRead(CD_PIN) == HIGH) {
    Display.fillRect(10, 168, 134, 25, C_WHITE);
    Display.print(F("NO SD CARD"));
    SSD.gotoRecord(CurrentRecord);
    return;
  }

  SDCardStatus = SDCARD.begin(SDCS_PIN, SD_SCK_MHZ(SD_SPI_SPEED));  //SD

  if (!SDCardStatus) {
    Display.fillRect(10, 168, 134, 25, C_WHITE);
    Display.print(F("NO SD CARD"));
    SSD.gotoRecord(CurrentRecord);
    return;
  }

  SSD.gotoRecord(1);
  LastRecord = SSD.getLastRecord();

  for (i = 1; i <= LastRecord; i++) {
    WatchDogTimer(RESET_WDT);
    SSD.gotoRecord(i);
    rt = SSD.getField(RecordType, frType);

    // reset the pit counter
    if (rt == NULL_RECORD) {
      SDDataFile.close();
      SSD.gotoRecord(CurrentRecord);
      return;
    }

    // advance progress indicator
    StatusBarCounter++;
    StatusBarWidth = ((float)(StatusBarCounter * 293.0) / (Count * LastRecord)) + 2;
    Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

    if (rt == RT_HEADER) {

      // reset the pit counter
      if (OKtoClose) {
        SDDataFile.close();
        OKtoClose = false;
      }

      sprintf(FileName, "C_%02d_%04d-%02d-%02d_00_GPS.csv",
              SSD.getHeaderField(RecordSETID, hrID),
              SSD.getHeaderField(Tyear, hrYear),
              SSD.getHeaderField(Tmonth, hrMonth),
              SSD.getHeaderField(Tday, hrDay));
      if (CarID == BLUE_CAR) {
        FileName[0] = 'B';
      } else if (CarID == RED_CAR) {
        FileName[0] = 'R';
      } else {
        FileName[0] = 'W';
      }
      Display.setTextColor(C_BLACK, C_WHITE);
      Display.fillRect(10, 167, 285, 20, C_WHITE);
      Display.setCursor(10, 168);
      Display.print(FileName);

      while (SDCARD.exists(FileName)) {

        next++;

        FileName[17] = (int)((next / 10) % 10) + '0';
        FileName[18] = (int)(next % 10) + '0';

        Display.setTextColor(C_BLACK, C_WHITE);
        Display.fillRect(10, 167, 285, 20, C_WHITE);
        Display.setCursor(10, 168);
        Display.print(FileName);

        if (next > 999) {
          break;
        }
      }

      SDCardStatus = SDDataFile.open(FileName, O_WRITE | O_CREAT);

      if (!SDCardStatus) {
        Display.setTextColor(C_RED, C_WHITE);
        Display.setCursor(200, 168);
        Display.print(F("No SD Card"));
        delay(1000);
        SSD.gotoRecord(CurrentRecord);
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
      SDDataFile.print(F("GPS Altitude"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("GPS-Speed"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Air-Speed"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel-X"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel-Y"));
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(F("Accel-Z"));
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

      SDDataFile.print(SSD.getField(Volts, frVolts), 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Amps, frAmps), 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Amps, frAmps) * SSD.getField(Volts, frVolts), 0);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(mRPM, frRPM));
      SDDataFile.write(DATA_DELIMITER);
      // very odd.. we store in MPH but the GPS site wants Km/Hr but displays MPH
      SDDataFile.print(SSD.getField(CarSpeed, frSpeed) * MPH_TO_KMPH, 2);

      RealClockTime = SSD.getField(RealClockTime, frRT);
      hr = (int)(RealClockTime / 3600);
      mn = (int)((RealClockTime - (hr * 3600)) / 60);
      sc = (int)(RealClockTime % 60);
      sprintf(buf, "%02d:%02d:%02d", hr, mn, sc);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(buf);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(GPSLon, frLon), 9);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(GPSLat, frLat), 9);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(Altitude, frAltitude) / METERS_TO_FEET, 1);  // very weird but GPS plotting software thinks this is in meters
      SDDataFile.write(DATA_DELIMITER);

      SDDataFile.print(SSD.getField(GPSAltitude, frGPSAltitude), 1);  //
      SDDataFile.write(DATA_DELIMITER);

      SDDataFile.print(SSD.getField(GPSSpeed, frGPSpeed), 2);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(SSD.getField(AirSpeed, frAirSpeed), 1);  //
      SDDataFile.write(DATA_DELIMITER);

      GForceX = SSD.getField(GForceX, frMax);
      GForceY = SSD.getField(GForceY, frMay);
      GForceZ = SSD.getField(GForceZ, frMaz);

      SDDataFile.print(GForceX, 3);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(GForceY, 3);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.print(GForceZ, 3);
      SDDataFile.write(DATA_DELIMITER);

      // compute and print the absolute max g force.
      SDDataFile.print(sqrt((GForceX * GForceX) + (GForceY * GForceY) + (GForceZ * GForceZ)), 3);

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

  SDDataFile.close();

  SSD.gotoRecord(CurrentRecord);
}

void DownloadEEPROM() {
  char SetupFileName[28] = "C_EEPROM_YYYY-MM-DD_NN.txt";
  uint8_t next = 0;
  int StatusBarWidth = 0;
  bool SDCardStatus = false;
  uint32_t StartRecord = 0;
  uint32_t NextRecord = 0;
  uint32_t FirstID = 0;
  uint16_t SensorCalibrationDate = 0;

  Display.setFont(FONT_14);
  Display.fillRect(10, 167, 300, 25, C_WHITE);

  if (digitalRead(CD_PIN) == HIGH) {
    Display.setTextColor(C_RED, C_WHITE);
    Display.fillRect(10, 167, 134, 25, C_WHITE);
    Display.setCursor(10, 168);
    Display.print(F("NO SD CARD"));
    return;
  }

  SDCardStatus = SDCARD.begin(SDCS_PIN, SD_SCK_MHZ(SD_SPI_SPEED));  //SD

  if (!SDCardStatus) {
    Display.setTextColor(C_RED, C_WHITE);
    Display.fillRect(10, 167, 134, 25, C_WHITE);
    Display.setCursor(10, 168);
    Display.print(F("NO SD CARD"));
    return;
  }

  StatusBarWidth = ((float)(1 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

  sprintf(SetupFileName, "X_EEPROM_%04d-%02d-%02d_00.txt", year(), month(), day());

  if (CarID == BLUE_CAR) {
    SetupFileName[0] = 'B';
  } else if (CarID == RED_CAR) {
    SetupFileName[0] = 'R';
  } else if (CarID == WHITE_CAR) {
    SetupFileName[0] = 'W';
  }

  while (SDCARD.exists(SetupFileName)) {
    WatchDogTimer(RESET_WDT);
    next++;

    SetupFileName[20] = (int)((next / 10) % 10) + '0';
    SetupFileName[21] = (int)(next % 10) + '0';
    if (next > 999) {
      return;
    }
  }

  Display.fillRect(10, 167, 285, 20, C_WHITE);
  Display.setTextColor(C_BLACK, C_WHITE);
  Display.setCursor(10, 168);
  Display.print(SetupFileName);


  StatusBarWidth = ((float)(2 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

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

  StatusBarWidth = ((float)(3 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

  SDDataFile.print(F("PATRIOT RACING TELEMETRY SETTINGS - CONFIDENTIAL"));
  SDDataFile.println("");

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
    sprintf(buf, "Report date: %d:%02d:%02d, %d/%d/%d", hour() % 12, minute(), second(), month(), day(), year());
  } else {
    sprintf(buf, "Report date: %d:%02d:%02d, %d/%d/%d", hour(), minute(), second(), month(), day(), year());
  }
  SDDataFile.println(buf);
  SDDataFile.println();

  StatusBarWidth = ((float)(4 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

  SDDataFile.println(F("Hardware Information"));
  SDDataFile.print(F("Processor: "));
#if defined(__MK20DX256__)  // Teensy 3.2
  SDDataFile.println(F("Teensy 3.2"));
#elif defined(__IMXRT1062__)  // Teensy 4.0 or 4.1
  SDDataFile.println(F("Teensy 4.0"));
#else
  SDDataFile.println(F("???"));
#endif

  EEPROM.get(65, NeedToReplaceBattery);
  SDDataFile.print(F("Backup Battery (CR-2032): "));
  SDDataFile.println(NeedToReplaceBattery ? "REPLACE" : "OK");

  SDDataFile.print(F("Memory chip JEDEC: "));
  SDDataFile.println(SSD.getChipJEDEC());
  SDDataFile.print(F("SSD record length: "));
  SDDataFile.println(SSD.getDatabaseRecordLength());
  SDDataFile.print(F("Database Record length: "));
  SDDataFile.println(SSD.getRecordLength());
  SDDataFile.print(F("Database Fields: "));
  SDDataFile.println(SSD.getFieldCount());
  SDDataFile.print(F("Header record length: "));
  SDDataFile.println(SSD.getHeaderRecordLength());
  SDDataFile.print(F("Header Fields: "));
  SDDataFile.println(SSD.getHeaderFieldCount());
  SDDataFile.print(F("Total Records: "));
  SDDataFile.println(SSD.getLastRecord());
  SDDataFile.print(F("SSD Size: "));
  SDDataFile.println(SSD.getTotalSpace());
  SDDataFile.print(F("SSD Used: "));
  SDDataFile.println(SSD.getUsedSpace());
  SDDataFile.print(F("Datapoint Averages: "));
  SDDataFile.println(AverageCounter * (1000 / UPDATE_LIMIT));

  SSD.gotoRecord(1);
  FirstID = SSD.getField(RecordSETID, hrID);
  if (SSD.getUsedSpace() == 0) {
    SDDataFile.println(F("Total RecordSets: 0"));
  } else {
    SDDataFile.print(F("Total RecordSets: "));
    SDDataFile.println(RecordSETID - FirstID + 1);
  }
  StartRecord = 0;

  if (SSD.getUsedSpace() > 0) {
    for (i = 1; i <= RecordSETID; i++) {
      StartRecord = SSD.getFirstRecord(i, hrID);
      NextRecord = SSD.getFirstRecord(i + 1, hrID);
      if (NextRecord == 0) {
        NextRecord = SSD.getLastRecord();
      }
      SSD.gotoRecord(NextRecord);

      SDDataFile.print(F("Record Set: "));
      SDDataFile.print(i);
      SDDataFile.print(F(", Records: "));
      SDDataFile.print(StartRecord);
      SDDataFile.print(F("-"));
      SDDataFile.print(NextRecord - 1);
      SDDataFile.print(F(" ("));
      SDDataFile.print(NextRecord - StartRecord - 1);
      SDDataFile.print(F("), Race Start "));
      SSD.gotoRecord(StartRecord);
      Tmonth = SSD.getHeaderField(Tmonth, hrMonth);
      Tday = SSD.getHeaderField(Tday, hrDay);
      Tyear = SSD.getHeaderField(Tyear, hrYear);
      Thour = SSD.getHeaderField(Thour, hrHour);
      Tminute = SSD.getHeaderField(Tminute, hrMinute);
      sprintf(buf, "Date: %d/%d/%d, Time: %d:%02d", Tmonth, Tday, Tyear, Thour, Tminute);
      SDDataFile.println(buf);
    }
  }

  SDDataFile.println();
  //////////////////////////////////////////////////////
  SDDataFile.println(F("Datalogger Settings"));
  SDDataFile.print(F("Display theme: "));
  SDDataFile.println(Theme ? "Light" : "Dark");
  SDDataFile.print(F("Button orientation: "));
  SDDataFile.println(Orientation ? "Buttons Up" : "Buttons Down");
  SDDataFile.print(F("Update [ms]: "));
  SDDataFile.println(UPDATE_LIMIT);
  SDDataFile.print(F("Start/Change trigger [amps]: "));
  SDDataFile.println(TriggerAmps);
  SDDataFile.print(F("Restart display each draw: "));
  SDDataFile.println(RestartDisplayAlways ? "Yes" : "No");
  SDDataFile.println();
  StatusBarWidth = ((float)(5 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

  //////////////////////////////////////////////////////
  SDDataFile.println(F("Software Versions"));
  SDDataFile.print(F("Code: "));
  SDDataFile.println(CODE_VERSION);
  SDDataFile.print(F("PatriotRacing_Utilities: "));
  SDDataFile.println(UTILITIES_VERSION);
  SDDataFile.print(F("BulletDB: "));
  SDDataFile.println(BULLET_DB_VER);
  SDDataFile.print(F("ILI9341_t3_Menu: "));
  SDDataFile.println(ILI9341_MENU_VER);
  SDDataFile.print(F("ILI9341_t3_Controls: "));
  SDDataFile.println(ILI9341_t3_CONTROLS_VER);
  SDDataFile.print(F("FlickerFreePrint: "));
  SDDataFile.println(ILI9341_FLICKER_FREE_PRINT_VER);
  SDDataFile.print(F("EBYTE: "));
  SDDataFile.println(EBYTE_E220_VER);
  SDDataFile.print(F("MS5837: "));
  SDDataFile.println(MS5837_VERSION);
  SDDataFile.print(F("MCP3208: "));
  SDDataFile.println(MPC3208_VER);
  SDDataFile.print(F("XGZP6897D Address: "));
  SDDataFile.println(I2C_device_address, HEX);
  SDDataFile.print(F("GPS: "));
  SDDataFile.println(_GPS_VERSION);
  SDDataFile.print(F("SDfat: "));
  SDDataFile.println(SD_FAT_VERSION_STR);
  SDDataFile.println();
  StatusBarWidth = ((float)(6 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

  //////////////////////////////////////////////////////
  SDDataFile.println(F("Calibration Information"));
  EEPROM.get(2, SensorCalibrationDate);
  SDDataFile.print(F("Voltage Sensor Calibration Date: "));
  SDDataFile.print(SensorCalibrationDate >> 12);
  SDDataFile.print(F("/"));
  SDDataFile.print((SensorCalibrationDate >> 7 & 0b0000000000011111));
  SDDataFile.print(F("/"));
  SDDataFile.println(2026 + (SensorCalibrationDate & 0b00000000001111111));
  EEPROM.get(4, SensorCalibrationDate);
  SDDataFile.print(F("Current Sensor Calibration Date: "));
  SDDataFile.print(SensorCalibrationDate >> 12);
  SDDataFile.print(F("/"));
  SDDataFile.print((SensorCalibrationDate >> 7 & 0b0000000000011111));
  SDDataFile.print(F("/"));
  SDDataFile.println(2026 + (SensorCalibrationDate & 0b00000000001111111));
  EEPROM.get(6, SensorCalibrationDate);
  SDDataFile.print(F("Temperature Sensor Calibration Date: "));
  SDDataFile.print(SensorCalibrationDate >> 12);
  SDDataFile.print(F("/"));
  SDDataFile.print((SensorCalibrationDate >> 7 & 0b0000000000011111));
  SDDataFile.print(F("/"));
  SDDataFile.println(2026 + (SensorCalibrationDate & 0b00000000001111111));
  EEPROM.get(8, SensorCalibrationDate);
  SDDataFile.print(F("Accelerometer Calibration Date: "));
  SDDataFile.print(SensorCalibrationDate >> 12);
  SDDataFile.print(F("/"));
  SDDataFile.print((SensorCalibrationDate >> 7 & 0b0000000000011111));
  SDDataFile.print(F("/"));
  SDDataFile.println(2026 + (SensorCalibrationDate & 0b00000000001111111));
  SDDataFile.print(F("Voltage Slope: "));
  SDDataFile.println(VoltageSlope, 3);
  SDDataFile.print(F("Voltage Offset: "));
  SDDataFile.println(VoltageOffset, 3);
  SDDataFile.print(F("Zero voltage at startup: "));
  SDDataFile.println(AutoCurrentCal ? "Yes" : "No");
  SDDataFile.print(F("V@0 amp: "));
  SDDataFile.println(VMid, 3);
  SDDataFile.print(F("mV/Amp: "));
  SDDataFile.println(mVPerAmp, 3);
  SDDataFile.print(F("Pickups: "));
  SDDataFile.println(Pickups);
  SDDataFile.print(F("Air Flow Sensor Enabled: "));
  SDDataFile.println(EnableAirFlowSensor ? "Yes" : "No");
  SDDataFile.print(F("Air Speed Offset: "));
  SDDataFile.println(AirSpeedOffset);
  SDDataFile.print(F("Temp resistor Motor: "));
  SDDataFile.println(ThermResMotor);
  SDDataFile.print(F("Temp resistor Aux: "));
  SDDataFile.println(ThermResAux);
  SDDataFile.print(F("Ambient temp offset: "));
  SDDataFile.println(AmbTempCF);
  SDDataFile.print(F("GPS Altitude correction [ft]: "));
  SDDataFile.println(GPSAltCorrection);
  SDDataFile.print(F("Accel direction: "));
  SDDataFile.println(ASensorDirectionText[ASensorDirection]);
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
  SDDataFile.println();

  //////////////////////////////////////////////////////
  SDDataFile.println(F("CYBORG Setup"));
  SDDataFile.print(F("Cyborg Enabled: "));
  SDDataFile.println(EnableCyborg ? "Yes" : "No");
  SDDataFile.print(F("Prediction sample size: "));
  SDDataFile.println(PREDICT_SAMPLES);
  SDDataFile.print(F("Prediction compensation [A]: "));
  SDDataFile.println(PredictionCompensation, 0);
  SDDataFile.print(F("Cyborg to manage: "));
  SDDataFile.println(CyborgInputText[CyborgInput]);
  SDDataFile.print(F("Cyborg 1st current range: "));
  SDDataFile.print(CyborgMinRange);
  SDDataFile.print(F("/"));
  SDDataFile.println(CyborgMaxRange);
  EEPROM.get(485, CyborgFirstLimit);
  SDDataFile.print(F("Cyborg 1st current limit: "));
  SDDataFile.println(CyborgFirstLimit);
  SDDataFile.print(F("Cyborg 2nd current limit: "));
  SDDataFile.println(CyborgSecondLimit);
  SDDataFile.print(F("Cyborg update time [ms]: "));
  SDDataFile.println(CyborgUpdateTime);
  SDDataFile.print(F("Cyborg min range: "));
  SDDataFile.println(ThrottleMinRange);
  SDDataFile.print(F("Cyborg max range: "));
  SDDataFile.println(ThrottleMaxRange);
  SDDataFile.print(F("ESC off threshold: "));
  SDDataFile.println(ESCVoltsOff);
  SDDataFile.print(F("ESC on threshold: "));
  SDDataFile.println(ESCVoltsOn);
  SDDataFile.print(F("Kp tuning parameter: "));
  SDDataFile.println(Kp);
  SDDataFile.print(F("Ki tuning parameter: "));
  SDDataFile.println(Ki);
  SDDataFile.print(F("Kd tuning parameter: "));
  SDDataFile.println(Kd);
  SDDataFile.println();
  StatusBarWidth = ((float)(7 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

  //////////////////////////////////////////////////////
  SDDataFile.println(F("Mechanical Information"));
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
    SDDataFile.print(F("Pressure [psi]: "));
    SDDataFile.println(TirePressureFront);
    SDDataFile.print(F("Radius [in]: "));
    SDDataFile.println(TireRadius[TireID], 3);
  } else {
    SDDataFile.print(F("Car: UNKNOWN"));
  }
  SDDataFile.println();
  StatusBarWidth = ((float)(8 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);
  StatusBarWidth = ((float)(9 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

  //////////////////////////////////////////////////////
  SDDataFile.println(F("Race Settings Information"));
  SDDataFile.print(F("Total Energy: "));
  SDDataFile.println(TotalEnergy);
  SDDataFile.print(F("Battery 1: "));
  SDDataFile.println(Battery1);
  SDDataFile.print(F("Battery 2: "));
  SDDataFile.println(Battery2);
  SDDataFile.print(F("Battery Warning [volts]: "));
  SDDataFile.println(WARNING_BATTERY);
  SDDataFile.print(F("Lap Amp Warning: "));
  SDDataFile.println(WARNING_LAPAMP);
  SDDataFile.print(F("Temp Warning [deg F]: "));
  SDDataFile.println(WARNING_MTEMP);
  SDDataFile.println();

  //////////////////////////////////////////////////////
  SDDataFile.println(F("Wireless Information"));
  sprintf(buf, "%s", Radio.getModel());
  SDDataFile.print(F("Transceiver Model: "));
  SDDataFile.println(buf);
  SDDataFile.print(F("Address Keys L/H: "));
  SDDataFile.print(RadioAddressL);
  SDDataFile.print(" / ");
  SDDataFile.println(RadioAddressH);
  SDDataFile.print(F("Transceiver air data rate: "));
  if (Radio.getAirDataRate() < ((sizeof(AirRateText) / sizeof(AirRateText[0])))) {
    SDDataFile.println(AirRateText[Radio.getAirDataRate()]);
  }
  SDDataFile.print(F("Transceiver radio power: "));
  if (Radio.getTransmitPower() < ((sizeof(PowerText) / sizeof(PowerText[0])))) {
    SDDataFile.println(PowerText[Radio.getTransmitPower()]);
  }
  SDDataFile.print(F("Transceiver channel: "));
  SDDataFile.println(Radio.getChannel());
  SDDataFile.print(F("Send time [s]: "));
  SDDataFile.println(RadioUpdate);
  SDDataFile.print(F("Use GPS speed when displaying car speed: "));
  SDDataFile.print(F("Lap trigger range [m]: "));
  SDDataFile.println(GPSTolerance);
  SDDataFile.print(F("Add lap when pitting: "));
  SDDataFile.println(AddLapInPit ? "Yes" : "No");
  SDDataFile.print(F("Delay GPS read at start [s]: "));
  SDDataFile.println(StartGPSDelay);
  SDDataFile.print(F("Lap Threshold [s]: "));
  SDDataFile.println(LapThreshold);
  SDDataFile.println();

  //////////////////////////////////////////////////////
  SDDataFile.println(F("END CAR PARAMETERS"));

  SDDataFile.close();

  StatusBarWidth = ((float)(10 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);
  delay(500);
}


/*
  PURPOSE : Screen settings
  PARAMS: -
  RETURNS : None
  NOTES:
*/

void SetScreenParameters() {
  if (Theme == 0) {
    // dark, black background
    BackColor = C_BLACK;
    ForeColor = C_WHITE;
    InactiveColor = C_DKGREY;
    DetailsColor = C_VDKGREY;
  } else {
    // light, white background
    BackColor = C_WHITE;
    ForeColor = C_BLACK;
    InactiveColor = C_GREY;
    DetailsColor = C_LTGREY;
  }

  if (Orientation == 0) {
    Display.setRotation(1);
  } else if (Orientation == 1) {
    Display.setRotation(3);
  } else {
    Display.setRotation(1);  //default setting
  }

  EnergyG.setColors(ForeColor, C_DKGREY, C_BLUE, BackColor, InactiveColor);
  TRemG.setColors(ForeColor, BackColor, C_BLUE, BackColor, InactiveColor);
  ERemG.setColors(ForeColor, BackColor, C_BLUE, BackColor, InactiveColor);
  MotorTempG.setColors(ForeColor, BackColor, C_BLUE, BackColor, BackColor);
  AuxTempG.setColors(ForeColor, BackColor, C_BLUE, BackColor, BackColor);
  AmbTempG.setColors(ForeColor, BackColor, C_BLUE, BackColor, BackColor);

  MotorTempG.setSectionColors(C_GREEN, C_YELLOW, C_RED, InactiveColor);
  AuxTempG.setSectionColors(C_GREEN, C_YELLOW, C_RED, InactiveColor);
  AmbTempG.setSectionColors(C_GREEN, C_YELLOW, C_RED, InactiveColor);
  TRemG.setSectionColors(C_LTBLUE, C_LTBLUE, C_LTBLUE, InactiveColor);
  ERemG.setSectionColors(C_GREEN, C_YELLOW, C_RED, InactiveColor);
  RedrawHeader = true;
}


/*
  PURPOSE : Build date strings if time is millis
  PARAMS :  -
  RETURNS : None
  NOTES :
*/
void BuildDateStringMS(unsigned long val) {
  val = val / 1000;
  hr = (int)(val / 3600);
  mn = (int)((val - (hr * 3600)) / 60);
  sc = (int)(val % 60);
  sprintf(buf, "%02d:%02d:%02d", hr, mn, sc);
}

/*
  PURPOSE : Build date strings if time is seconds
  PARAMS :  -
  RETURNS : None
  NOTES :
*/
void BuildDateStringS(unsigned long val) {
  hr = (int)(val / 3600);
  mn = (int)((val - (hr * 3600)) / 60);
  sc = (int)(val % 60);
  sprintf(buf, "%02d:%02d:%02d", hr, mn, sc);
}

/*
  PURPOSE : creates the database schema--change this and SSD chip must be erased
  PARAMS :  -
  RETURNS : None
  NOTES :
*/
void BuildFieldList() {
  // adding fields is fine, but you may need to adjust the limit the library is capped at
  // in BulletDB.h change the limit as needed #define MAX_FIELDS 25

  frType = SSD.addField(&RecordType);                    // Type 1
  frID = SSD.addField(&RecordSETID);                     // Recordset ID 1
  frLap = SSD.addField(&LapCount);                       // Lap Count 1
  frDriver = SSD.addField(&Driver);                      // Driver 1
  frPoint = SSD.addField(&Point);                        // Point 2
  frVolts = SSD.addField(&Volts);                        // Volts 4
  frAmps = SSD.addField(&Amps);                          // Amps 4
  frMotorTemp = SSD.addField(&MotorTemp);                // MotorTemp 4
  frAuxTemp = SSD.addField(&AuxTemp);                    // AuxTemp 4
  frAmbTemp = SSD.addField(&AmbTemp);                    // AmbTemp 4
  frEnergy = SSD.addField(&Energy);                      // Energy 4
  frAmpHours = SSD.addField(&AmpHours);                  // AmpHours 4
  frPredictedEnergy = SSD.addField(&PredictedEnergy);    // Energy
  frCyborgFirstLimit = SSD.addField(&CyborgFirstLimit);  // Amps tune
  frRPM = SSD.addField(&mRPM);                           // MRPM 4
  frWRPM = SSD.addField(&WRPM);                          // MRPM 4
  frSpeed = SSD.addField(&CarSpeed);                     // Speed 4
  frDist = SSD.addField(&Distance);                      // "Distance 4
  frRT = SSD.addField(&RealClockTime);                   // RealClockTime 4
  frLon = SSD.addField(&GPSLon);                         // GPSLon 4
  frLat = SSD.addField(&GPSLat);                         // GPSLat 4
  frAltitude = SSD.addField(&Altitude);                  // Alt 4
  frGPSpeed = SSD.addField(&GPSSpeed);                   // GPSSpeed 4
  frMax = SSD.addField(&GForceX);                        // Max X 4
  frMay = SSD.addField(&GForceY);                        // Max Y 4
  frMaz = SSD.addField(&GForceZ);                        // Max Z 4
  frRestoreType = SSD.addField(&RestoreType);            // RestoreType 1
  frCyborgInSignal = SSD.addField(&CyborgInSignal);      // Cyborg Input 1
  frCyborgOutSignal = SSD.addField(&CyborgOutSignal);    // Cyborg Output 1
  frAirSpeed = SSD.addField(&AirSpeed);                  // air speed
  frGPSAltitude = SSD.addField(&GPSAltitude);            // GPS altitude
  frDisplayID = SSD.addField(&DisplayID);                // display ID
  frTrackLength = SSD.addField(&TrackLength);            // display ID
  frLapTime = SSD.addField(&LapTime);                    // display ID

  // you cannot have more header fields that data fields--BulletDB.h rule
  // header fields can be any type
  // headers are simply the first record and can hold setup data (start time for example) to be printed later
  hrType = SSD.addHeaderField(&RecordType);                      // "Record Type" 1
  hrID = SSD.addHeaderField(&RecordSETID);                       // "Recordset ID" 2
  hrYear = SSD.addHeaderField(&Tyear);                           // "Year" 2
  hrMonth = SSD.addHeaderField(&Tmonth);                         // "Month" 2
  hrDay = SSD.addHeaderField(&Tday);                             // "Day" 2
  hrHour = SSD.addHeaderField(&Thour);                           // "Hour" 2
  hrMinute = SSD.addHeaderField(&Tminute);                       // "Minute" 2
  hrMSprocket = SSD.addHeaderField(&MotorSprocket);              // "Motor Sprocket" 1
  hrWSprocket = SSD.addHeaderField(&WheelSprocket);              // "Wheel Sprocket" 1
  hrTirePressureFront = SSD.addHeaderField(&TirePressureFront);  // "Tire Pressure" front
  hrTirePressureRear = SSD.addHeaderField(&TirePressureRear);    // "Tire Pressure back
  hrMotorID = SSD.addHeaderField(&MotorID);                      // "Motor ID" 1
  hrTemp = SSD.addHeaderField(&AmbTemp);                         // "Amb Temp" 4
  hrStartAltitude = SSD.addHeaderField(&Altitude);               // "Start Alt" 4
  hrStartPressure = SSD.addHeaderField(&AtmPressure);            // "Start pressure"
  hrEnergy = SSD.addHeaderField(&TotalEnergy);                   // "Energy"
  hrCounter = SSD.addHeaderField(&AverageCounter);               // "Counter"
  hrBattery1 = SSD.addHeaderField(&Battery1);                    // "Battery 1"
  hrBattery2 = SSD.addHeaderField(&Battery2);                    // "Battery 2"
}

// this is in a function to setup accelerometer parameters
// we may have to restart during a race or we may change settings in menu
// all this code needed after a restart or parameter change
void SetupAccelerometer() {
  // probably need to bounds check other array type stuff (tire ID for example)
  if (GForceRange < ((sizeof(AccelFSRange) / sizeof(AccelFSRange[0])))) {
  } else {
    GForceRange = 0;
  }

  AccelSensor.setZeroMotionDetectionThreshold(2);
  AccelSensor.setDLPFMode(AccelLPFilter);
  AccelSensor.setDHPFMode(AccelHPFilter);
  AccelSensor.setFullScaleGyroRange((MPU6050_IMU::MPU6050_GYRO_FS_1000));
  AccelSensor.setFullScaleAccelRange(GForceRange);
  AccelSensor.setXAccelOffset(AccelCalX);
  AccelSensor.setYAccelOffset(AccelCalY);
  AccelSensor.setZAccelOffset(AccelCalZ);

  ASensorBits = AccelFSBits[GForceRange];

  delay(50);
}

/*
  PURPOSE : fire up the GPS
  PARAMS :  -
  RETURNS : None
  NOTES :
*/
void SetupGPS() {
  // bounds check GPS start delay
  if (StartGPSDelayID < ((sizeof(GPSReadTimeText) / sizeof(GPSReadTimeText[0])))) {
    StartGPSDelay = GPSReadTime[StartGPSDelayID];
  } else {
    StartGPSDelay = 0;
  }
}


/*
  PURPOSE : initilizes most sensors, some are done in setup as the are dependent (Display for example)
  PARAMS :  -
  RETURNS : None
  NOTES :
*/
void InitializeDevices() {
  Display.setCursor(STATUS_RESULT, 40);

  if (SSDStatus) {
    Display.setTextColor(C_GREEN);
    Display.print(SSD.getChipJEDEC());
  } else {
    Display.setTextColor(C_RED);
    Display.print(F("SSD FAIL"));
  }

  if (EXTADCStatus) {
    Display.setTextColor(C_GREEN);
    Display.print(F(" / OK"));
  } else {
    Display.setTextColor(C_RED);
    Display.print(F(" / FAIL"));
  }

  Display.setCursor(STATUS_RESULT, 60);

  // init the accelerometer
  GForceStatus = false;

  AccelSensor.initialize();

  GForceStatus = AccelSensor.isConnected();

  if (GForceStatus) {
    delay(50);
    SetupAccelerometer();
    Display.setTextColor(C_GREEN);
    ax = AccelSensor.getAccelerationX() / ASensorBits;
    ay = AccelSensor.getAccelerationY() / ASensorBits;
    az = AccelSensor.getAccelerationZ() / ASensorBits;
  } else {
    Display.setTextColor(C_RED);
    Display.print(F("Fail: "));
    Warnings = Warnings | GFORCE_WARNING;
  }


  Display.print(ax, 2);
  Display.print(F(", "));
  Display.print(ay, 2);
  Display.print(F(", "));
  Display.print(az, 2);

  delay(10);

  // get the battery voltage

  vVolts = 0;
  for (i = 0; i < 50; i++) {
    vVolts = vVolts + EXTADC.analogRead(EXTADC_VM_PIN);
    delay(10);
  }
  vVolts = vVolts / 50.0f;
  vVolts = vVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);
  Volts = (vVolts * VoltageSlope) + VoltageOffset;

  Display.setCursor(STATUS_RESULT, 80);

  if ((Volts < WARNING_BATTERY) || (Volts > 30.0f)) {
    Display.setTextColor(C_RED);
    Display.print(Volts, 1);
  } else {
    Display.setTextColor(C_GREEN);
    Display.print(Volts, 1);
  }
  Display.print(F(" / "));
  Display.setTextColor(C_GREEN);
  vVolts = 0;
  // data logger may have been started when key is on
  // or we may need to reset race when key is on

  if ((AutoCurrentCal) && (RaceStatus == RACE_NOTSTARTED)) {

    aVolts = 0;
    for (i = 0; i < 50; i++) {
      aVolts = aVolts + EXTADC.analogRead(EXTADC_AM_PIN);
      delay(10);
    }
    aVolts = aVolts / 50.0f;
    aVolts = aVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);
    // datalogger consumes 0.066 amps and since its current is not running through
    // current sensor, we're just compensate the offset and subtract 0.001 volts
    // possible 2 different sensors (100U and 200U)
    if (mVPerAmp > 30) {
      VMid = aVolts - 0.002;
    } else {
      VMid = aVolts - 0.001;
    }

    EEPROM.put(230, VMid);
    delay(10);
    // update the menu value
    SensorMenu.SetItemValue(SensorMenuID4, VMid);
    Display.setTextColor(C_CYAN);
  } else {
    aVolts = 0;
    for (i = 0; i < 50; i++) {
      aVolts = aVolts + EXTADC.analogRead(EXTADC_AM_PIN);
      delay(10);
    }
    aVolts = aVolts / 50.0f;
    aVolts = aVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);
    SensorMenu.SetItemValue(SensorMenuID4, VMid);
    Display.setTextColor(C_GREEN);
  }

  // get current draw
  Amps = ((aVolts - VMid) * 1000.0f) / mVPerAmp;

  if ((Amps < -2.0f) || (Amps > 70.0f)) {
    Display.setTextColor(C_RED);
    Display.print(Amps, 3);
  } else {
    Display.print(Amps, 3);
  }
  aVolts = 0;

  thmVolts = 0;
  for (i = 0; i < 50; i++) {
    thmVolts = thmVolts + EXTADC.analogRead(EXTADC_THM_PIN);
    delay(10);
  }
  // test Motor temp sensor
  thmVolts = thmVolts / 50;
  thmVolts = thmVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);
  ThermistorResistence = (thmVolts * ThermResMotor) / (REFERENCE_VOLTAGE - thmVolts);
  TempK = 1.0 / (NTC_A + (NTC_B * (log(ThermistorResistence / 10000.0f))) + (NTC_C * pow(log(ThermistorResistence / 10000.0f), 2)) + (NTC_D * pow(log(ThermistorResistence / 10000.0f), 3)));
  MotorTemp = (TempK * 1.8f) - 459.67f;

  if ((MotorTemp < 10.0f) || (MotorTemp > WARNING_MTEMP)) {
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
  thxVolts = 0;
  for (i = 0; i < 50; i++) {
    thxVolts = thxVolts + EXTADC.analogRead(EXTADC_THX_PIN);
    delay(10);
  }
  thxVolts = thxVolts / 50;
  thxVolts = (EXTADC.analogRead(EXTADC_THX_PIN));
  thxVolts = thxVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);
  ThermistorResistence = (thxVolts * ThermResAux) / (REFERENCE_VOLTAGE - thxVolts);
  TempK = 1.0 / (NTC_A + (NTC_B * (log(ThermistorResistence / 10000.0f))) + (NTC_C * pow(log(ThermistorResistence / 10000.0f), 2)) + (NTC_D * pow(log(ThermistorResistence / 10000.0f), 3)));
  AuxTemp = (TempK * 1.8f) - 459.67f;

  if ((AuxTemp < 10.0f) || (AuxTemp > WARNING_MTEMP)) {
    Display.setTextColor(C_RED);
    Display.print(F(" / "));
    Display.print("NC");
    Warnings = Warnings | TEMP_WARNING;
  } else {
    Display.setTextColor(C_GREEN);
    Display.print(F(" / "));
    Display.print(AuxTemp, 0);
  }

  AltimiterStatus = PressureSensor.init();

  if (!AltimiterStatus) {
    Display.setTextColor(C_RED);
    Display.print(F("!ALT"));
  } else {
    Display.setTextColor(C_GREEN);
    PressureSensor.setFluidDensity(MS5837_SEALEVEL_MBAR);
    PressureSensor.setResolution(MS5837_OSR_4096);
    PressureSensor.read();
    Altitude = (PressureSensor.getAltitude() * METERS_TO_FEET) + AltCorrection;
    AtmPressure = PressureSensor.getPressure();
    AmbTemp = PressureSensor.getTemperature();
    AmbTemp = (AmbTemp * 1.8) + 32.0 + AmbTempCF;
    Display.print(F(" / "));
    Display.print(AmbTemp, 0);
    Display.print(F(", "));
    Display.print(Altitude, 0);
  }

  Display.setCursor(STATUS_RESULT, 120);
  digitalWrite(LAPLED_PIN, HIGH);

  // test
  GPSLapTimer = 0;
  // alert pit that it's time to test GPS

  GPSStatus = false;
  while (GPSLapTimer < 5000) {  // test for 5 sec

    GPSRead();
    GPSAltitude = (GPSSensor.altitude.meters() * METERS_TO_FEET) + GPSAltCorrection;
    GPSLat = GPSSensor.location.lat();
    GPSLon = GPSSensor.location.lng();
    GPSStatus = GPSSensor.location.isValid();
    if (GPSStatus) {
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
    digitalWrite(LAPLED_PIN, LOW);
  }

#if defined(__MK20DX256__)  // Teensy 3.2
  RPMStatus = RPMSensor.begin(RPM_PIN);

#elif defined(__IMXRT1062__)  // Teensy 4.0 or 4.1
  FreqMeasure.begin();
  RPMStatus = true;
#else
  RPMStatus = RPMSensor.begin(RPM_PIN);
#endif

  Display.setCursor(STATUS_RESULT, 140);

  ThrottleInputBits = EXTADC.analogRead(EXTADC_THROTTLE_PIN);
  Volts = ThrottleInputBits / (EXADC_BIT_CONVERSION / EXADC_VREF);

  if (Volts > 0.4f) {  // off is .6 or so, full on should be 3.25
    Display.setTextColor(C_GREEN);
    Display.print(Volts, 2);
  } else {
    Display.setTextColor(C_RED);
    Display.print(Volts, 2);
    Warnings = Warnings | THROTTLE_WARNING;
  }

  if (!RPMStatus) {
    Display.setTextColor(C_RED);
    Display.print(F(" / !RPM / "));
  } else {
    Display.setTextColor(C_GREEN);
    Display.print(F(" / RPM OK / "));
  }

  if (EnableAirFlowSensor) {
    AirSpeed = 0;
    AirFlowSensorStatus = false;
    if (AirFlowSensor.begin()) {
      AirFlowSensorStatus = true;
      if (AirFlowSensor.readSensor(AmbTemp, ASPressure)) {
        // sensor is giving pascals
        // need to convert to inches of water
        ASPressure = ASPressure * 0.00401865f;
        if (ASPressure < 0.0f) {
          ASPressure = 0;
        }
        // this equation needs inches of water
        fpm = sqrt(ASPressure) * 4005.0f;
        // convert feet per minute to miles per hour
        AirSpeed = (fpm * 0.0113636f) + AirSpeedOffset;
        if (AirSpeed < 0.0f) {
          AirSpeed = 0;
        }
      }
      Display.setTextColor(C_GREEN);
      Display.print(AirSpeed, 1);
    } else {
      Display.setTextColor(C_RED);
      Display.print(F("!AFS"));
      Warnings = Warnings | SPEED_WARNING;
    }
  } else {
    Display.setTextColor(C_RED);
    Display.print(F("!AFS"));
  }

  // test transceiver
  delay(100);

  if (RadioUpdate != 0) {
    RadioStatus = Radio.init();
    DataPacket.begin(details(Data), &ESerial);

    if (!RadioStatus) {
      // RestoreEBYTEDefaults();
      Display.setTextColor(C_RED);
      Display.setCursor(STATUS_RESULT, 160);
      Display.print(F("FAIL"));
      Warnings = Warnings | EBYTE_FAIL;
    } else {

      Radio.setPacketSize(SUB_64BYTES);
      RadioChannel = Radio.getChannel();
      AirDataRate = Radio.getAirDataRate();
      RadioPower = Radio.getTransmitPower();
      RadioAddressL = Radio.getAddressL();
      RadioAddressH = Radio.getAddressH();

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
    Radio.printParameters();
    Serial.println(F("******* End EBYTE Parameters *******"));
#endif
  }

  // get set key statge

  if (EXTADC.analogRead(EXTADC_KEY_PIN) > DIGITAL_ON_LIMIT) {
    KeyState = HIGH;
    OldKeyState = LOW;
    banner_back = C_DKRED;
  } else {
    KeyState = LOW;
    OldKeyState = LOW;
    banner_back = C_DKGREEN;
  }

  Warnings = Warnings | KEY_OFF;

  // get SSD status

  UsedSpace = 0;
  if (!SSDStatus) {
    Display.setTextColor(C_RED);
    Display.setCursor(STATUS_RESULT, 180);
    Display.print(F("FAIL"));
    Warnings = Warnings | SSD_FAIL;
  } else {
    UsedSpace = SSD.getUsedSpace();
    if ((UsedSpace + 900000) < SSD.getTotalSpace()) {  // approx 800K of data per race
      Display.fillRoundRect(STATUS_RESULT, 180, 160, 18, 2, C_DKGREEN);
      Display.fillRoundRect(STATUS_RESULT, 180, ((float)(UsedSpace * 160.0) / SSD.getTotalSpace()) + 2, 18, 2, C_GREEN);
    } else {
      Display.fillRoundRect(STATUS_RESULT, 180, 160, 18, 2, C_RED);
      SSDStatus = false;
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

  CyborgFL = analogRead(EXTADC_CYBORGFIRSTLIMIT_PIN) / (EXADC_BIT_CONVERSION / EXADC_VREF);

  CyborgFirstLimit = FloatMap(CyborgFL, 0.0f, 3.3f, CyborgMinRange, CyborgMaxRange);
  CyborgFirstLimit = (int)((CyborgFirstLimit + 0.05f) * 10.0f) / 10.0f;
  OldCyborgFirstLimit = CyborgFirstLimit;

  // set PID for CYBORG
  Input = 0.0f;
  Setpoint = CyborgFirstLimit;

  CyborgPID.SetMode(AUTOMATIC);

  // limits of operation
  // low 800 * 3.3 / 4096 = 0.6 volts (CYBORG_LOWER_LIMIT = 800)
  // high = 4096 * 3.3 / 4096 = 3.3 volts
  // we typically set the ESC to off at 1.1 on at 3.1 (to give us tolerance for
  // temperature and other electronic tolerances)
  CyborgPID.SetOutputLimits(CYBORG_LOWER_LIMIT, EXADC_BIT_CONVERSION);
  //CyborgPID.SetOutputLimits((ESCVoltsOff + .2) * (EXADC_BIT_CONVERSION / EXADC_VREF), (ESCVoltsOn - .2) * (EXADC_BIT_CONVERSION / EXADC_VREF));
  CyborgPID.SetTunings(Kp, Ki, Kd);
}

/*
  PURPOSE : RTC time functions
  PARAMS :  -
  RETURNS : None
  NOTES :
*/
time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void StartRTC() {
  setSyncProvider(getTeensy3Time);

  Teensy3Clock.set(now());
}

void meansensors(int *mean_ax, int *mean_ay, int *mean_az, int *mean_gx, int *mean_gy, int *mean_gz) {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  int16_t ax, ay, az, gx, gy, gz;
  int buffersize = 500;  //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)

  while (i < (buffersize + 101)) {

    // read raw accel/gyro measurements from device
    AccelSensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    WatchDogTimer(RESET_WDT);
    if (i > 100 && i <= (buffersize + 100)) {  //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      *mean_ax = buff_ax / buffersize;
      *mean_ay = buff_ay / buffersize;
      *mean_az = buff_az / buffersize;
      *mean_gx = buff_gx / buffersize;
      *mean_gy = buff_gy / buffersize;
      *mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(10);  // was 2 Needed so we don't get repeated measures
  }
}

void CalibrateThrottle() {
  uint16_t StatusBarWidth = 0;
  uint16_t AvgCounter = 0;
  uint8_t StatusCounter = 0;
  float MinVal = 999.0f, MaxVal = -999.0f;

  Display.fillRect(39, 26, 253, 195, C_WHITE);
  Display.fillRect(42, 30, 246, 188, C_BLUE);
  Display.setFont(FONT_16B);
  Display.setTextColor(C_WHITE);
  Display.setCursor(65, 43);
  Display.print(F("Calibrate Throttle"));

  Display.setFont(FONT_14);
  Display.setTextColor(C_WHITE);
  Display.setCursor(59, 75);
  Display.print(F("Turn throttle from max"));
  Display.setCursor(59, 102);

  Display.print(F("to min several times."));
  Display.setCursor(63, 136);
  Display.print(F("Min"));
  Display.setCursor(221, 136);
  Display.print(F("Max"));

  Display.fillRoundRect(68, 180, 192, 18, 2, C_DKGREEN);

  Display.setFont(FONT_14);
  Display.setTextColor(C_WHITE);
  ffDriverTime.setTextColor(C_WHITE, C_BLACK);
  ffDriverLapTime.setTextColor(C_WHITE, C_BLACK);

  GPSLEDTimer = 0;

  while (1) {
    WatchDogTimer(RESET_WDT);
    delay(10);
    ThrottleInputBits = ThrottleInputBits + EXTADC.analogRead(EXTADC_THROTTLE_PIN);
    AvgCounter++;

    if (GPSLEDTimer > 100) {

      ThrottleInputBits = ThrottleInputBits / AvgCounter;
      ThrottleInputVolts = (float)ThrottleInputBits / (EXADC_BIT_CONVERSION / EXADC_VREF);
      if (ThrottleInputVolts < MinVal) {
        MinVal = ThrottleInputVolts;
      }

      if (ThrottleInputVolts > MaxVal) {
        MaxVal = ThrottleInputVolts;
      }

      Display.setCursor(63 + 40, 158);
      ffDriverTime.print(MinVal, 2);

      Display.setCursor(221 + 40, 160);
      ffDriverLapTime.print(MaxVal, 2);
      StatusCounter++;

      if (StatusCounter > 100) {
        break;
      }

      StatusBarWidth = ((float)(StatusCounter * 192.0f) / 100.0f) + 2;
      Display.fillRoundRect(68, 180, StatusBarWidth, 18, 2, C_GREEN);
      ThrottleInputBits = 0;
      GPSLEDTimer = 0;
      AvgCounter = 0;
    }
  }

  ThrottleMinRange = MinVal;
  ThrottleMaxRange = MaxVal;

  EEPROM.put(75, ThrottleMinRange);
  EEPROM.put(80, ThrottleMaxRange);

  // now alert the user to double check the calibration on the ESC
  Display.fillRect(33, 21, 265, 208, C_WHITE);
  Display.fillRect(38, 25, 258, 198, C_RED);
  Display.setFont(FONT_14);
  Display.setTextColor(C_WHITE);

  Display.setCursor(45, 35);
  Display.print(F("1. Put ESC in program mode"));
  Display.setCursor(45, 55);
  Display.print(F("2. Connect ESC to PC"));
  Display.setCursor(45, 75);
  Display.print(F("3. Run DEScribe software"));
  Display.setCursor(45, 95);
  Display.print(F("4. SyRen50, Analog Tab"));
  Display.setCursor(45, 115);
  Display.print(F("5. Input: Min / Ctr. / Max"));
  Display.setCursor(55, 135);
  sprintf(buf, "%01.2f / %01.2f / %01.2f", ThrottleMinRange + 0.15f, (ThrottleMinRange + ThrottleMaxRange) / 2.0f, ThrottleMaxRange - 0.1f);
  Display.print(buf);
  Display.setCursor(45, 155);
  Display.print(F("6. Calibration: Custom"));
  Display.setCursor(45, 175);
  Display.print(F("7. Set ESC OFF/ON in menu"));
  Display.setCursor(55, 198);
  Display.print(F("Press any button to exit."));

  while (1) {
    WatchDogTimer(RESET_WDT);
    delay(10);
    ButtonPressed = WhatButtonWasPressed();
    if ((ButtonPressed == L_BUTTON) || (ButtonPressed == R_BUTTON) || (ButtonPressed == C_BUTTON)) {
      break;
    }
  }
}

void CalibrateAccererometer() {
  uint16_t StatusBarWidth = 0;
  uint8_t Found = 0, oFound = 0;
  uint8_t Tries = 0;
  int accel_deadzone = 8;  //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
  int giro_deadzone = 1;
  int ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;
  int mean_ax = 0, mean_ay = 0, mean_az = 0, mean_gx = 0, mean_gy = 0, mean_gz = 0;

  ax_offset = -mean_ax / ASensorBits;
  ay_offset = -mean_ay / ASensorBits;
  az_offset = (16384 - mean_az) / ASensorBits;

  gx_offset = -mean_gx / ASensorBits;
  gy_offset = -mean_gy / ASensorBits;
  gz_offset = -mean_gz / ASensorBits;

  AccelSensor.setXAccelOffset(0);
  AccelSensor.setYAccelOffset(0);
  AccelSensor.setZAccelOffset(0);

  AccelSensor.setXGyroOffset(0);
  AccelSensor.setYGyroOffset(0);
  AccelSensor.setZGyroOffset(0);

  Display.setFont(FONT_16B);
  Display.fillRect(0, 160, 320, 100, C_DKGREY);

  Display.setTextColor(C_WHITE, C_DKGREY);
  Display.setCursor(13, 170);
  Display.print(F("Hold any button to exit."));
  Display.fillRoundRect(13, 195, 293, 36, 2, C_DKGREEN);

  WaitForRelease();

  while (1) {
    WatchDogTimer(RESET_WDT);
    Found = 0;

    AccelSensor.setXAccelOffset(ax_offset);
    AccelSensor.setYAccelOffset(ay_offset);
    AccelSensor.setZAccelOffset(az_offset);

    AccelSensor.setXGyroOffset(gx_offset);
    AccelSensor.setYGyroOffset(gy_offset);
    AccelSensor.setZGyroOffset(gz_offset);

    meansensors(&mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);

    GForceMenu.value[GForceMenuID2] = ax_offset;
    GForceMenu.value[GForceMenuID3] = ay_offset;
    GForceMenu.value[GForceMenuID4] = az_offset;
    GForceMenu.drawRow(GForceMenuID2);
    GForceMenu.drawRow(GForceMenuID3);
    GForceMenu.drawRow(GForceMenuID4);

    if (abs(mean_ax) <= accel_deadzone) Found++;
    else ax_offset = ax_offset - mean_ax / accel_deadzone;

    if (abs(mean_ay) <= accel_deadzone) Found++;
    else ay_offset = ay_offset - mean_ay / accel_deadzone;

    if (abs(ASensorBits - mean_az) <= accel_deadzone) Found++;
    else az_offset = az_offset + (ASensorBits - mean_az) / accel_deadzone;

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
    StatusBarWidth = (float)((Found / 6.0) * 293.0);
    Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

    ButtonPressed = WhatButtonWasPressed();
    if ((Found == 6) || (Tries >= 50) || (ButtonPressed == L_BUTTON) || (ButtonPressed == R_BUTTON)) {
      break;
    }
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
    WaitForRelease();

  } else {
    Display.fillRect(0, 160, 320, 30, C_DKGREY);
    Display.setTextColor(C_WHITE, C_DKGREY);
    Display.setCursor(13, 170);
    Display.print(F("Calibration complete."));
    Display.fillRoundRect(13, 195, 293, 36, 2, C_GREEN);
    AccelCalX = ax_offset;
    AccelCalY = ay_offset;
    AccelCalZ = az_offset;
    delay(1000);

    // save to eeprom
  }
}
void WaitForExit() {
  uint8_t WaitTime = 10;
  uint32_t Start = millis();
  Display.setFont(FONT_16B);
  Display.setTextColor(C_WHITE);
  Display.setCursor(40, 160);
  Display.print("PRESS ANY BUTTON");
  Display.setCursor(40, 180);
  Display.print("OR WAIT");
  Display.setCursor(160, 180);
  Display.fillRect(160, 180, 40, 30, C_RED);
  Display.print(WaitTime--);

  GPSLapTimer = 0;

  while (true) {

    WatchDogTimer(RESET_WDT);

    ButtonPressed = WhatButtonWasPressed();

    if ((ButtonPressed == L_BUTTON) || (ButtonPressed == C_BUTTON) || (ButtonPressed == R_BUTTON)) {
      break;
    }

    if (millis() - Start > 1000) {
      Start = millis();
      Display.setCursor(160, 180);
      Display.fillRect(160, 180, 40, 30, C_RED);
      Display.print(WaitTime--);
    }

    if (GPSLapTimer > 10000) {
      break;
    }

    delay(10);
  }

  WaitForRelease();
}

void SetCyborgSetpointLimits() {
  if (CyborgInput == CYBORG_CONTROL_AMPS) {
    CyborgMinRange = CYBORG_MIN_AMPS;
    CyborgMaxRange = CYBORG_MAX_AMPS;
  } else {
    CyborgMinRange = CYBORG_MIN_SPEED;
    CyborgMaxRange = CYBORG_MAX_SPEED;
  }
}



void DisplayErrors() {
  if (timeStatus() != timeSet) {
    Display.fillRect(26, 26, 268, 188, C_WHITE);
    Display.fillRect(30, 30, 260, 180, C_RED);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE);
    Display.setCursor(40, 40);
    Display.print("FATAL ERROR");
    Display.setFont(FONT_16B);
    Display.setCursor(40, 75);
    Display.print("Internal clock error");
    Display.setCursor(40, 100);
    Display.print("Replace the datalogger");
    Display.setCursor(40, 125);
    Display.print("CR-2032 battery.");
    WaitForExit();
  }

  if (SSD.getRecordLength() != SSD.getDatabaseRecordLength()) {
    Display.fillRect(26, 26, 268, 188, C_WHITE);
    Display.fillRect(30, 30, 260, 180, C_RED);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE);
    Display.setCursor(40, 40);
    Display.print("FATAL ERROR");
    Display.setFont(FONT_16B);
    Display.setCursor(40, 80);
    Display.print("DATABASE: ");
    Display.print(SSD.getDatabaseRecordLength());
    Display.print(F("/"));
    Display.print(SSD.getRecordLength());
    Display.setCursor(40, 110);
    Display.print("ERASE SSD CARD");
    WaitForExit();
  }

  if (!EXTADCStatus) {
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
    Display.print("ADC SENSOR");
    WaitForExit();
  }

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
  if (Warnings & THROTTLE_WARNING) {
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
    Display.print("THROTTLE");
    WaitForExit();
  }
  if (Warnings & ALTIMITER_FAIL) {
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
    Display.print("ALTIMITER");
    WaitForExit();
  }
  if (Warnings & SPEED_WARNING) {
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
    Display.print("AIR SPEED SENSOR");
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
    Display.print("SSD CHIP FULL");
    Display.setCursor(40, 110);
    Display.print("DOWNLOAD, ERASE");
    WaitForExit();
  }
}

float FloatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void AreWeInARace() {
  // need > 90 + 5 min for a potential red flag min between race1 start and race2 start to consider a new race
  // and test if current race could be restored
  // duration is in seconds
  RaceStatus = RACE_NOTSTARTED;
  Duration = ((hour() * 3600) + (minute() * 60) + second()) - ((RaceHour * 3600) + (RaceMinute * 60) + RaceSecond);
  if ((RaceMonth == month()) && (RaceDay == day()) && (Duration < (RACE_TIME_SECONDS + RACE_EXTENSION))) {
    RaceStatus = RACE_INPROGRESS;
  }
}

void HandleRaceStatus() {
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
    Display.print(F("Current Race"));

    ReturnCode = RestoreRaceData();

    Display.fillRect(STATUS_RESULT, 220, 320 - STATUS_RESULT, 25, C_BLACK);

    Display.setCursor(STATUS_RESULT, 200);

    if (ReturnCode == RR_ERROR) {
      Display.setTextColor(C_RED);
      Display.print(F("FAIL to restore"));
      delay(10);
      // consider this now a new race dataset
      Display.setFont(FONT_14);
      Display.setTextColor(C_CYAN);
      Display.setCursor(STATUS_RESULT, 200);
      Display.print(F("New Race"));

      RecordSETID++;

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
      hr = (int)(ReturnCode / 3600);
      mn = (int)(ReturnCode / 60);
      sc = (int)(ReturnCode % 60);
      sprintf(buf, "%02d:%02d:%02d", hr, mn, sc);
      // display the amount of time we had to restore
      Display.print(buf);
    }

    // reset restore type
    RestoreType = STATUS_OK;
  }

  if (RaceStatus == RACE_NOTSTARTED) {
    // either the race hasn't started yet, or an attempt to restore the race failed
    // either way, treat this like it's a new race and move on
    // reset GPS start just in case race did not end naturally
    Display.setFont(FONT_14);
    Display.setTextColor(C_CYAN);
    Display.setCursor(STATUS_RESULT, 200);
    Display.print(F("New Race"));
    ResetRaceDate();
    SaveStartGPS(false);

    if (SSDStatus) {
      Display.print(RecordSETID);
    } else {
      Display.setCursor(STATUS_RESULT, 220);
      Display.setTextColor(C_RED);
      Display.print(F("SSD FULL"));
      Warnings = Warnings | SSD_FAIL;
    }

#ifdef DO_DEBUG
    Serial.println("New Race.");
    Serial.print("RecordSETID: ");
    Serial.println(RecordSETID);
#endif
  }
}

void WaitForPress(uint8_t Button) {
  delay(100);
  if (Button == C_BUTTON) {
    while (!digitalRead(L_PIN) && !digitalRead(R_PIN)) {
      WatchDogTimer(RESET_WDT);
      delay(10);
    }
  } else if (Button == EITHER_BUTTON) {
    while ((!digitalRead(L_PIN) && digitalRead(R_PIN)) || (digitalRead(L_PIN) && !digitalRead(R_PIN))) {
      WatchDogTimer(RESET_WDT);
      delay(10);
    }
  }

  else if (Button == L_BUTTON) {
    while (digitalRead(L_PIN) && !digitalRead(R_PIN)) {
      WatchDogTimer(RESET_WDT);
      delay(10);
    }
  }

  else if (Button == R_BUTTON) {
    while (!digitalRead(L_PIN) && digitalRead(R_PIN)) {
      delay(10);
    }
  }
}

void WaitForRelease() {
  delay(10);
  while ((digitalRead(L_PIN) == LOW) || (digitalRead(R_PIN) == LOW)) {
    WatchDogTimer(RESET_WDT);
    delay(10);
  }
}

uint8_t WhatButtonWasPressed() {
  if (digitalRead(L_PIN) == LOW) {

    delay(1);
    if (digitalRead(R_PIN) == LOW) {
      uint32_t StartTime = millis();
      while (digitalRead(R_PIN) == LOW) {
        WatchDogTimer(RESET_WDT);
        if ((millis() - StartTime) > MENU_ENTER_TIMEOUT) {
          return C_BUTTON_LONG;
        }
      }

      return C_BUTTON;
    }
    return L_BUTTON;
  }
  if (digitalRead(R_PIN) == LOW) {
    delay(1);
    if (digitalRead(L_PIN) == LOW) {
      uint32_t StartTime = millis();
      while (digitalRead(L_PIN) == LOW) {
        WatchDogTimer(RESET_WDT);
        if ((millis() - StartTime) > MENU_ENTER_TIMEOUT) {
          return C_BUTTON_LONG;
        }
      }

      return C_BUTTON;
    }
    return R_BUTTON;
  }

  return NO_BUTTON;
}

void ButtonPress() {
  if (DisplayID > MaxDisplayIDs) {
    DisplayID = 0;
  }

  //main menu trigger
  if (ButtonPressed == L_BUTTON) {

    DisplayID++;
    DriverChangeScreen = 4000;
    if (DisplayID > MaxDisplayIDs) {
      DisplayID = 0;
    }

    EEPROM.put(320, DisplayID);
    delay(50);
  } else if (ButtonPressed == R_BUTTON) {

    if (DisplayID == 0) {
      DisplayID = MaxDisplayIDs;
    } else {
      DisplayID--;
    }

    DriverChangeScreen = 4000;
    EEPROM.put(320, DisplayID);
    delay(50);
  } else if (ButtonPressed == C_BUTTON_LONG) {
    ProcessMainMenu();
  }

  RestartDisplay();
  OldDisplayID = DisplayID;
}

uint16_t GetPlotColor(float val, float MinTemp, float MaxTemp) {
  uint8_t red = 0, green = 0, blue = 0;
  float a = MinTemp + (MaxTemp - MinTemp) * 0.2;   // .21
  float b = MinTemp + (MaxTemp - MinTemp) * 0.35;  //.32
  float c = MinTemp + (MaxTemp - MinTemp) * 0.5;   // .43
  float d = MinTemp + (MaxTemp - MinTemp) * 0.8;   //.82

  red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255);

  if ((val > MinTemp) & (val < a)) {
    green = constrain(255.0 / (a - MinTemp) * val - (255.0 * MinTemp) / (a - MinTemp), 0, 255);
  } else if ((val >= a) & (val <= c)) {
    green = 255;
  } else if (val > c) {
    green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
  } else if ((val > d) | (val < a)) {
    green = 0;
  }

  if (val <= b) {
    blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
  } else if ((val > b) & (val <= d)) {
    blue = 0;
  } else if (val > d) {
    blue = constrain(240.0 / (MaxTemp - d) * val - (d * 240.0) / (MaxTemp - d), 0, 240);
  }

  // use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
  return Display.color565(red, green, blue);
}
void WatchDogTimer(uint8_t state) {

#if defined(__MK20DX256__)  // Teensy 3.2
  if (state == ENABLE_WDT) {
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

#elif defined(__IMXRT1062__)  // Teensy 4.0 or 4.1
  if (state == ENABLE_WDT) {
    WDTConfig.window = 5000;   /* in seconds, 0->128 */
    WDTConfig.timeout = 10000; /* in seconds, 0->128 */
    WDTConfig.callback = WDTCallback;
    WDT.begin(WDTConfig);

  } else if (state == DISABLE_WDT) {
    WDTConfig.window = 120000;
    WDTConfig.timeout = 500000;
    WDTConfig.callback = WDTCallback;
    WDT.begin(WDTConfig);

  } else {
    WDT.feed();
  }
#endif
}

#if defined(__IMXRT1062__)
void WDTCallback() {
  WDT.reset();
  WDT.feed();
}
#endif

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
  - moved deep functions into main setup() and  loop() codes to relieve stress on stack
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
  - Add menu option under car menu to change race start amp draw trigger Threshold.
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
  
  B9_v3.40   Kris   10/24/2024
  hard coding some options (min pulses) added input for tire pressure

  B9_v5.2    Goins 6/3/2025
  reworked display code got the bar to work

  B10_v1.2    Kasprzak 6/3/2025
  reworked to a new PCB board, external ADC, suppoort for T4.0 or T3.2, more USB inputs, 
  changed barametric pressure sensor, scaled back encryptor to send less data
  
  B10.32.050    Kasprzak 12/3/2025
  added energy prediction calculator, reworked encryptor and database schema

  B10.64.001    Goins 5/25/2026
  added math to compute AME

  10.30.02.03    Kasprzak 8/26/2026
  removed AME--nice idea but not accurate enough and driver control over limit does a good job
  changed cyborg to still control amps but throttle now controls setpoint--that way no jumping at the threashold crossover



*/



/*
  END OF DATALOGGER CODE
  CODED BY:
  Kris Kasprzak, Jacob H., Ben Runyan, JOSHUA C., YASHAS G., THOMAS T, Delmont G.
*/
