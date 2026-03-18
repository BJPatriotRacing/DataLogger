/*
  ---------------------------------------------------------
  PROGRAM INFORMATION
  ---------------------------------------------------------
  Bob Jones Patriot Racing Car Datalogger
  Copyright 2016-2025, All Rights reserved
  This code is property of Patriot Racing and Kris Kasprzak
  This code cannot be used outside of Bob Jones High School
  Code for Teensy 3.2
  ---------------------------------------------------------
  COMPILE INSTRUCTIONS
  ---------------------------------------------------------
  Compile Speed:  72 MHz       
  Optimize:       smallest code

  LIBRARY CHANGES
  1) in ILI9341_Menu.h has #define MAX_OPT 16 (or max number of menu items)
  adding fields is fine, but you may need to adjust the limit the library

  // in BulletDB.h change 
  1) the limit as needed #define MAX_FIELDS 25 (change to max files in BuildFieldList--the 
  lib will not automatically accomodate larger arrays)
  2) the WRITE_SPEED should be 20 000 000 (25 000 000 is too fast for SaveRecordFast)

  // in ILI9341_t3_Controls.h change 
  1) #define MAX_GRAPHS 12 (change to max files in BuildFieldList--the lib will not automatically 
  accomodate larger arrays)

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

#include <avr/io.h>                    // standard library that ships with Teensy
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
#include <EBYTE.h>                     // transceiver lib   https://github.com/KrisKasprzak/EBYTE
#include <ILI9341_FlickerFreePrint.h>  // eliminates text drawing flicker https://github.com/KrisKasprzak/FlickerFreePrint/blob/master/FlickerFreePrint.h
#include <Arial_100_BINO.h>            // special font file that is numbers only. stored in PatriotRacing_Fonts, recreate use https://spooksoft.pl/download/, https://spooksoft.pl/en/download-3/
#include <Arial_48BINO.h>              // special font file that is numbers only. stored in PatriotRacing_Fonts, recreate use https://spooksoft.pl/download/, https://spooksoft.pl/en/download-3/
#include <font_ArialBold.h>            // comes with display or go here https://github.com/PaulStoffregen/ILI9341_fonts
#include <font_Arial.h>                // comes with display or go here https://github.com/PaulStoffregen/ILI9341_fonts
#include <font_ArialBoldItalic.h>      // https://github.com/PaulStoffregen/ILI9341_fonts
#include <BulletDB.h>                  // flash chip database driver https://github.com/KrisKasprzak/BulletDB
#include <Wire.h>
#include "MPU6050.h"           // accelerometer lib  https://github.com/ElectronicCats/mpu6050
#include <FreqMeasureMulti.h>  // lib for speed sensor https://github.com/PaulStoffregen/FreqMeasure
#include <PID_v1.h>            // https://github.com/mblythe86/C-PID-Library/tree/master/PID_v1
#include <avr/io.h>
#include <avr/interrupt.h>
#include "MCP3208.h"    // https://github.com/KrisKasprzak/MCP3208
#include "MS5837.h"     // https://github.com/bluerobotics/BlueRobotics_MS5837_Library
#include <XGZP6897D.h>  // https://github.com/fanfanlatulipe26/XGZP6897D

// #define DO_DEBUG


/*-------------------*/
// Code Version
/*-------------------*/
// board.MCU.version.build
#define CODE_VERSION "10.32.061.24"

/*-------------------*/
//Constant Definitions
/*-------------------*/

// see PatriotRacing_Utilities.h for many defines

#define PREDICT_SAMPLES 20

//Global Fonts and Locations
#define FONT_100BINO Arial_100_BINO     // font for the large data
#define FONT_24BI Arial_24_Bold_Italic  // font for the small data
#define FONT_16B Arial_16_Bold          // font for all headings
#define FONT_14 Arial_14                // font for menus
#define FONT_48BINO Arial_48BINO

#define MENU_ENTER_TIMEOUT 3000
#define MENU_EXIT_TIMEOUT 20000

// we had an issue where batteries were dropped very low and caused error key stat reads
// this setting allows us to set the key voltage limit
// inside the current sensor the battery voltage goes through voltage divider (1/11, so
// the math sets the key on voltage to be around 3.5 volts
#define KEY_ON_LIMIT 400

#define PITMESSAGEX 110
#define PITTIMEX 215
#define PEAK_TIMER 4000

// manual limits for PID values as entered by the user in the menu system
#define MAX_KP 100
#define MAX_KI 2000
#define MAX_KD 100

/*-------------------*/
// Program Variables
/*-------------------*/

uint16_t DATA_X = 15;
uint8_t DataJustificationRight = 0;
bool DrawGraph = true;
uint16_t EnergyPoints[100];
uint16_t BLEnergy[16] = { 0, 43, 86, 128, 171, 212, 254, 295, 335, 375, 414, 453, 491, 528, 564, 600 };
uint32_t EnergyXPoint = 0;
int8_t EnergyID = 0, bEnergyID = 0;
int8_t GraphVoltsID = 0, GraphAmpsID = 0, GraphLapAmpsID = 0, GraphSpeedID = 0, GraphMTempID = 0, GraphAltitudeID = 0;
uint8_t GraphCyborgInID = 0, GraphCyborgOutID = 0, GraphPredictedID = 0;
uint8_t Button = 0;
uint8_t ButtonPressed = NO_BUTTON;
uint8_t MaxDisplayID = 7;
float ASTemp = 0.0f;
float ASPressure = 0.0f;
float fpm = 0.0f;
float AirSpeed = 0.0f;
float AirSpeedOffset = 0.0f;

// AutonomousEnergyManagement variables
float PredictionCompensation = 86.0f;
// variables for energy prediction
float CurrentBias = 0.0f, CurrentB = 0.0f;
float PredictedEnergy = 0.0, RawPredictedEnergy = 0.0;
float LimitTolerance = 0.0f;
double m_PE = 0.0, b_PE = 0.0;
float EnergyArray[PREDICT_SAMPLES];
float PointArray[PREDICT_SAMPLES];
uint16_t PredictedEnergyCounter = 0;
float PredictedEnergyAverage = 0.0f;
uint8_t PredictionArrayCounter = 0;
uint16_t iCP = 0;
double sum_y = 0.0f;
double sum_xx = 0;
double sum_x = 0;
double sum_xy = 0;
double den = 0.0f;
double c1 = 0;
double c2 = 0;
double c3 = 0;
double c4 = 0;
uint16_t EPointer, oEPointer;
// sensors
bool RPMStatus = false;
bool RadioStatus = false;
bool GForceStatus = false;
bool AltimiterStatus = false;
bool SDCardStatus = false;
bool GPSStatus = false;
bool SSDStatus = false;
bool EXTADCStatus = false;
bool RedrawDisplay = false;
bool RedrawHeader = false;
bool DriverTimeOK = false;

// data for chip
uint8_t RecordSETID = 0;
uint8_t RecordType = RT_HEADER;
uint16_t Point = 0;  // Counter for the data Point
uint8_t LapCount = 0;
uint8_t Driver = 0;  // variable for current driver 0-2
uint8_t CyborgInSignal = 0;
uint8_t CyborgOutSignal = 0;
float mRPM = 0.0f;
float Volts = 0.0f;
float Power = 0.0f;
float AmbTemp = 0.0f;
float MotorTemp = 0.0f;
float AuxTemp = 0.0f;
float Energy = 0.0f;
float Amps = 0.0f;
float CarSpeed = 0.0f;
float Distance = 0.0f;
float Altitude = 0.0f, StartAltitude = 0.0f;
int16_t AltCorrection, GPSAltCorrection;
float GPSStartLat = 0.0f, GPSStartLon = 0.0f, GPSSpeed = 0.0f;
float GForceX = 0.0f, GForceY = 0.0f, OldGForceY = 0.0f, GForceZ = 0.0f, GForce = 0.0f;
uint8_t RestoreType = STATUS_OK;
uint8_t PeakGValue = 0;
bool EnableAirFlowSensor = false;
bool AirFlowSensorFound = false;
// cyborg variables
uint16_t ThrottleInputBits = 0;
float ThrottleInputVolts = 0.0f;
bool EnableCyborg = false;
uint8_t AveragingUpdateTime = 1;
uint16_t CyborgOutputPWM = 0;
uint32_t CyborgCounter = 0;
uint32_t CyborgAmpBits = 0;
float CyborgAmpVolts = 0.0f;
float CyborgAmps = 0.0f;
float CyborgFirstLimit = 19.0f;
float CyborgSecondLimit = 25.0;
uint16_t CyborgUpdateTime = 100;
bool CyborgTurbo = false;
bool CyborgActive = false;
float CyborgThresholdVolts = 3.1f;
float OldCyborgFirstLimit = 0.0f;
float OldCyborgSecondLimit = 0.0f;
int32_t PitTime = 0;
int32_t MinDriveTime = 0;  // ms
int32_t MaxDriveTime = 0;
double Setpoint = 0.0, Input = 0.0, Output = 0.0;
double Kp = 0.5f, Ki = 155.0f, Kd = 0.1f;
int CyborgXPoint = 0;
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
unsigned long RadioUpdate = 1;  // index to char array
uint16_t CarID = 0;             // 0 = red, 1 = blue, 3 = white
uint8_t MotorSprocket = 15;     // number of teeth for motor sprocket
uint8_t WheelSprocket = 70;     // number of teeth for driven sprocket
float GearRatio = 4.66666f;     // large, small sprocket teeth
uint8_t TireID = 0;             // tire id for tiretext and tirerad arrays
uint8_t MotorID = 0;            // motor id for tiretext and tirerad arrays
uint8_t TirePressure = 115;     // input for tirepressure
float TireRad = 9.0f;           // will get set later by tire ID
uint16_t TotalEnergy = 600;     // default energy value, sum of both batteries at 10.5 volt mark
uint8_t Pickups = 4;            // default pickups
uint8_t Theme = 0;              // track white / black display background
uint8_t Orientation = 0;        // track display orientation
uint8_t GPSTolerance = 4;       // default to 8 meter for GPS tolerance
volatile uint32_t SpeedTimer = 500;
bool RPBDrawGraphs = false;
bool RPBRaceLines = false;
bool RPBPlotVolts = false;
bool RPBPlotAmps = false;
bool RPBPlotLapAmps = false;
bool RPBPlotSpeed = false;
bool RPBPlotMTemp = false;
bool RPBPlotAltitude = false;
bool RPBCyborgIn = false;
bool RPBCyborgOut = false;
bool RPBPlotMPEnergy = false;

bool AutoCurrentCal = true;
uint8_t Battery1 = 0;
uint8_t Battery2 = 0;

// note the ACS-770 U200 curent sensor has sensitivity of 20 mV/Amp, 0.5 volts offset at 0 amps,
// unit is powered with 5.0 Vcc
float VMid = 0.5f;             // offset for current sensor, right from data sheet (note we have an option to zero this out on startup)
float mVPerAmp = 20.0f;        // sensitivity for current sensor, right from data sheet
float VoltageSlope = 11.0f;    // Vin slope from the voltage divider.
float VoltageOffset = 0.299f;  // Vin is comming through diodes need to compensate that loss.
float BatWarning = 21.0f;      // default voltage for batter warning
float TempWarning = 140.0f;    // default temp for motor warning
float ThermResMotor = 10000;   // voltage divider resistor for external thermistor sensor
float ThermResAux = 10000;     // voltage divider resistor for external thermistor sensor
uint8_t LapThreshold = 30;     // time in seconds required to elapse before another lap is allowed to be counted
uint8_t PacketSize = 0;        // transceiver packet size
float ASensorBits = 4096.0f;
int16_t AccelCalX = 0;
int16_t AccelCalY = 0;
int16_t AccelCalZ = 0;
uint32_t Duration = 0;
uint8_t ASensorDirection = 0;
float ax = 0.0, ay = 0.0, az = 0.0;
int16_t gx = 0.0, gy = 0.0, gz = 0.0;
uint32_t DelayAmount = 0;
int16_t ByteWidth = 0;
uint8_t sByte = 0, b = 0;
char buf[50];  // generic buffer for various uses

uint8_t GraphPointX = 0;

//Time Variables (RTCTime is a time_t object; others are used to set RTCTime)
int16_t hours = 0, minutes = 0, seconds = 0, days = 0, months = 0, years = 0;
int16_t RaceDay = 0, RaceHour = 0, RaceMinute = 0, RaceSecond = 0, RaceMonth = 0;
uint32_t ExitStartTime = 0;
bool EnableAutoExit = true;

// accelerometer calibration variables
int ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;
int mean_ax = 0, mean_ay = 0, mean_az = 0, mean_gx = 0, mean_gy = 0, mean_gz = 0, state = 0;
int buffersize = 500;    //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int accel_deadzone = 8;  //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;
float oldGForceZ = 0.0f, oldGForceY = 0.0f;
uint8_t RaceStatus = RACE_NOTSTARTED;
uint8_t KeyState = LOW, OldKeyState = LOW, KeyStatus = KEY_NO_CHANGE;
uint8_t MaxG = 0;
float LapSpeed = 0.0f;
float MaxLapGForceY = 0.0f;
float GForceYLapMaxL, GForceYLapMaxR, TempGForceYLapMaxL, TempGForceYLapMaxR;

// Speed and Distance Variables
volatile uint32_t RPMSum = 0;
volatile uint32_t RPMCount = 0;
volatile uint16_t RPMMinCount = 0;
float Revolutions = 0.0;  // counts total to compute Distance
uint32_t Counter = 0;     // the number of measurements between each display
uint32_t AverageCounter = 0;

//Driver Variables
uint8_t DriverLaps[3] = { 0, 0, 0 };  // Array for number of laps per driver
int32_t DriverTime[3] = { 0, 0, 0 };  // Array for driver time

//Transceivers Variables
uint16_t AirDataRate = 0, oAirDataRate = 0;
uint16_t RadioPower = 0, oRadioPower = 0;
uint16_t RadioChannel = 0, oRadioChannel = 0;

//Car Variables
float WRPM = 0.0f;  // wheel rpm (measured)
float vVolts = 0.0f;
float thmVolts = 0.0f, thxVolts = 0.0f, TempK = 0.0f;
float CyborgFL = 0.0f, CyborgSL = 0.0f;
float AmbTempCF = 0.0f, LapAmps = 0.0;
float ERem = 100.0f, TRem = 100.0f;  // Energy and Time remaining
float aVolts = 0.0f;                 // computed Amps
float ThermistorResistence = 0.0f;   // computed thermistor resistance
int16_t hr = 0, mn = 0, sc = 0;      // for formatting min and sec
uint32_t StatusBarCounter = 0;
float AtmPressure = 0.0f;

//Buttons
uint8_t DisplayID = 0;  // right button tracker
uint8_t ResetEBYTE = 0;
uint8_t L_PIN = LEFT_PIN;   // pin for the up display mode button (uint8_ts to address display upside down)
uint8_t R_PIN = RIGHT_PIN;  // pin for the down button

//Variables for Average Calculations
uint32_t AverageCount = 0;
float LapVolts = 0.0f, AverageAmps = 0.0f, MaxLapAmps = 19.5f, TargetAmps = 0.0f, TempTargetAmps = 0.0f;
float TriggerAmps = 30.0f;  // Minimum amp required to trigger a race start or dirver change
float AverageVolts = 0.0f, LapEnergy = 0.0f, StartLapEnergy = 0.0f, AverageCarSpeed = 0.0f;
uint8_t StartGPSDelayID = 0;
uint32_t StartGPSDelay = 0;

// GPS and Channel Variables
float GPSLat = 0.0, GPSLon = 0.0, GPSDistance = 0.0;
uint16_t LapTime = 0, LastLapTime = 0, PlotWide = 0, PlotHigh = 0;
int16_t TimeDelta = 0;
uint16_t GPSSatellites = 0;
uint16_t fore_color = 0, back_color = 0, BarColor = 0, PeakColor = 0;  // remember the foreground and background colors
uint16_t i = 0, j = 0;                                                 // just some storage variables

//Warning Global Variable
uint16_t Warnings = 0;

// menu ID variables
uint8_t MainMenuOption = 0, MainMenuOption1 = 0, MainMenuOption2 = 0, MainMenuOption3 = 0;
uint8_t MainMenuOption4 = 0, MainMenuOption5 = 0, MainMenuOption6 = 0, MainMenuOption7 = 0;
uint8_t MainMenuOption8 = 0, MainMenuOption9 = 0;
uint8_t MenuOption = 0;
uint8_t RaceMenuOption1 = 0, RaceMenuOption2 = 0, RaceMenuOption3 = 0, RaceMenuOption4 = 0;
uint8_t RaceMenuOption5 = 0, RaceMenuOption6 = 0, RaceMenuOption7 = 0, RaceMenuOption8 = 0;
uint8_t RaceMenuOption9 = 0, RaceMenuOption10 = 0, RaceMenuOption11, RaceMenuOption12, RaceMenuOption13;
uint8_t RaceMenuOption14 = 0;
uint8_t SettingsMenuOption1 = 0, SettingsMenuOption2 = 0, SettingsMenuOption3 = 0, SettingsMenuOption4 = 0;
uint8_t SettingsMenuOption5 = 0, SettingsMenuOption6 = 0, SettingsMenuOption7 = 0;
uint8_t WirelessMenuOption1 = 0, WirelessMenuOption2 = 0, WirelessMenuOption3 = 0, WirelessMenuOption4 = 0;
uint8_t WirelessMenuOption5 = 0, WirelessMenuOption6 = 0, WirelessMenuOption7 = 0;
uint8_t SensorMenuOption1 = 0, SensorMenuOption2 = 0, SensorMenuOption3 = 0, SensorMenuOption4 = 0;
uint8_t SensorMenuOption5 = 0, SensorMenuOption6 = 0, SensorMenuOption7 = 0, SensorMenuOption8 = 0;
uint8_t SensorMenuOption9 = 0, SensorMenuOption10 = 0, SensorMenuOption11 = 0, SensorMenuOption12 = 0;
uint8_t GForceMenuOption1 = 0, GForceMenuOption2 = 0, GForceMenuOption3 = 0, GForceMenuOption4 = 0;
uint8_t GForceMenuOption5 = 0, GForceMenuOption6 = 0, GForceMenuOption7 = 0, GForceMenuOption8 = 0;
uint8_t ClockMenuOption1 = 0, ClockMenuOption2 = 0, ClockMenuOption3 = 0, ClockMenuOption4 = 0;
uint8_t ClockMenuOption5 = 0, ClockMenuOption6 = 0;
uint8_t CyborgMenuOption1 = 0, CyborgMenuOption2 = 0, CyborgMenuOption3 = 0, CyborgMenuOption4 = 0;
uint8_t CyborgMenuOption5 = 0, CyborgMenuOption6 = 0, CyborgMenuOption7 = 0, CyborgMenuOption8 = 0;
uint8_t CyborgMenuOption9 = 0, CyborgMenuOption10 = 0, CyborgMenuOption11 = 0;
uint8_t SSDMenuOption1 = 0, SSDMenuOption3 = 0, SSDMenuOption4 = 0, SSDMenuOption5 = 0, SSDMenuOption6 = 0;
uint8_t PlayBackOption1 = 0, PlayBackOption2 = 0, PlayBackOption3 = 0, PlayBackOption4 = 0, PlayBackOption5 = 0;
uint8_t PlayBackOption6 = 0, PlayBackOption7 = 0, PlayBackOption8 = 0, PlayBackOption9 = 0, PlayBackOption10 = 0;
uint8_t PlayBackOption11 = 0;

// field ID variables
uint8_t frType = 0, frID = 0, frPoint = 0, frLap = 0, frDriver = 0, frVolts = 0, frAmps = 0, frMotorTemp = 0, frAuxTemp = 0;
uint8_t frEnergy = 0, frRPM = 0, frWRPM = 0;
uint8_t frSpeed = 0, frDist = 0, frRT = 0, frLon = 0, frLat = 0, frAltitude = 0, frGPSpeed = 0, frRestoreType = 0, frMax = 0, frMay = 0, frMaz = 0;
uint8_t frAmbTemp = 0, frCyborgInSignal = 0, frCyborgOutSignal = 0, frAirSpeed = 0, frAirOffset = 0;  // frKp = 0, frKi = 0, frKd = 0;
uint8_t frPredictedEnergy = 0, frCyborgFirstLimit = 0;

// header ID variables
uint8_t hrType = 0, hrID = 0, hrYear = 0, hrMonth = 0, hrDay = 0, hrHour = 0, hrMinute = 0, hrMSprocket = 0, hrWSprocket = 0;
uint8_t hrStartAltitude = 0, hrStartPressure = 0, hrTireID = 0, hrTirePressureID = 0;
uint8_t hrD0ID = 0, hrD1ID = 0, hrD2ID = 0, hrCarID = 0, hrMotorID = 0, hrTemp = 0, hrAmbTemp = 0;
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
TinyGPSPlus GPS;

// sd card
SdFat SDCARD;
SdFile SDDataFile;
SdFile SDSetupFile;

// structure for the wireless data tansmission
Transceiver Data;

// wireless device
EBYTE Radio(&ESerial, M0_PIN, M1_PIN, AX_PIN);  //Transceiver object

// special lib to address data packing for wireless
EasyTransfer DataPacket;

// flicker free objects for data display
ILI9341_FlickerFreePrint ffMainData(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffLapVolts(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffLapSpeed(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffEnergy(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffLapAmps(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffLapEnergy(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffLaps(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffDriverLapTime(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffDriverTime(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffTime(&Display, C_WHITE, C_BLACK, JUSTIFY_LEFT);
ILI9341_FlickerFreePrint ffACalX(&Display, C_WHITE, C_BLACK, JUSTIFY_LEFT);
ILI9341_FlickerFreePrint ffACalY(&Display, C_WHITE, C_BLACK, JUSTIFY_LEFT);
ILI9341_FlickerFreePrint ffACalZ(&Display, C_WHITE, C_BLACK, JUSTIFY_LEFT);
ILI9341_FlickerFreePrint ffCyborgFirstLimit(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffCyborgSecondLimit(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffGForceY(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffGForceYLapMaxL(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffGForceYLapMaxR(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);
ILI9341_FlickerFreePrint ffPitMessage(&Display, C_WHITE, C_BLACK, JUSTIFY_LEFT);
ILI9341_FlickerFreePrint ffPitTime(&Display, C_WHITE, C_BLACK, JUSTIFY_LEFT);
ILI9341_FlickerFreePrint ffPredict(&Display, C_WHITE, C_BLACK, JUSTIFY_RIGHT);

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

time_t RTCTime;

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
elapsedMillis PeakTimer = 0;
elapsedMillis GPSLEDTimer = 0;
elapsedMillis AveragingUpdateTimer = 0;

// accelerometer
MPU6050 ASensor;

// object for speed sensor
FreqMeasureMulti RPM;

// flash chip data driver
BulletDB SSD(SSD_PIN);

// object for external ADC chip
MCP3208 EXTADC(EXTADC_CS_PIN);  // my chip select pin

// object for dedicated altimiter (not GPS altimiter)
MS5837 Altimiter;

// air flow sensor
XGZP6897D ASSensor(FLOW_SENSOR_K);

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
  pinMode(OUTPUT_PIN, OUTPUT);

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
  analogWriteFrequency(OUTPUT_PIN, 10000);

  analogWrite(OUTPUT_PIN, 0);

  StartDisplay();

  Display.fillScreen(C_BLACK);

  // init the SSD chip
  SSDStatus = SSD.init();

  analogReadRes(12);
  analogReadAveraging(1);

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

  Display.setFont(FONT_24BI);
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
  SSD.dumpBytes(0, 500);
#endif

  if (SSD.getUsedSpace() > 7000000000) {  // 8 mb chip and race typically needs 970K
    Warnings = Warnings | SSD_FAIL;
  }

  AreWeInARace();

  //Get the structure size for the transceiver
  PacketSize = sizeof(Data);

  //Configure up/down
  ConfigureButtons();

  InitializeSensors();

  HandleRaceStatus();

  // display big giant errors
  if (RaceStatus == RACE_NOTSTARTED) {
    DisplayErrors();
    delay(500);
  }

  Display.fillScreen(back_color);  //Once done with setup, transition to showing stats on car screen

  // WatchDogTimer(ENABLE_WDT);

  AveragingUpdateTimer = 0;
  TempGForceYLapMaxL = 0.0;
  TempGForceYLapMaxR = 0.0;
  GraphDrawTimer = 0;
  GraphStoreTimer = 0;
  DisplayUpdateTimer = 0;
  RadioUpdateTimer = 0;
  GPSLapTimer = 0;
  GraphDrawTimer = 60000;
  GPSMaxReadTimer = 0;
  SpeedUpdateTimer = 0;
  CyborgTimer = 0;
  RedrawHeader = true;
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
  // delay(5);
  ButtonPressed = WhatButtonWasPressed();

  if ((ButtonPressed == L_BUTTON) || (ButtonPressed == C_BUTTON_LONG) || (ButtonPressed == R_BUTTON)) {
    ButtonPress();
    WaitForRelease();
  }

  //Measure volts, amps, and temperature
  vVolts = vVolts + EXTADC.analogRead(EXTADC_VM_PIN);
  aVolts = aVolts + EXTADC.analogRead(EXTADC_AM_PIN);
  thmVolts = thmVolts + EXTADC.analogRead(EXTADC_THM_PIN);
  thxVolts = thxVolts + EXTADC.analogRead(EXTADC_THX_PIN);

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // CYBORG, this section updates pid on each read
  // if this works better, roll it into the next section
  if (EnableCyborg) {

    ThrottleInputBits = EXTADC.analogRead(EXTADC_THROTTLE_PIN);
    CyborgAmpBits = EXTADC.analogRead(EXTADC_AM_PIN);

    // get input volts
    ThrottleInputVolts = (float)ThrottleInputBits / (BIT_CONVERSION / REFERENCE_VOLTAGE);

    // get current draw
    CyborgAmpVolts = CyborgAmpBits / (BIT_CONVERSION / REFERENCE_VOLTAGE);

    CyborgAmps = ((CyborgAmpVolts - VMid) * 1000.0f) / mVPerAmp;

    // pass in  Input - which is the meaured current
    // pass in  Setpoint is either first limit or second limit determined below
    // pass out Output which is the PWM signal

    Input = CyborgAmps;

    // fyi the PID object hangs on to the input, output and setpoint
    // PID CyborgPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
    CyborgPID.Compute();
    // now we have output
    // we obly allow cyborg to work above a certian throttle value, otherwise reducing throttle
    // will not slow car since cyborg is managing esc input to maintain x amps
    if (ThrottleInputVolts > CyborgThresholdVolts) {
      CyborgActive = true;
      CyborgOutputPWM = Output;
    } else {
      CyborgActive = false;
      CyborgOutputPWM = ThrottleInputBits;
    }
    // bounds check
    if (CyborgOutputPWM > BIT_CONVERSION) {
      CyborgOutputPWM = BIT_CONVERSION;
    }

    // send a PWM signal to the ESC, could be direct from the throttle, or a cyborg calculation
    analogWrite(OUTPUT_PIN, CyborgOutputPWM);
  }

  // CYBORG, this section gets limits and PID tuning values
  // not sure what we need a second if maybe delete and have one big if????
  if (EnableCyborg) {

    CyborgFL = CyborgFL + EXTADC.analogRead(EXTADC_CYBORGFIRSTLIMIT_PIN);
    CurrentB = CurrentB + EXTADC.analogRead(EXTADC_CYBORGCURRENTBIAS_PIN);

    CyborgCounter++;
    CyborgTurbo = digitalRead(TURBO_PIN);

    if (CyborgTimer > CyborgUpdateTime) {

      CyborgFL = CyborgFL / CyborgCounter;
      CyborgFL = CyborgFL / (EXADC_BIT_CONVERSION / EXADC_VREF);
      CurrentB = CurrentB / CyborgCounter;
      CurrentB = CurrentB / (EXADC_BIT_CONVERSION / EXADC_VREF);

      if (CurrentB < 0.03f) {
        // process any changes to the first/second limits
        CyborgFirstLimit = FloatMap(CyborgFL, 0.0, 3.3, 14.0, 22.0);
        CyborgFirstLimit = (uint16_t)(CyborgFirstLimit * 10.0f) / 10.0f;
      } else {
        CurrentBias = FloatMap(CurrentB, 0.0, 3.3, 0.0, 0.2);
      }

      CurrentBias = uint8_t(CurrentBias * 100.0f) / 100.0f;

      Setpoint = CyborgFirstLimit;
      if (!CyborgTurbo) {
        // INPUT_PULLUP so off is high, on is low
        CyborgActive = false;
        Setpoint = CyborgSecondLimit;
      }

      // compute the % duty cycle for saving to the SSD
      CyborgInSignal = (ThrottleInputVolts * 100.0) / REFERENCE_VOLTAGE;

      // here's the deal... throttle output range is approx 1-4 VDC, MCU can accept up to 3v3 and output 3v3
      // using an onboard voltage divider we're throttling back max throttle to 3.25 or so
      // not a full 3.3 in case of tolerances in the throttle, temperature, etc.
      // that means the 1.0 get's throttled back as well
      // we're not sure how much we have throttled back the throttle but according to the math
      // 3.25 / 4.0 = 0.8125 so low become 1.0 x 0.82 so input signal is now 0.82 to 3.25
      // CyborgOutSignal is in percent
      // CyborgOutputPWM is bit depth
      // high output will be BIT_CONVERSION but low will be
      // out signal is a percentage and really only used for display, sending to pit and saving to SSD
      CyborgOutSignal = (CyborgOutputPWM * 100.0) / BIT_CONVERSION;
      // because min throttle is 0.8 or so we need to scale from that below are percentages
      // but we allow bottom to be 0.6 volts or 18.2%
      CyborgOutSignal = FloatMap(CyborgOutSignal, 18.2f, 100.0f, 0.0f, 100.0f);

      // zero out counter variables
      CyborgTimer = 0;
      CyborgCounter = 0;
      ThrottleInputBits = 0;
      ThrottleInputVolts = 0;
      CyborgAmpBits = 0;
      CyborgFL = 0.0f;
      CurrentB = 0.0f;
      CyborgSL = 0.0f;
    }
  }
  // end CYBORG calculations

  if (GPSTolerance != 0) {
    GPSRead();
  }

  if (RPM.available()) {
    RPMSum = RPMSum + RPM.read();
    RPMCount++;
    Revolutions += 1.0f / (float)Pickups;
  }

  // reason we call out speed as it's own is so we can check less frequently for slow speeds
  if (SpeedUpdateTimer >= SpeedTimer) {
    SpeedUpdateTimer = 0;
    ComputeSpeed();
    RPMSum = 0;
    RPMCount = 0;
  }
  // Check if we can update the display and them compute, send, and save
  if (DisplayUpdateTimer >= UPDATE_LIMIT) {

    WatchDogTimer(RESET_WDT);

    DisplayUpdateTimer = 0;

    ComputeData();
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

    if (EXTADC.analogRead(EXTADC_KEY_PIN) > KEY_ON_LIMIT) {
      KeyState = HIGH;
    } else {
      KeyState = LOW;
    }

    if (KeyState == LOW) {
      Warnings = Warnings | KEY_OFF;
    }

    if ((KeyState == HIGH) && (OldKeyState == LOW)) {
      // key just turned on
      OldKeyState = HIGH;
      RedrawHeader = true;
      RestoreType = STATUS_OK;
      banner_back = C_DKRED;
      KeyStatus = KEY_TURNED_ON;
    } else if ((KeyState == LOW) && (OldKeyState == HIGH)) {
      // key turned off
      OldKeyState = LOW;
      RedrawHeader = true;
      RestoreType = STATUS_PITSTOP;
      banner_back = C_DKGREEN;
      KeyStatus = KEY_TURNED_OFF;
      Warnings = Warnings | KEY_OFF;
    } else if ((KeyState == HIGH) && (OldKeyState == HIGH)) {
      // car is running
      RestoreType = STATUS_OK;
      banner_back = C_DKRED;
    } else if ((KeyState == LOW) && (OldKeyState == LOW)) {
      // car must be in put
      RestoreType = STATUS_PITSTOP;
      banner_back = C_DKGREEN;
    }

    if (GPSLEDTimer > 2000) {
      digitalWrite(LAPLED_PIN, LOW);
    }

    CheckIfStarting();

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
        if (EnergyXPoint < 0) {
          EnergyXPoint = 0;
        }
        if (EnergyXPoint > 95) {
          EnergyXPoint = 95;
        }
        EnergyPoints[EnergyXPoint] = Energy;
      }

      // compute energy prediction
      if (KeyState == HIGH) {

        ComputePrediction();


        // only if "use prediciton" is on, then use it
        if (CurrentBias > 0.015f) {
          // math and logic by Delmont Goins 12/17/2025
          if ((PredictedEnergy - TotalEnergy) > ((LimitTolerance / 100.0f) * TotalEnergy)) {
            CyborgFirstLimit = CyborgFirstLimit - CurrentBias;
          } else if ((TotalEnergy - PredictedEnergy) > ((LimitTolerance / 100.0f) * TotalEnergy)) {
            // need to increase CyborgFirstLimit
            CyborgFirstLimit = CyborgFirstLimit + CurrentBias;
          }
        }
      } else {
        PredictedEnergyCounter = 0;
        PredictedEnergyAverage = 0.0f;
        PredictionArrayCounter = 0;
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
    if (!SSDStatus) {
      Warnings = Warnings | SSD_FAIL;
    }
    if (!EXTADCStatus) {
      Warnings = Warnings | EXTADC_WARNING;
    }
    if (RaceStatus == RACE_INPROGRESS) {
      Warnings = Warnings | RACE_START;
    }
    if (Volts < BatWarning) {
      Warnings = Warnings | BAT_WARNING;
    }
    if (Amps > 25.0) {
      Warnings = Warnings | AMP_WARNING;
    }
    if (LapAmps > 19.0) {
      Warnings = Warnings | LAPAMP_WARNING;
    }
    if ((GForceY > 0.6) || (!GForceStatus)) {
      Warnings = Warnings | GFORCE_WARNING;
    }
    if ((MotorTemp > TempWarning) | (MotorTemp < 10.0f)) {
      Warnings = Warnings | TEMP_WARNING;
    }
    if ((AuxTemp > TempWarning) | (AuxTemp < 10.0f)) {
      Warnings = Warnings | TEMP_WARNING;
    }
    if (!GPSStatus) {
      Warnings = Warnings | GPS_WARNING;
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
        Display.fillScreen(back_color);
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
          CYBORGView();
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
    // these are in the main loop where we compute speed
    // compute is in loop so we can slow compute time
    // to measure slow speeds
    // CarSpeed = 0.0f;
    // WRPM = 0.0f;
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
  PURPOSE : Computes data for the car
  PARAMS :  -
  RETURNS : None
  NOTES : Called by loop()
*/

void ComputeSpeed() {

  AverageCounter = Counter;

  // get car speed from wheel RPM

  if (RPMCount < MINIMUM_PULSES) {
    WRPM = 0;
    CarSpeed = 0.0f;
  } else {
    WRPM = (60.0f / (float)Pickups) * RPM.countToFrequency(RPMSum / RPMCount);
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

  if ((CarSpeed > 99.0f) || (CarSpeed < 0.0f)) {
    CarSpeed = 0.0f;
  }

  SpeedTimer = 500;
  if (CarSpeed < 4.0f) {
    SpeedTimer = 1000;
  }
  // get the driven Distance in miles
  Distance = (Revolutions * TireRad * 2.0f * 3.1416f) / (12.0f * 5280.0f);
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

  // compute motor casing temperature
  // no need to average, just one read is fine
  thmVolts = thmVolts / Counter;
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
  thxVolts = thxVolts / Counter;
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

  // compute remaining Energy note total Energy is based on battery tests
  ERem = ((TotalEnergy - Energy) / TotalEnergy) * 100.0f;
  if (ERem < 0.0f) {
    ERem = 0.0f;
  }
  if (ERem > 100.0f) {
    ERem = 100.0f;
  }

  // read time is 42 ms
  Altimiter.read();
  Altitude = (Altimiter.altitude() * METERS_TO_FEET) + AltCorrection;
  AtmPressure = Altimiter.pressure();
  AmbTemp = Altimiter.temperature();
  AmbTemp = (AmbTemp * 1.8) + 32.0 + AmbTempCF;

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
    } else {
      GPSLEDTimer = 0;
    }

    if (StartGPSFound) {
      GPSDistance = GPS.distanceBetween(GPSStartLat, GPSStartLon, GPSLat, GPSLon);  // in meters
    } else {
      GPSDistance = 0.0;
    }
  }

  // read time is 1ms
  GForceStatus = ASensor.isConnected();
  if (GForceStatus) {
    // we consider Y + to be forward
    if (ASensorDirection == 0) {
      // +X
      GForceY = -(ASensor.getAccelerationX()) / ASensorBits;
      GForceX = ASensor.getAccelerationY() / ASensorBits;
      GForceZ = ASensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 1) {
      // -X
      GForceY = (ASensor.getAccelerationX()) / ASensorBits;
      GForceX = ASensor.getAccelerationY() / ASensorBits;
      GForceZ = ASensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 2) {
      // +Y
      GForceX = ASensor.getAccelerationX() / ASensorBits;
      GForceY = -(ASensor.getAccelerationY()) / ASensorBits;
      GForceZ = ASensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 3) {
      // -Y
      GForceX = ASensor.getAccelerationX() / ASensorBits;
      GForceY = (ASensor.getAccelerationY()) / ASensorBits;
      GForceZ = ASensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 4) {
      // +Z
      GForceX = ASensor.getAccelerationX() / ASensorBits;
      GForceY = ASensor.getAccelerationY() / ASensorBits;
      GForceZ = ASensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 5) {
      // -Z
      GForceX = ASensor.getAccelerationX() / ASensorBits;
      GForceY = -(ASensor.getAccelerationY()) / ASensorBits;
      GForceZ = ASensor.getAccelerationZ() / ASensorBits;
    }
    GForceX = ((int16_t)(GForceX * 100.0f)) / 100.0f;
    GForceY = ((int16_t)(GForceY * 100.0f)) / 100.0f;
    GForceZ = ((int16_t)(GForceZ * 100.0f)) / 100.0f;
    GForce = sqrt((GForceX * GForceX) + (GForceY * GForceY) + (GForceZ * GForceZ));
    GForce = ((int16_t)(GForce * 100.0f)) / 100.0f;
  }

  if ((GForce == 0.0f) || (!GForceStatus)) {
    ASensor.initialize();
    SetupAccelerometer();
    GForceX = 0.0f;
    GForceY = 0.0f;
    GForceZ = 0.0f;
    GForce = 0.0f;
  }

  // left
  if ((abs(GForceY) > TempGForceYLapMaxL) && (abs(GForceY) > 0)) {
    TempGForceYLapMaxL = abs(GForceY);
  }
  // right
  if ((abs(GForceY) > TempGForceYLapMaxR) && (abs(GForceY) > 0)) {
    TempGForceYLapMaxR = abs(GForceY);
  }

  // reads are 8ms
  if (EnableAirFlowSensor) {
    if (!AirFlowSensorFound) {
      AirFlowSensorFound = ASSensor.begin();
    }
    if (ASSensor.readSensor(ASTemp, ASPressure)) {
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
    RawPredictedEnergy = m_PE * 10800.0f + b_PE;
    //RawPredictedEnergy = (Energy / Point) * 10800.0f;

    // this compensator will attempt to compensate for the non linear nature of the energy curve
    // the farther we are from 90 min the more we deduct. Race data shows about 70 wHr at t=0
    RawPredictedEnergy = RawPredictedEnergy - (PredictionCompensation * (10800.0f - (float)Point)) / 10800.0f;

    memmove(&EnergyArray[0], &EnergyArray[1], sizeof(float) * (PREDICT_SAMPLES - 1));
    memmove(&PointArray[0], &PointArray[1], sizeof(float) * (PREDICT_SAMPLES - 1));
    EnergyArray[PREDICT_SAMPLES - 1] = Energy;
    PointArray[PREDICT_SAMPLES - 1] = Point;

    if (RawPredictedEnergy < Energy) {
      RawPredictedEnergy = Energy;
    }

    // alwasy compute predicted energy
    PredictedEnergyAverage = PredictedEnergyAverage + RawPredictedEnergy;
    PredictedEnergyCounter++;
    if (AveragingUpdateTimer > (AveragingUpdateTime * 1000)) {
      AveragingUpdateTimer = 0;
      PredictedEnergy = PredictedEnergyAverage / (float)PredictedEnergyCounter;
      PredictedEnergyCounter = 0;
      PredictedEnergyAverage = 0.0f;
    }
  }
#ifdef DO_DEBUG
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
  Serial.print(", RPE ");
  Serial.print(RawPredictedEnergy);
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
    if (GPS.location.isValid()) {
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
      delay(50);
      // here is where we add a new record, write header data, called only once when race starts
      AddNewRecordset();
      // back out time up to this Point so driver sees starting time of zero
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
      // timer for when to record start GPS location
      StartGPSDelayTimer = 0;
      RedrawHeader = true;
      KeyStatus = KEY_NO_CHANGE;
    }
  } else {
    // race is
    // this handles driver 1 and 2
    if (Amps >= TriggerAmps) {
      if (DriverTimeOK) {
        if (KeyStatus == KEY_TURNED_ON) {
          // if race is running and driver time is within limit and key just turned on
          if (Driver < 2) {
            ChangeDriver();
            ShowNewDriverScreen();
            RaceStatus = RACE_INPROGRESS;
            KeyStatus = KEY_NO_CHANGE;
          }
        }
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
  Display.fillScreen(back_color);

  Display.setCursor(30, 30);
  Display.setFont(FONT_24BI);
  Display.setTextColor(fore_color, back_color);
  Display.print(F("Driver:"));
  Display.setFont(FONT_100BINO);
  Display.setTextColor(fore_color, back_color);
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

  StartAltitude = Altitude;

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
  Data.RPM_DNO_DID = ((uint16_t)mRPM << 4) | ((Driver & 0b0000000000000011) << 2);
  Data.RPM_DNO_DID = Data.RPM_DNO_DID & 0b1111111111111100;
  Data.WARNINGS_PE = ((uint16_t)(Warnings) << 10) | ((uint16_t)(PredictedEnergy)&0b0000001111111111);
  Data.TEMPF_TEMPX = ((uint16_t)MotorTemp << 8) | (((uint16_t)AuxTemp) & 0b0000000011111111);
  Data.VOLTS_LAPS = ((uint16_t)(Volts * 10.0f)) << 7 | (((uint16_t)LapCount) & 0b0000000001111111);
  Data.SPEED_EREM = ((uint16_t)(CarSpeed * 10.0f)) << 7 | (((uint16_t)ERem) & 0b0000000001111111);
  Data.DISTANCE_TREM = ((uint16_t)(Distance * 10.0f)) << 7 | (((uint16_t)TRem) & 0b0000000001111111);
  Data.TWHR_LAPAMPS = (uint16_t)((TotalEnergy / 10) << 9) | ((uint16_t)(LapAmps * 10.0f) & 0b0000000111111111);
  Data.AMPS_D0TIME = ((uint16_t)(abs(Amps) * 10.0f)) << 5 | (((uint16_t)(DriverTime[0] / 1000)) & 0b0000111111111111) >> 7;
  if (Amps < 0) {
    Data.AMPS_D0TIME = Data.AMPS_D0TIME | 0b1000000000000000;
  } else {
    Data.AMPS_D0TIME = Data.AMPS_D0TIME & 0b0111111111111111;
  }
  Data.LAP2AMPS_D0TIME = ((uint16_t)(abs(TargetAmps) * 10.0f)) << 7 | (((uint16_t)(DriverTime[0] / 1000) & 0b0000000001111111));
  Data.ENERGY = ((uint16_t)(Energy) << 6);
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
  // we disable cyborg on board 9, but we still have to transmit something
  // Data.CYBORGOUT_CYBORGLO = 0;
  Data.CYBORGOUT_CYBORGLO = (((uint16_t)(CyborgOutSignal)&0b0000000001111111) << 9) | (((uint16_t)(CyborgFirstLimit * 10.0f) & 0b0000000111111111));
  Data.LAT = GPSLat;
  Data.LON = GPSLon;
  DataPacket.sendData();

  // ESerial.write((uint8_t *)&Data, PacketSize);

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

  bool RestoreEEPROM = false;

  ButtonPressed = WhatButtonWasPressed();

  if ((ButtonPressed == L_BUTTON) || (ButtonPressed == R_BUTTON)) {

    Display.setRotation(1);
    Display.fillScreen(C_RED);
    Display.setFont(FONT_16B);
    Display.setTextColor(C_WHITE, back_color);
    Display.setCursor(10, 30);
    Display.print(F("Resetting Race"));

    WaitForRelease();

    ResetRaceDate();
    SaveStartGPS(false);
  }

  // if unprogrammed or user want's to reset
  // could be due to corrupted eeprom data or added parameter and eeprom has some old data
  else if ((ButtonPressed == C_BUTTON)) {
    Display.setRotation(1);
    Display.fillScreen(C_RED);
    Display.setFont(FONT_16B);
    Display.setTextColor(C_WHITE, back_color);
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
      /* we do not store these in memory as they are only used when calibrating sensors
        or when downloading the settings to a text file
        ...hence no need to consume memory for rare usage
        adding here as a reminder to NOT use these addresses
        VoltSensorCalibrationDate = 0;
        EEPROM.put(2, SensorCalibrationDate);
        AmpSensorCalibrationDate = 0;
        EEPROM.put(4, SensorCalibrationDate);
        TempSensorCalibrationDate = 0;
        EEPROM.put(6, SensorCalibrationDate);
      */

      MotorSprocket = 15;
      EEPROM.put(10, MotorSprocket);
      WheelSprocket = 70;
      EEPROM.put(20, WheelSprocket);
      DataJustificationRight = 0;
      EEPROM.put(25, DataJustificationRight);
      TireID = 0;
      EEPROM.put(30, TireID);
      TirePressure = 115;
      EEPROM.put(35, TirePressure);
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
      TotalEnergy = 600;
      EEPROM.put(70, TotalEnergy);
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
      MaxLapAmps = 19.5;
      EEPROM.put(155, MaxLapAmps);
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
      mVPerAmp = 30.0f;
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
      TriggerAmps = 20.0f;
      EEPROM.put(380, TriggerAmps);
      CyborgThresholdVolts = 3.2f;
      EEPROM.put(385, CyborgThresholdVolts);
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
      AveragingUpdateTime = 1;
      EEPROM.put(492, AveragingUpdateTime);
      CyborgUpdateTime = 100;
      EEPROM.put(495, CyborgUpdateTime);
      Kp = 0.5f;
      EEPROM.put(500, Kp);
      Ki = 150.0f;
      EEPROM.put(510, Ki);
      Kd = 0.1f;
      EEPROM.put(520, Kd);
      LimitTolerance = 1.0f;
      EEPROM.put(525, LimitTolerance);
      PredictionCompensation = 86.0f;
      EEPROM.put(530, PredictionCompensation);
      CyborgSecondLimit = 25.0;
      EEPROM.put(535, CyborgSecondLimit);

      Display.setCursor(10, 200);
      Display.print(F("Restoring complete."));
      delay(1000);
      Display.fillScreen(C_BLACK);
    }
  }
#ifdef DO_DEBUG
  Serial.println(F("Getting EEPROM data"));
#endif
  /* reminder to not use these addresses
  EEPROM.get(5, VoltCalibrationDate);
  EEPROM.get(5, CurrentCalibrationDate);
  EEPROM.get(5, TemperatureCalibrationDate);
  */
  EEPROM.get(10, MotorSprocket);
  EEPROM.get(20, WheelSprocket);
  EEPROM.get(25, DataJustificationRight);
  EEPROM.get(30, TireID);
  EEPROM.get(35, TirePressure);
  EEPROM.get(40, Theme);
  EEPROM.get(45, AltCorrection);
  EEPROM.get(47, GPSAltCorrection);
  EEPROM.get(50, Orientation);
  EEPROM.get(55, AutoCurrentCal);
  EEPROM.get(60, RadioUpdate);
  EEPROM.get(70, TotalEnergy);
  EEPROM.get(110, VoltageSlope);
  EEPROM.get(120, VoltageOffset);
  EEPROM.get(130, MotorID);
  EEPROM.get(140, TempWarning);
  EEPROM.get(150, BatWarning);
  EEPROM.get(155, MaxLapAmps);
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
  EEPROM.get(385, CyborgThresholdVolts);
  EEPROM.get(400, RaceMonth);
  EEPROM.get(410, EnableAirFlowSensor);
  EEPROM.get(412, AirSpeedOffset);
  EEPROM.get(450, AccelCalX);
  EEPROM.get(460, AccelCalY);
  EEPROM.get(470, AccelCalZ);
  EEPROM.get(480, AmbTempCF);
  EEPROM.get(485, CyborgFirstLimit);
  EEPROM.get(490, EnableCyborg);
  EEPROM.get(492, AveragingUpdateTime);
  EEPROM.get(495, CyborgUpdateTime);
  EEPROM.get(500, Kp);
  EEPROM.get(510, Ki);
  EEPROM.get(520, Kd);
  EEPROM.get(525, LimitTolerance);
  EEPROM.get(530, PredictionCompensation);
  EEPROM.get(535, CyborgSecondLimit);

  // bounds check the ID to ensure it's not past array bounds
  if (ASensorDirection > (sizeof(ASensorDirectionText) / sizeof(ASensorDirectionText[0]))) {
    ASensorDirection = 0;
  }

  OldCyborgFirstLimit = CyborgFirstLimit;
  OldCyborgSecondLimit = CyborgSecondLimit;

  SetTextJustification();

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
  Serial.print(F("Battery Warning: "));
  Serial.println(BatWarning);
  Serial.print(F("Lap Threshold: "));
  Serial.println(LapThreshold);
  Serial.print(F("Temp Warning: "));
  Serial.println(TempWarning);
  Serial.print(F("MaxLapAmps: "));
  Serial.println(MaxLapAmps);
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
  Serial.print(F("CyborgThresholdVolts: "));
  Serial.println(CyborgThresholdVolts);
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

void SetTextJustification() {
  if (DataJustificationRight == 1) {
    ffMainData.setJustification(JUSTIFY_RIGHT);
    DATA_X = 305;
  } else {
    ffMainData.setJustification(JUSTIFY_LEFT);
    DATA_X = 15;
  }
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
  uint16_t StatusBarCounter = 0, StatusBarWidth = 0;
  uint32_t DataRecord = 0, HeaderRecord = 0;
  uint32_t TempTime;
  uint32_t TempDriver0 = 0, TempDriver1 = 0, TempDriver2 = 0, ii = 0;
  uint32_t StartRecord = 0, LastRecord;
  uint32_t EnergyPointCounter = 0;
  uint8_t RecordsToBackup = 1;
  uint32_t TempDriver = 0;
  float InitalEnergy = 0.0f;

  // get the last known record and read the data

  Display.setTextColor(C_CYAN);
  Display.setCursor(STATUS_RESULT, 220);
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

  // got to headerRecord and get the initial start altituide
  HeaderRecord = SSD.getFirstRecord(RecordSETID, hrID);
  SSD.gotoRecord(HeaderRecord);

#ifdef DO_DEBUG
  Serial.println("Line 2222 Header Record _________________________");
  Serial.print("RecordType ");
  Serial.println(SSD.getHeaderField(RecordType, hrType));
  Serial.print("RecordSETID ");
  Serial.println(SSD.getHeaderField(RecordSETID, hrID));
  Serial.print("Tyear ");
  Serial.println(SSD.getHeaderField(Tyear, hrYear));
  Serial.print("Tmonth ");
  Serial.println(SSD.getHeaderField(Tmonth, hrMonth));
  Serial.print("Tday ");
  Serial.println(SSD.getHeaderField(Tday, hrDay));
  Serial.print("Thour ");
  Serial.println(SSD.getHeaderField(Thour, hrHour));
  Serial.print("Tminute ");
  Serial.println(SSD.getHeaderField(Tminute, hrMinute));
  Serial.print("MotorSprocket ");
  Serial.println(SSD.getHeaderField(MotorSprocket, hrMSprocket));
  Serial.print("WheelSprocket ");
  Serial.println(SSD.getHeaderField(WheelSprocket, hrWSprocket));
  Serial.print("TirePressure ");
  Serial.println(SSD.getHeaderField(TirePressure, hrTirePressureID));
  Serial.print("MotorID ");
  Serial.println(SSD.getHeaderField(MotorID, hrMotorID));
  Serial.print("AmbTemp ");
  Serial.println(SSD.getHeaderField(AmbTemp, hrTemp));
  Serial.print("StartAltitude ");
  Serial.println(SSD.getHeaderField(StartAltitude, hrStartAltitude));
  Serial.print("AtmPressure ");
  Serial.println(SSD.getHeaderField(AtmPressure, hrStartPressure));
  Serial.print("TotalEnergy ");
  Serial.println(SSD.getHeaderField(TotalEnergy, hrEnergy));
  Serial.print("AverageCounter ");
  Serial.println(SSD.getHeaderField(AverageCounter, hrCounter));
  Serial.print("Battery1 ");
  Serial.println(SSD.getHeaderField(Battery1, hrBattery1));
  Serial.print("Battery2 ");
  Serial.println(SSD.getHeaderField(Battery2, hrBattery2));
#endif

  StartAltitude = SSD.getHeaderField(StartAltitude, hrStartAltitude);

  // now go to the data record and get race data (this will be used during restore)
  SSD.gotoRecord(DataRecord);
  RecordType = SSD.getField(RecordType, frType);
  Point = SSD.getField(Point, frPoint);
  LapCount = SSD.getField(LapCount, frLap);
  Driver = SSD.getField(Driver, frDriver);
  Energy = SSD.getField(Energy, frEnergy);
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

    if (Point % ((60 * 1000) / UPDATE_LIMIT) == 0) {
      Energy = SSD.getField(Energy, frEnergy);
      EnergyPoints[EnergyPointCounter] = Energy;
      EnergyPointCounter++;
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
  DriverTime[0] = (TempDriver0 * UPDATE_LIMIT * 1000) / 1000;
  DriverTime[1] = (TempDriver1 * UPDATE_LIMIT * 1000) / 1000;
  DriverTime[2] = (TempDriver2 * UPDATE_LIMIT * 1000) / 1000;

#ifdef DO_DEBUG
  Serial.println("Line 2343 computed driver times _________________________");
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
  Serial.println("Line 2363 Last record data _________________________");
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
  Serial.print("StartAltitude ");
  Serial.println(StartAltitude);
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

  if (EXTADC.analogRead(EXTADC_KEY_PIN) < KEY_ON_LIMIT) {
    // key off, probably in in pit
    RestoreType = STATUS_PITSTOP;
  } else {
    // key on probably restart during run time, meaning key probably on
    RestoreType = STATUS_RESTORE;
  }
  // compensate for if we backed up any records
  TempTime = TempTime + 1 + RecordsToBackup;

  // add missing records to the database, this will fill in the downtime with default data

  Display.setTextColor(C_BLACK);
  Display.setCursor(STATUS_RESULT, 220);
  Display.print(F("Reading: "));

  Display.setTextColor(C_CYAN);
  Display.setCursor(STATUS_RESULT, 220);
  Display.print(F("Recreating: "));

  // now we are on the first writable record
  SSD.gotoRecord(LastRecord);
  // Get start time

  uint32_t StartTime = millis();

  for (i = 0; i <= RecordsToRestore; i++) {

    //Display.setCursor(270, 220);
    //ffVolts.setTextColor(C_CYAN, C_BLACK);
    //ffVolts.print(i);

    StatusBarCounter++;
    StatusBarWidth = ((float)(i * 160.0) / RecordsToRestore) + 2;
    Display.fillRoundRect(STATUS_RESULT, 200, StatusBarWidth, 18, 2, C_GREEN);

#ifdef DO_DEBUG
    Serial.print("Saving to SSD, Point: ");
    Serial.println(Point);
#endif

    // clock must increment by UPDATE_LIMIT
    RealClockTime = TempTime + (int)((i * UPDATE_LIMIT) / 1000);
    if (Point > ((float)((1000.0 / UPDATE_LIMIT)) * (RACE_TIME_SECONDS + RACE_EXTENSION))) {
      break;
    }

    if (SSD.addRecord()) {
      SSD.saveRecord();
    }
    delay(5);

    if (i % ((60 * 1000) / UPDATE_LIMIT) == 0) {
      EnergyPointCounter++;
      EnergyPoints[EnergyPointCounter] = Energy;
    }

    Point++;
  }

  // we have incremented a point but not added a record
  // so back out that point
  Point--;

  Duration = ((hour() * 3600) + (minute() * 60) + second()) - ((RaceHour * 3600) + (RaceMinute * 60) + RaceSecond);

  StartTime = 0;

  Duration = Duration + 7 + ((millis() - StartTime) / 1000);

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

void StartDisplay() {
  Display.begin();
  Display.setTextWrap(false);
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
    Display.print(F("SPEED"));
    Display.fillRect(0, 170, 320, 70, C_DKGREY);
    // Display.fillRect(0, 170, 320, 4, fore_color);
    Display.setFont(FONT_16B);
    Display.setCursor(5, 180);

    Display.print(F("Lap Average"));
    LapAverageBar.setSectionColors(C_GREEN, C_GREEN, C_GREEN, C_VDKGREY);
    LapAverageBar.setSectionSize(19, 22);
    LapAverageBar.setScale(18, 30, 1);
    RedrawHeader = false;
  }

  Display.setFont(FONT_16B);
  Display.setCursor(DATA_X, DATA_Y);
  Display.setFont(FONT_100BINO);
  ffMainData.setTextColor(fore_color, back_color);
  ffMainData.print(CarSpeed, 1);

  LapAverageBar.draw(LapSpeed);
  Display.setFont(FONT_48BINO);
  Display.setCursor(LADATA_X, LADATA_Y);
  ffLapSpeed.setTextColor(C_WHITE, C_DKGREY);
  ffLapSpeed.print(LapSpeed, 1);

  if (AirSpeed > (CarSpeed + 6.0f)) {
    Display.fillRect(0, 170, 320, 4, C_RED);
  } else if (AirSpeed > (CarSpeed + 3.0f)) {
    Display.fillRect(0, 170, 320, 4, C_YELLOW);
  } else if (AirSpeed < (CarSpeed - 3.0f)) {
    Display.fillRect(0, 170, 320, 4, C_GREEN);
  } else {
    Display.fillRect(0, 170, 320, 4, fore_color);
  }
}

/*
   PURPOSE : Draws cyborg View
   PARAMS :  -
   RETURNS : None
   NOTES : Draws signal going to the esc as well as energy (both in %)
*/

void CYBORGView() {

  uint16_t CyborgColor = 0;
  uint8_t ThrottlePercent = 0;

  if (RedrawHeader) {
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("Cyborg"));

    Display.setTextColor(fore_color);
    Display.setFont(FONT_14);

    for (i = 0; i <= 100; i += 20) {
      if (i == 100) {
        Display.setCursor((i * 3) - 20, 135);
      } else if (i == 0) {
        Display.setCursor(10 + (i * 3), 135);
      } else {
        Display.setCursor((i * 3), 135);
      }
      Display.print(i);
    }
    for (i = 0; i <= 100; i += 10) {
      Display.drawLine(10 + (i * 3), 125, 10 + (i * 3), 130, fore_color);
    }
    Display.drawLine(10, 125, 310, 125, fore_color);

    Display.fillRect(0, 165, 320, 75, C_DKGREY);
    Display.fillRect(0, 165, 320, 4, fore_color);
    Display.setFont(FONT_16B);
    Display.setTextColor(C_WHITE);
    Display.setCursor(5, 175);
    Display.print(F("Race"));
    Display.setCursor(250, 175);
    Display.print(F("Bias"));
    Display.fillRect(160 - 80, 165, 160, 75, fore_color);


    RedrawHeader = false;
  }

  if (CyborgActive) {
    CyborgColor = C_GREEN;
  } else {
    CyborgColor = C_RED;
  }

  //ThrottlePercent = ((TotalEnergy - Energy) * 100) / TotalEnergy;
  ThrottlePercent = CyborgInSignal;

  if (CyborgOutSignal <= ThrottlePercent) {
    Display.fillRect(10, 50, (CyborgOutSignal * 3), 60, C_MDBLUE);
    Display.fillRect((ThrottlePercent * 3) + 10, 50, 6, 60, CyborgColor);
    Display.fillRect((CyborgOutSignal * 3) + 10, 50, (ThrottlePercent - CyborgOutSignal) * 3, 60, C_DKBLUE);
    Display.fillRect((ThrottlePercent * 3) + 10 + 6, 50, 314 - (ThrottlePercent * 3) - 6, 60, C_DKBLUE);
  } else {
    Display.fillRect(10, 50, (ThrottlePercent)*3, 60, C_MDBLUE);
    Display.fillRect((ThrottlePercent * 3) + 10, 50, 6, 60, CyborgColor);
    Display.fillRect((ThrottlePercent * 3) + 10 + 6, 50, (CyborgOutSignal - ThrottlePercent) * 3, 60, C_MDBLUE);
    Display.fillRect((CyborgOutSignal * 3) + 16, 50, 314 - (CyborgOutSignal * 3) - 6, 60, C_DKBLUE);
  }

  Display.setFont(FONT_24BI);
  Display.setCursor(70, 200);
  ffCyborgFirstLimit.setTextColor(C_WHITE, C_DKGREY);
  ffCyborgFirstLimit.print(CyborgFirstLimit, 1);

  Display.setCursor(310, 200);
  ffCyborgSecondLimit.setTextColor(C_WHITE, C_DKGREY);
  ffCyborgSecondLimit.print(CurrentBias, 2);

  Display.setFont(FONT_48BINO);
  Display.setCursor(230, 180);
  ffPredict.setTextColor(back_color, fore_color);

  sprintf(buf, "%d%%", (uint16_t)(PredictedEnergy * 100.0f / TotalEnergy));
  ffPredict.print(buf);
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
    Display.print(F("AMPS"));
    Display.fillRect(0, 170, 320, 70, C_DKGREY);
    Display.fillRect(0, 170, 320, 4, fore_color);
    Display.setFont(FONT_16B);
    Display.setCursor(5, 180);
    Display.print(F("Lap Average"));
    LapAverageBar.setSectionColors(C_GREEN, C_YELLOW, C_RED, C_VDKGREY);
    LapAverageBar.setSectionSize(MaxLapAmps - 0.5f, MaxLapAmps + 0.5f);
    LapAverageBar.setScale(MaxLapAmps - 5.0f, MaxLapAmps + 5.0f, 1);
    RedrawHeader = false;
  }

  ffMainData.setTextColor(fore_color, back_color);
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
  ffLapAmps.setTextColor(C_WHITE, C_DKGREY);
  ffLapAmps.print(LapAmps, 1);
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
    Display.print(F("VOLTS"));
    Display.fillRect(0, 170, 320, 70, C_DKGREY);
    Display.fillRect(0, 170, 320, 4, fore_color);
    Display.setFont(FONT_16B);
    Display.setCursor(5, 180);
    Display.print(F("Lap Average"));
    LapAverageBar.setSectionSize(BatWarning, BatWarning + 1.0f);
    LapAverageBar.setScale(18, 24, 1);
    RedrawHeader = false;
  }

  Display.setCursor(DATA_X, DATA_Y);
  Display.setFont(FONT_100BINO);
  ffMainData.setTextColor(fore_color, back_color);
  ffMainData.print(Volts, 1);

  //Bottom Info
  if ((LapVolts + 1.0f) < BatWarning) {
    LapAverageBar.setSectionColors(C_RED, C_RED, C_RED, C_VDKGREY);
  } else if (LapVolts < BatWarning) {
    LapAverageBar.setSectionColors(C_YELLOW, C_YELLOW, C_YELLOW, C_VDKGREY);
  } else {
    LapAverageBar.setSectionColors(C_GREEN, C_GREEN, C_GREEN, C_VDKGREY);
  }
  LapAverageBar.draw(LapVolts);
  Display.setFont(FONT_48BINO);
  Display.setCursor(LADATA_X, LADATA_Y);
  ffLapVolts.setTextColor(C_WHITE, C_DKGREY);
  ffLapVolts.print(LapVolts, 1);
}

void EnergyView() {

  if (RedrawHeader) {
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("ENERGY"));
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
    for (i = 0; i < 16; i++) {
      GraphPointX = i * 6;
      if (GraphPointX > 89) {
        GraphPointX = 89;
      }
      EnergyG.setX(GraphPointX);
      EnergyG.plot(bEnergyID, BLEnergy[i] * (TotalEnergy / 600.0f));  // 600 is base line data for good batteries
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
  Display.fillTriangle(302, oEPointer, 302 + 15, oEPointer - 5, 302 + 15, oEPointer + 5, back_color);
  Display.fillTriangle(302, EPointer, 302 + 15, EPointer - 5, 302 + 15, EPointer + 5, C_GREEN);
  oEPointer = EPointer;

  Display.setFont(FONT_24BI);
  Display.setCursor(280, 180);
  ffPredict.setTextColor(fore_color, back_color);
  ffPredict.print(PredictedEnergy, 0);
}

void GForceView() {

  if (RedrawHeader) {
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("G-Force"));
    Display.fillRect(0, 170, 320, 70, C_DKGREY);
    Display.fillRect(0, 170, 320, 4, fore_color);
    Display.setFont(FONT_16B);
    Display.setCursor(83, 180);
    Display.print(F("Lap Maximums"));
    Display.fillRect(158, 110, 4, 40, fore_color);
    for (i = 0; i <= 25; i++) {
      Display.fillRect(164 + (i * 6), 110 - (i / 2.0), 5, 40 + (i), C_VDKGREY);
    }
    for (i = 0; i <= 25; i++) {
      Display.fillRect(151 - (i * 6), 110 - (i / 2.0), 5, 40 + (i), C_VDKGREY);
    }
    RedrawHeader = false;
  }

  Display.setFont(Arial_48BINO);
  /*
  Display.setCursor(180, 40);
  // keep the text from bouncing around if < 0
  if (GForceY < 0) {
    Display.setTextColor(fore_color, back_color);
    Display.print("-");
  } else {
    Display.setTextColor(back_color, back_color);
    Display.print("-");
  }
*/
  Display.setCursor(220, 45);
  ffGForceY.setTextColor(fore_color, back_color);
  ffGForceY.print(abs(GForceY), 2);

  if (PeakTimer >= PEAK_TIMER) {
    PeakTimer = 0;
    PeakGValue = 0;
  }

  if (((OldGForceY <= 0) && (GForceY >= 0)) || ((OldGForceY >= 0) && (GForceY <= 0))) {
    OldGForceY = GForceY;
    PeakGValue = 0;
    PeakColor = C_VDKGREY;
  }

  if (GForceY > 0) {
    for (i = 0; i <= 25; i++) {
      Display.fillRect(151 - (i * 6), 110 - (i / 2.0), 5, 40 + (i), C_VDKGREY);
    }
    if ((i == 0) && (PeakGValue > 0)) {
      PeakGValue = 0;
    }
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
          Display.fillRect(164 + (i * 6), 110 - (i / 2.0), 5, 40 + (i), C_VDKGREY);
        }
      }
    }
    if (PeakTimer < PEAK_TIMER) {
      Display.fillRect(164 + (PeakGValue * 6), 110 - (PeakGValue / 2.0), 5, 40 + (PeakGValue), PeakColor);
    }

  } else {
    for (i = 0; i <= 25; i++) {
      Display.fillRect(164 + (i * 6), 110 - (i / 2.0), 5, 40 + (i), C_VDKGREY);
    }

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
          Display.fillRect(151 - (i * 6), 110 - (i / 2.0), 5, 40 + (i), C_VDKGREY);
        }
      }
    }
    if (PeakTimer < PEAK_TIMER) {
      Display.fillRect(151 - (PeakGValue * 6), 110 - (PeakGValue / 2.0), 5, 40 + (PeakGValue), PeakColor);
    }
  }

  Display.setFont(FONT_24BI);
  Display.setCursor(80, 205);
  ffGForceYLapMaxL.setTextColor(C_WHITE, C_DKGREY);
  ffGForceYLapMaxL.print(GForceYLapMaxL, 2);

  Display.setCursor(305, 205);
  ffGForceYLapMaxR.setTextColor(C_WHITE, C_DKGREY);
  ffGForceYLapMaxR.print(GForceYLapMaxR, 2);
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
    Display.print(F("TEMP"));
    Display.fillRect(0, 200, 320, 40, fore_color);
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

  // bottom stuff

  // Display.setFont(FONT_16B);
  Display.setFont(FONT_24BI);

  // Display.setCursor(105, 208);
  if (hour() > 12) {
    sprintf(buf, "%d:%02d:%02d", hour() - 12, minute(), second());
  } else {
    sprintf(buf, "%d:%02d:%02d", hour(), minute(), second());
  }

  Display.setCursor(100, 208);
  ffTime.setTextColor(back_color, fore_color);
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
    //Display.fillScreen(back_color);
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("TIME"));
    Display.fillRect(0, 170, 320, 70, C_DKGREY);
    Display.fillRect(0, 170, 320, 4, fore_color);
    Display.setFont(FONT_16B);
    Display.setTextColor(C_WHITE, C_DKGREY);
    Display.setCursor(5, 180);
    Display.print(F("Seat Time"));
    Display.setCursor(215, 180);
    Display.print(F("Last Lap"));
    Display.fillRect(160 - 40, 170, 80, 65, fore_color);

    RedrawHeader = false;
  }

  // split time
  if (RaceStatus == RACE_INPROGRESS) {
    if (TimeDelta > 0) {
      ffMainData.setTextColor(C_RED, back_color);
      sprintf(buf, "+%d", TimeDelta);
    } else if (TimeDelta < 0) {
      sprintf(buf, "-%d", abs(TimeDelta));
      ffMainData.setTextColor(C_GREEN, back_color);
    } else {
      sprintf(buf, "%d", TimeDelta);
      ffMainData.setTextColor(fore_color, back_color);
    }
  } else if ((RaceStatus == RACE_FINISHED) || (RaceStatus == RACE_NOTSTARTED)) {
    ffMainData.setTextColor(fore_color, back_color);
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
  ffDriverTime.setTextColor(C_WHITE, C_DKGREY);
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
  ffDriverLapTime.setTextColor(C_WHITE, C_DKGREY);
  ffDriverLapTime.print(buf);

  Display.setFont(FONT_48BINO);
  Display.setCursor(145, 180);
  Display.setTextColor(back_color, fore_color);
  Display.print(Driver + 1);
}

/*
   PURPOSE : Draws Usage View
    PARAMS :  -
   RETURNS : None
     NOTES : Draws Usage view in the Display Data Function when called in the switch
*/

void UsageView() {

  if (RedrawHeader) {
    //Display.fillScreen(back_color);
    Display.setFont(FONT_16B);
    Display.fillRect(0, 0, 320, 38, banner_back);
    Display.setCursor(10, 10);
    Display.setTextColor(C_WHITE);
    Display.print(F("USAGE"));
    Display.setFont(FONT_14);
    Display.setCursor(140, 50);
    Display.print(F("CONSUMPTION ("));
    Display.print(Driver + 1);
    Display.print(F(")"));
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
    RedrawHeader = false;
    TRemG.refresh();
    ERemG.refresh();
  }

  //Draw Time
  Display.setTextColor(fore_color);
  TRemG.draw(TRem);

  // draw the Energy and time remaining, use red if it gets below time by 3%
  if ((TRem - ERem) > 10) {
    ERemG.setSectionColors(C_RED, C_RED, C_RED, C_VDKGREY);
  } else if ((TRem - ERem) > 5) {
    ERemG.setSectionColors(C_YELLOW, C_YELLOW, C_YELLOW, C_VDKGREY);
  } else {
    ERemG.setSectionColors(C_GREEN, C_GREEN, C_GREEN, C_VDKGREY);
  }
  ERemG.draw(ERem);

  Display.setFont(FONT_24BI);

  Display.setCursor(305, 85);
  ffLapAmps.setTextColor(fore_color, back_color);
  ffLapAmps.print(LapAmps, 1);

  Display.setCursor(305, 140);
  ffLapEnergy.setTextColor(fore_color, back_color);
  ffLapEnergy.print(LapEnergy, 1);

  Display.setCursor(305, 195);
  ffEnergy.setTextColor(fore_color, back_color);
  ffEnergy.print(Energy, 0);
}

/*
   PURPOSE : Generates warnings based on car data
    PARAMS :  -
   RETURNS : None
     NOTES : Warnings are displayed in the form of an icon
*/


void DrawPitMessage() {

  DriverTimeOK = false;
  // math and logic by Delmont Goins 6/3/2025
  // test driver
  if (Driver == 0) {
    if ((DriverTime[0] < (15 * 60 * 1000))) {
      ffPitMessage.setTextColor(C_WHITE, banner_back);
      Display.setFont(FONT_16B);
      Display.setCursor(PITMESSAGEX, 10);
      ffPitMessage.print("Can pit in:");
      PitTime = (15 * 60 * 1000) - (DriverTime[0]);  // rule is 15 min but +1 second to address round off
      PitTime = (PitTime / 1000) + 1;
      ffPitTime.setTextColor(C_WHITE, banner_back);
      mn = PitTime / 60;
      sc = abs(PitTime) % 60;
      Display.setFont(FONT_24BI);
      sprintf(buf, "% 01d:%02d", mn, sc);
      Display.setCursor(PITTIMEX, 6);
      ffPitTime.print(buf);
    } else if ((DriverTime[0] >= (15 * 60 * 1000)) && (DriverTime[0] < (45 * 60 * 1000))) {
      DriverTimeOK = true;
      Display.setFont(FONT_16B);
      ffPitMessage.setTextColor(C_WHITE, banner_back);
      Display.setCursor(PITMESSAGEX, 10);
      ffPitMessage.print("Pit within:");
      PitTime = (45 * 60 * 1000) - DriverTime[0];
      PitTime = (PitTime / 1000) + 1;
      ffPitTime.setTextColor(C_WHITE, banner_back);
      if (PitTime < 300) {
        ffPitTime.setTextColor(C_YELLOW, banner_back);
      }
      mn = PitTime / 60;
      sc = abs(PitTime) % 60;
      Display.setFont(FONT_24BI);
      sprintf(buf, "% 01d:%02d", mn, sc);
      Display.setCursor(PITTIMEX, 6);
      ffPitTime.print(buf);
    } else if (DriverTime[0] >= (45 * 60 * 1000)) {
      DriverTimeOK = true;
      Display.setFont(FONT_16B);
      ffPitMessage.setTextColor(C_YELLOW, banner_back);
      Display.setCursor(PITMESSAGEX, 10);
      ffPitMessage.print("PIT NOW!");
      Display.setCursor(PITTIMEX, 6);
      Display.setFont(FONT_24BI);
      ffPitTime.setTextColor(C_YELLOW, banner_back);
      PitTime = (45 * 60 * 1000) - DriverTime[0];
      PitTime = (PitTime / 1000) + 1;
      if (PitTime < 0) {
        PitTime = abs(PitTime);
        mn = PitTime / 60;
        sc = PitTime % 60;
        sprintf(buf, "-%01d:%02d", mn, sc);
      } else {
        mn = PitTime / 60;
        sc = (PitTime) % 60;
        sprintf(buf, " %01d:%02d", mn, sc);
      }
      ffPitTime.print(buf);
    }
  }

  else if (Driver == 1) {
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
      Display.setFont(FONT_16B);
      ffPitMessage.setTextColor(C_WHITE, banner_back);
      Display.setCursor(PITMESSAGEX, 10);
      ffPitMessage.print("Can pit in:");
      PitTime = (MinDriveTime / 1000) - (DriverTime[1] / 1000) + 1;
      ffPitTime.setTextColor(C_WHITE, banner_back);
      mn = PitTime / 60;
      sc = abs(PitTime) % 60;
      Display.setFont(FONT_24BI);
      sprintf(buf, "% 01d:%02d", mn, sc);
      Display.setCursor(PITTIMEX, 6);
      ffPitTime.print(buf);
    } else if ((DriverTime[1] >= MinDriveTime) && (DriverTime[1] <= (MaxDriveTime - (0 * 60 * 1000)))) {
      DriverTimeOK = true;
      ffPitMessage.setTextColor(C_WHITE, banner_back);
      Display.setFont(FONT_16B);
      Display.setCursor(PITMESSAGEX, 10);
      ffPitMessage.print("Pit within:");
      PitTime = (MaxDriveTime / 1000) - (DriverTime[1] / 1000) + 1;  // in min
      ffPitTime.setTextColor(C_WHITE, banner_back);
      if (PitTime < 300) {
        ffPitTime.setTextColor(C_YELLOW, banner_back);
      }
      mn = PitTime / 60;
      sc = abs(PitTime) % 60;
      sprintf(buf, "% 01d:%02d", mn, sc);
      Display.setFont(FONT_24BI);
      Display.setCursor(PITTIMEX, 6);
      ffPitTime.print(buf);
    } else {
      DriverTimeOK = true;
      Display.setCursor(PITMESSAGEX, 10);
      ffPitMessage.setTextColor(C_YELLOW, banner_back);
      Display.setFont(FONT_16B);
      ffPitMessage.print("PIT NOW!");
      ffPitTime.setTextColor(C_YELLOW, banner_back);
      Display.setFont(FONT_24BI);
      Display.setCursor(PITTIMEX, 6);
      PitTime = MaxDriveTime - DriverTime[1];
      PitTime = (PitTime / 1000) + 1;
      if (PitTime < 0) {
        PitTime = abs(PitTime);
        mn = PitTime / 60;
        sc = PitTime % 60;
        sprintf(buf, "-%01d:%02d", mn, sc);
      } else {
        mn = PitTime / 60;
        sc = (PitTime) % 60;
        sprintf(buf, " %01d:%02d", mn, sc);
      }
      ffPitTime.print(buf);
    }
  } else if (Driver == 2) {
    ffPitMessage.setTextColor(C_WHITE, banner_back);
    Display.setFont(FONT_16B);
    Display.setCursor(PITMESSAGEX, 10);
    ffPitMessage.print("Time left:");
    if (((DriverTime[0] + DriverTime[1] + DriverTime[2]) / 1000) > 5400) {
      PitTime = 0;
    } else {
      PitTime = 5400 - (DriverTime[0] / 1000) - (DriverTime[1] / 1000) - (DriverTime[2] / 1000) + 1;  // in sec
    }
    ffPitTime.setTextColor(C_WHITE, banner_back);
    if (PitTime < 0) {
      PitTime = 0;
    }
    mn = PitTime / 60;
    sc = PitTime % 60;
    sprintf(buf, "% 01d:%02d", mn, sc);
    Display.setFont(FONT_24BI);
    Display.setCursor(PITTIMEX, 6);
    ffPitTime.print(buf);
  }
}

void DrawWarnings() {
  // force fails to test icons
  // Warnings = 0b1111111111111111;
  // show racing status
  /*
  if (Warnings & RACE_START) {
    drawBitmap(93, 3, start_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(93, 3, start_icon, 32, 32, banner_back);
  }
*/
  // SSD chip--this is bad...
  if (Warnings & SSD_FAIL) {
    drawBitmap(125, 3, ssd_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(125, 3, ssd_icon, 32, 32, banner_back);
  }

  // battery
  if (Warnings & BAT_WARNING) {
    drawBitmap(157, 3, battery_icon, 32, 32, C_WHITE);
  } else {
    drawBitmap(157, 3, battery_icon, 32, 32, banner_back);
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
      TargetAmps = TempTargetAmps / 2.0;
    }

    LastLapTime = LapTime;

    LapTime = LapTimer / 1000;

    TimeDelta = LapTime - LastLapTime;

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
    sprintf(buf, "%d:%02d:%02d, %d/%02d/%02d", hour() - 12, minute(), second(), month(), day(), year());
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
  Radio.SetAddressH(0);
  Radio.SetAddressL(0);
  Radio.SetSpeed(0b00011100);
  Radio.SetChannel(1);
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
  TopMainMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, 35, 5, "Main Menu", FONT_16B, FONT_16B);
  MainMenuOption1 = TopMainMenu.add565("Race", race_icon565, 32, 32);
  MainMenuOption9 = TopMainMenu.add565("Race Playback", graph_icon565, 32, 32);
  MainMenuOption6 = TopMainMenu.add565("Data Storage", SSD_icon565, 32, 32);
  MainMenuOption8 = TopMainMenu.add565("Cyborg", cyborg_icon565, 32, 32);
  MainMenuOption2 = TopMainMenu.add565("Settings", car_icon565, 32, 32);
  MainMenuOption3 = TopMainMenu.add565("Wireless", transceiver_icon565, 32, 32);
  MainMenuOption4 = TopMainMenu.add565("Sensors", calibrate_icon565, 32, 32);
  MainMenuOption7 = TopMainMenu.add565("Accelerometer", GForce_icon565, 32, 32);
  MainMenuOption5 = TopMainMenu.add565("Clock", clock_icon565, 32, 32);
  TopMainMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  TopMainMenu.setMenuBarMargins(10, 319, 6, 2);
  TopMainMenu.setItemTextMargins(10, 9, 5);
  TopMainMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  TopMainMenu.setTitleTextMargins(50, 13);

  RaceMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 230, 22, 8, "Driver Setup", FONT_14, FONT_16B);
  RaceMenuOption2 = RaceMenu.addNI("Tires", TireID, 0, sizeof(TireText) / sizeof(TireText[0]), 1, 0, TireText);
  RaceMenuOption9 = RaceMenu.addNI("Tire pressure [psi]", TirePressure, 70, 200, 5);
  sprintf(buf, "Motor sprocket (%.2f)", GearRatio);
  RaceMenuOption3 = RaceMenu.addNI(buf, MotorSprocket, 10, 20, 1);
  sprintf(buf, "Wheel sprocket (%.2f)", GearRatio);
  RaceMenuOption4 = RaceMenu.addNI(buf, WheelSprocket, 20, 90, 1);
  RaceMenuOption11 = RaceMenu.addNI("Lap Amp warning [a]", MaxLapAmps, 15, 25, .1, 1);
  RaceMenuOption14 = RaceMenu.addNI("Volt warning [v]", BatWarning, 14, 24, 0.1, 1);
  RaceMenuOption5 = RaceMenu.addNI("Battery energy [whr]", TotalEnergy, 550, 750, 1);
  RaceMenuOption6 = RaceMenu.addNI("Battery 1", Battery1, 1, 99, 1);
  RaceMenuOption7 = RaceMenu.addNI("Battery 2", Battery2, 1, 99, 1);
  RaceMenuOption8 = RaceMenu.addNI("Add lap when pitting", AddLapInPit, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  RaceMenuOption10 = RaceMenu.addNI("Delay GPS start read", StartGPSDelayID, 0, sizeof(GPSReadTimeText) / sizeof(GPSReadTimeText[0]), 1, 0, GPSReadTimeText);
  RaceMenuOption12 = RaceMenu.addNI("GPS trigger range", GPSTolerance, 0, sizeof(GPSToleranceText) / sizeof(GPSToleranceText[0]), 1, 0, GPSToleranceText);
  RaceMenuOption13 = RaceMenu.addNI("GPS lap threshold [s]", LapThreshold, 10, 120, 5);
  RaceMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  RaceMenu.setItemTextMargins(2, 3, 5);
  RaceMenu.setMenuBarMargins(1, 319, 3, 1);
  RaceMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  RaceMenu.setTitleTextMargins(50, 13);

  SettingsMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 230, 22, 8, "Settings", FONT_14, FONT_16B);
  SettingsMenuOption5 = SettingsMenu.addNI("Start/Driver trigger [a]", TriggerAmps, 10, 90, 5, 0);
  SettingsMenuOption1 = SettingsMenu.addNI("Motor", MotorID, 0, sizeof(MotorText) / sizeof(MotorText[0]), 1, 0, MotorText);
  SettingsMenuOption2 = SettingsMenu.addNI("Screen button location", Orientation, 0, sizeof(OrientationText) / sizeof(OrientationText[0]), 1, 0, OrientationText);
  SettingsMenuOption3 = SettingsMenu.addNI("Theme", Theme, 0, sizeof(ThemeText) / sizeof(ThemeText[0]), 1, 0, ThemeText);
  SettingsMenuOption4 = SettingsMenu.addNI("Car", CarID, 0, sizeof(CarText) / sizeof(CarText[0]), 1, 0, CarText);
  SettingsMenuOption6 = SettingsMenu.addNI("Restart display each draw", RestartDisplayAlways, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  SettingsMenuOption7 = SettingsMenu.addNI("Main data text justify", DataJustificationRight, 0, sizeof(TextJustifyText) / sizeof(TextJustifyText[0]), 1, 0, TextJustifyText);

  SettingsMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  SettingsMenu.setItemTextMargins(2, 3, 5);
  SettingsMenu.setMenuBarMargins(1, 319, 3, 1);
  SettingsMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  SettingsMenu.setTitleTextMargins(50, 13);

  WirelessMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 220, 22, 5, "Wireless Setup", FONT_14, FONT_16B);
  WirelessMenuOption1 = WirelessMenu.addNI("Send time [s]", RadioUpdate, 0, sizeof(SendTimeText) / sizeof(SendTimeText[0]), 1, 0, SendTimeText);
  WirelessMenuOption2 = WirelessMenu.addNI("Channel", RadioChannel, 0, 69, 1);
  WirelessMenuOption3 = WirelessMenu.addNI("Data rate", AirDataRate, 0, sizeof(AirRateText) / sizeof(AirRateText[0]), 1, 0, AirRateText);
  WirelessMenuOption4 = WirelessMenu.addNI("Radio power [dB]", RadioPower, 0, sizeof(PowerText) / sizeof(PowerText[0]), 1, 0, PowerText);
  WirelessMenuOption5 = WirelessMenu.addNI("Altitude correction [ft]", AltCorrection, -400, 400, 5, 0);
  WirelessMenuOption6 = WirelessMenu.addNI("GPS Altitude corr. [ft]", GPSAltCorrection, -400, 400, 5, 0);
  WirelessMenuOption7 = WirelessMenu.addNI("RESET WIRELESS", ResetEBYTE, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  WirelessMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  WirelessMenu.setItemTextMargins(2, 3, 5);
  WirelessMenu.setMenuBarMargins(1, 319, 3, 1);
  WirelessMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  WirelessMenu.setTitleTextMargins(50, 13);

  SensorMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 230, 22, 5, "Sensor Setup", FONT_14, FONT_16B);
  SensorMenuOption1 = SensorMenu.addNI("Volt slope (11.0)", VoltageSlope, 8.0, 12.0, 0.001, 3, NULL);
  SensorMenuOption2 = SensorMenu.addNI("Volt offset (0.32)", VoltageOffset, 0.20, 0.40, 0.001, 3, NULL);
  SensorMenuOption3 = SensorMenu.addNI("Amp slope (20 | 40)", mVPerAmp, 10, 90.0, 0.01, 2);
  SensorMenuOption4 = SensorMenu.addNI("Amp offset (0.5 | 0.6)", VMid, 0.4, 0.8, 0.001, 3);
  SensorMenuOption5 = SensorMenu.addNI("Temp. Motor [ohm]", ThermResMotor, 7000.0, 15000.0, 10, 0);
  SensorMenuOption6 = SensorMenu.addNI("Temp. AUX [ohm]", ThermResAux, 7000.0, 15000.0, 10, 0);
  SensorMenuOption7 = SensorMenu.addNI("Temp. amb. (offset)", AmbTempCF, -20.0, 20.0, .1, 1);
  SensorMenuOption8 = SensorMenu.addNI("Temp warning [f]", TempWarning, 70, 160, 5, 0);
  SensorMenuOption9 = SensorMenu.addNI("Speed sensor pickups", Pickups, 1, 50, 1);
  SensorMenuOption10 = SensorMenu.addNI("Zero current at startup", AutoCurrentCal, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  SensorMenuOption11 = SensorMenu.addNI("Enable Air Sensor", EnableAirFlowSensor, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  SensorMenuOption12 = SensorMenu.addNI("Air Speed Offset", AirSpeedOffset, -10, 10, .05, 2);
  SensorMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  SensorMenu.setItemTextMargins(2, 3, 5);
  SensorMenu.setMenuBarMargins(1, 319, 3, 1);
  SensorMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  SensorMenu.setTitleTextMargins(50, 13);

  GForceMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 200, 22, 4, "Accelerometer Setup", FONT_14, FONT_16B);
  GForceMenuOption7 = GForceMenu.addNI("Auto Calibrate", 0, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  GForceMenuOption2 = GForceMenu.addNI("X calibration", AccelCalX, -8000, 8000, 50, 0);
  GForceMenuOption3 = GForceMenu.addNI("Y calibration", AccelCalY, -8000, 8000, 50, 0);
  GForceMenuOption4 = GForceMenu.addNI("Z calibration", AccelCalZ, -8000, 8000, 50, 0);
  GForceMenuOption8 = GForceMenu.addNI("Axis point forward", ASensorDirection, 0, sizeof(ASensorDirectionText) / sizeof(ASensorDirectionText[0]), 1, 0, ASensorDirectionText);
  GForceMenuOption1 = GForceMenu.addNI("G-Force range", GForceRange, 0, sizeof(AccelFSRange) / sizeof(AccelFSRange[0]), 1, 0, AccelFSRange);
  GForceMenuOption5 = GForceMenu.addNI("Low-pass filter", AccelLPFilter, 0, sizeof(AccelLPFilterText) / sizeof(AccelLPFilterText[0]), 1, 0, AccelLPFilterText);
  GForceMenuOption6 = GForceMenu.addNI("High-pass filter", AccelHPFilter, 0, sizeof(AccelHPFilterText) / sizeof(AccelHPFilterText[0]), 1, 0, AccelHPFilterText);
  GForceMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  GForceMenu.setItemTextMargins(2, 3, 5);
  GForceMenu.setMenuBarMargins(1, 319, 3, 1);
  GForceMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  GForceMenu.setTitleTextMargins(50, 13);

  ClockMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 230, 22, 4, "Clock Setup", FONT_14, FONT_16B);
  ClockMenuOption1 = ClockMenu.addNI("Year", years, 2020, 2040, 1);
  ClockMenuOption2 = ClockMenu.addNI("Month", months, 1, 12, 1);
  ClockMenuOption3 = ClockMenu.addNI("Day", days, 1, 31, 1);
  ClockMenuOption4 = ClockMenu.addNI("Hour", hours, 0, 23, 1);
  ClockMenuOption5 = ClockMenu.addNI("Minute", minutes, 0, 60, 1);
  ClockMenuOption6 = ClockMenu.addNI("Disable menu auto exit", EnableAutoExit, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  ClockMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  ClockMenu.setItemTextMargins(2, 3, 5);
  ClockMenu.setMenuBarMargins(1, 319, 3, 1);
  ClockMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  ClockMenu.setTitleTextMargins(50, 13);

  CyborgMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 215, 22, 8, "Cyborg Setup", FONT_14, FONT_16B);
  CyborgMenuOption2 = CyborgMenu.addNI("Enable Cyborg", EnableCyborg, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  CyborgMenuOption9 = CyborgMenu.addNI("Prediction Avg [s]", AveragingUpdateTime, 10, 180, 1, 0);
  CyborgMenuOption10 = CyborgMenu.addNI("Prediction Comp [A]", PredictionCompensation, 10, 180, 1, 0);
  CyborgMenuOption11 = CyborgMenu.addNI("Limit bias tolerance [%]", LimitTolerance, 0, 10, 0.1, 1);
  CyborgMenuOption1 = CyborgMenu.addNI("Race current [A]", CyborgFirstLimit, 8.0, 50.0, 0.1, 1);
  CyborgMenuOption8 = CyborgMenu.addNI("Turbo current [A]", CyborgSecondLimit, 8.0, 50.0, 0.1, 1);
  CyborgMenuOption6 = CyborgMenu.addNI("Screen updates [ms]", CyborgUpdateTime, 100, 500, 50, 0);
  CyborgMenuOption7 = CyborgMenu.addNI("Cyborg on at [V]", CyborgThresholdVolts, 3, 3.3, 0.1, 1);
  CyborgMenuOption3 = CyborgMenu.addNI("Kp (rise time)", Kp, 0, MAX_KP, .1, 1);
  CyborgMenuOption4 = CyborgMenu.addNI("Ki (converge rate)", Ki, 0, MAX_KI, 1, 0);
  CyborgMenuOption5 = CyborgMenu.addNI("Kd (future prediction)", Kd, 0, MAX_KD, .1, 1);
  CyborgMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  CyborgMenu.setItemTextMargins(2, 3, 5);
  CyborgMenu.setMenuBarMargins(1, 319, 3, 1);
  CyborgMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  CyborgMenu.setTitleTextMargins(50, 13);

  SSDMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, 35, 3, "Data Storage Options", FONT_16B, FONT_16B);
  SSDMenuOption3 = SSDMenu.addNI("Download all data");
  SSDMenuOption1 = SSDMenu.addNI("ERASE SSD chip");
  SSDMenuOption4 = SSDMenu.addNI("Download race data");
  SSDMenuOption5 = SSDMenu.addNI("Download GPS data");
  SSDMenuOption6 = SSDMenu.addNI("Download settings");
  SSDMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  SSDMenu.setMenuBarMargins(10, 319, 6, 2);
  SSDMenu.setItemTextMargins(10, 9, 5);
  SSDMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  SSDMenu.setTitleTextMargins(50, 13);

  PlaybackMenu.init(MENU_TEXT, MENU_BACK, MENU_HIGHTEXT, MENU_HIGHLIGHT, MENU_SELECTTEXT, MENU_SELECT, 250, 22, 8, "Playback Options", FONT_14, FONT_16B);
  // default number of recordsets to 1 and well update when we draw the menu
  PlayBackOption9 = PlaybackMenu.addNI("Plot Lines (select heat)", RPBRaceLines, 0, 1, 1, 0);
  PlayBackOption1 = PlaybackMenu.addNI("Draw graphs", RPBDrawGraphs, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackOption2 = PlaybackMenu.addNI("Plot volts", RPBPlotVolts, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackOption3 = PlaybackMenu.addNI("Plot amps", RPBPlotAmps, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackOption8 = PlaybackMenu.addNI("Plot lap amps", RPBPlotLapAmps, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackOption4 = PlaybackMenu.addNI("Plot speed", RPBPlotSpeed, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackOption5 = PlaybackMenu.addNI("Plot motor temp", RPBPlotMTemp, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackOption10 = PlaybackMenu.addNI("Plot altitude", RPBPlotAltitude, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackOption6 = PlaybackMenu.addNI("Plot throttle signal", RPBCyborgIn, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackOption7 = PlaybackMenu.addNI("Plot ESC signal", RPBCyborgOut, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);
  PlayBackOption11 = PlaybackMenu.addNI("Plot Predicted Energy", RPBPlotMPEnergy, 0, sizeof(YesNoText) / sizeof(YesNoText[0]), 1, 0, YesNoText);

  PlaybackMenu.setTitleColors(MENU_TITLETEXT, MENU_TITLEBACK);
  PlaybackMenu.setItemTextMargins(2, 3, 5);
  PlaybackMenu.setMenuBarMargins(1, 319, 3, 1);
  PlaybackMenu.setItemColors(C_DKGREY, MENU_HIGHBORDER);
  PlaybackMenu.setTitleTextMargins(50, 13);

  MotorTempG.init(10, 181, 40, 105, 40, 160, 20, "Motor", C_WHITE, back_color, C_MAGENTA, C_DKMAGENTA, back_color, Arial_14, Arial_14);
  MotorTempG.useSegmentBars(true);
  MotorTempG.setBars(18, 5, 1);
  MotorTempG.setSectionColors(C_GREEN, C_YELLOW, C_RED, C_VDKGREY);
  MotorTempG.setSectionSize(TempWarning - 20.0f, TempWarning);
  MotorTempG.setScale(40, 160, 20);

  AuxTempG.init(110, 181, 40, 105, 40, 140, 20, "Auxiliary", C_WHITE, back_color, C_YELLOW, C_DKYELLOW, back_color, Arial_14, Arial_14);
  AuxTempG.useSegmentBars(true);
  AuxTempG.setBars(18, 5, 1);
  AuxTempG.setSectionColors(C_GREEN, C_YELLOW, C_RED, C_VDKGREY);
  AuxTempG.setSectionSize(100, 110);
  AuxTempG.setScale(40, 140, 20);
  AmbTempG.init(220, 181, 40, 105, 40, 100, 10, "Ambient", C_WHITE, back_color, C_CYAN, C_DKCYAN, back_color, Arial_14, Arial_14);
  AmbTempG.useSegmentBars(true);
  AmbTempG.setBars(18, 5, 1);
  AmbTempG.setSectionColors(C_GREEN, C_YELLOW, C_RED, C_VDKGREY);
  AmbTempG.setSectionSize(100, 110);
  AmbTempG.setScale(40, 100, 10);

  LapAverageBar.init(2, 205, 150, 30, 25, 40, 3, "", C_WHITE, C_RED, C_YELLOW, C_GREEN, C_VDKGREY, FONT_16B, FONT_16B);
  LapAverageBar.useSegmentBars(true);
  LapAverageBar.showScale(false);
  LapAverageBar.showTitle(false);
  LapAverageBar.useSegmentBars(true);
  LapAverageBar.setBars(28, 5, 1);

  TRemG.init(10, 235, 45, 160, 0, 100, 1, "Time", C_WHITE, back_color, C_CYAN, C_DKCYAN, back_color, Arial_14, Arial_14);
  TRemG.useSegmentBars(true);
  TRemG.showScale(false);
  TRemG.useSegmentBars(true);
  TRemG.setBars(27, 5, 1);
  TRemG.setSectionColors(C_LTBLUE, C_LTBLUE, C_LTBLUE, C_VDKGREY);
  TRemG.setSectionSize(50, 70);
  TRemG.setScale(0, 100, 1);

  ERemG.init(70, 235, 45, 160, 0, 100, 1, "Energy", C_WHITE, back_color, C_CYAN, C_DKCYAN, back_color, Arial_14, Arial_14);
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
  GraphSpeedID = EnergyG.add("S", GCOLOR_SPEED);
  GraphMTempID = EnergyG.add("Tm", GCOLOR_TEMP);
  GraphAltitudeID = EnergyG.add("Al", GCOLOR_ALT);
  GraphCyborgInID = EnergyG.add("CgI", GCOLOR_CBGIN);
  GraphCyborgOutID = EnergyG.add("CgO", GCOLOR_CBGOUT);
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


void DrawExitTimerProgress(int Val) {

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

  WatchDogTimer(DISABLE_WDT);

  MainMenuOption = 1;

  if (EnableCyborg && (CyborgOutputPWM > 0)) {
    Display.fillScreen(C_BLACK);
    Display.setFont(FONT_24BI);
    Display.setTextColor(C_WHITE, C_BLACK);
    Display.setCursor(30, 50);
    Display.print(F("Shutting down"));
    Display.setCursor(30, 100);
    Display.print(F("Cyborg."));

    Display.drawRect(19, 179, 282, 42, C_DKRED);
    // cyborg could be on so shut the ESC off
    for (int i = CyborgOutputPWM; i >= 0; i -= 20) {
      analogWrite(OUTPUT_PIN, i);
      delay(1);
      // draw progress
      Display.fillRect(20, 180, 280.0f * ((float)i / 4096.0f), 40, C_DKRED);
      // draw blankout
      Display.fillRect((280.0f * ((float)i / 4096.0f)) + 20, 180, 280 - (280.0f * ((float)i / 4096.0f)), 40, C_BLACK);
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

      if (MainMenuOption == MainMenuOption1) {
        Display.fillScreen(C_BLACK);
        ProcessRaceMenu();

      } else if (MainMenuOption == MainMenuOption2) {
        Display.fillScreen(C_BLACK);
        ProcessSettingsMenu();

      } else if (MainMenuOption == MainMenuOption3) {
        Display.fillScreen(C_BLACK);
        ProcessWirelessMenu();

      } else if (MainMenuOption == MainMenuOption4) {
        Display.fillScreen(C_BLACK);
        ProcessSensorMenu();

      } else if (MainMenuOption == MainMenuOption5) {
        Display.fillScreen(C_BLACK);
        ProcessClockMenu();

      } else if (MainMenuOption == MainMenuOption6) {
        Display.fillScreen(C_BLACK);
        ProcessSSDMenu();

      } else if (MainMenuOption == MainMenuOption7) {
        Display.fillScreen(C_BLACK);
        ProcessGForceMenu();

      } else if (MainMenuOption == MainMenuOption8) {
        Display.fillScreen(C_BLACK);
        ProcessCyborgMenu();

      } else if (MainMenuOption == MainMenuOption9) {
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
  ClockMenu.SetItemValue(ClockMenuOption6, EnableAutoExit);


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
  uint8_t OldButtonPressed = 0;
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
    if (OldButtonPressed != ButtonPressed) {
      OldButtonPressed = ButtonPressed;
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
      MotorSprocket = RaceMenu.value[RaceMenuOption3];
      WheelSprocket = RaceMenu.value[RaceMenuOption4];
      TireID = RaceMenu.value[RaceMenuOption2];
      GetGearParameters();


      if (ogr != GearRatio) {
        ogr = GearRatio;
        sprintf(buf, "Motor sprocket (%.2f)", GearRatio);
        RaceMenu.setItemText(RaceMenuOption3, buf);
        sprintf(buf, "Wheel sprocket (%.2f)", GearRatio);
        RaceMenu.setItemText(RaceMenuOption4, buf);
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
      MotorSprocket = RaceMenu.value[RaceMenuOption3];
      WheelSprocket = RaceMenu.value[RaceMenuOption4];
      TireID = RaceMenu.value[RaceMenuOption2];
      GetGearParameters();

      if (ogr != GearRatio) {
        ogr = GearRatio;
        sprintf(buf, "Motor sprocket (%.2f)", GearRatio);
        RaceMenu.setItemText(RaceMenuOption3, buf);
        sprintf(buf, "Wheel sprocket (%.2f)", GearRatio);
        RaceMenu.setItemText(RaceMenuOption4, buf);
      }
      ExitStartTime = millis();
    } else if (ButtonPressed == C_BUTTON) {

      PressTimer = 0;
      MenuOption = RaceMenu.selectRow();
      WaitForRelease();
      ExitStartTime = millis();
    }
  }

  TireID = (uint8_t)RaceMenu.value[RaceMenuOption2];
  MotorSprocket = (int)RaceMenu.value[RaceMenuOption3];
  WheelSprocket = (int)RaceMenu.value[RaceMenuOption4];
  TotalEnergy = RaceMenu.value[RaceMenuOption5];
  Battery1 = (uint8_t)RaceMenu.value[RaceMenuOption6];
  Battery2 = (uint8_t)RaceMenu.value[RaceMenuOption7];
  AddLapInPit = (bool)RaceMenu.value[RaceMenuOption8];
  TirePressure = (uint8_t)RaceMenu.value[RaceMenuOption9];
  StartGPSDelayID = (uint8_t)RaceMenu.value[RaceMenuOption10];
  MaxLapAmps = RaceMenu.value[RaceMenuOption11];
  BatWarning = RaceMenu.value[RaceMenuOption14];
  GPSTolerance = (uint8_t)RaceMenu.value[RaceMenuOption12];
  LapThreshold = (uint8_t)RaceMenu.value[RaceMenuOption13];  // LapThreshold, seconds GPS considers a lap

  EEPROM.put(10, MotorSprocket);
  EEPROM.put(20, WheelSprocket);
  EEPROM.put(30, TireID);
  EEPROM.put(35, TirePressure);
  EEPROM.put(70, TotalEnergy);
  EEPROM.put(150, BatWarning);
  EEPROM.put(155, MaxLapAmps);
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

    if (ButtonPressed == L_BUTTON) {
      SettingsMenu.MoveUp();

      delay(100);
      if (!SettingsMenu.isEditing()) {
        WaitForRelease();
      }
      ExitStartTime = millis();
    } else if (ButtonPressed == R_BUTTON) {
      SettingsMenu.MoveDown();
      delay(100);
      if (!SettingsMenu.isEditing()) {
        WaitForRelease();
      }
      ExitStartTime = millis();
    } else if (ButtonPressed == C_BUTTON) {
      MenuOption = SettingsMenu.selectRow();
      WaitForRelease();
      ExitStartTime = millis();
    }
  }

  MotorID = (int)SettingsMenu.value[SettingsMenuOption1];
  Orientation = (uint8_t)SettingsMenu.value[SettingsMenuOption2];
  Theme = (uint8_t)SettingsMenu.value[SettingsMenuOption3];
  CarID = (uint8_t)SettingsMenu.value[SettingsMenuOption4];
  TriggerAmps = (uint8_t)SettingsMenu.value[SettingsMenuOption5];
  RestartDisplayAlways = (bool)SettingsMenu.value[SettingsMenuOption6];
  DataJustificationRight = (uint8_t)SettingsMenu.value[SettingsMenuOption7];

  SetTextJustification();


  EEPROM.put(25, DataJustificationRight);
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
  float GPSAltitude = 0.0f;

  MenuOption = 1;
  ResetEBYTE = 0;

  // get the current parameters
  RadioChannel = Radio.GetChannel();
  AirDataRate = Radio.GetAirDataRate();
  RadioPower = Radio.GetTransmitPower();

  oRadioChannel = RadioChannel;
  oAirDataRate = AirDataRate;
  oRadioPower = RadioPower;

  // reset the menu data
  WirelessMenu.SetItemValue(WirelessMenuOption2, RadioChannel);
  WirelessMenu.SetItemValue(WirelessMenuOption3, AirDataRate);
  WirelessMenu.SetItemValue(WirelessMenuOption4, RadioPower);
  Display.fillRect(0, 158, 319, 101, C_DKGREY);
  // now we are ready to draw
  WirelessMenu.draw();
  WaitForRelease();
  ExitStartTime = millis();
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

    if (ButtonPressed == L_BUTTON) {
      WirelessMenu.MoveUp();
      delay(100);
      if (!WirelessMenu.isEditing()) {
        WaitForRelease();
      }
      ExitStartTime = millis();
    }
    if (ButtonPressed == R_BUTTON) {
      WirelessMenu.MoveDown();
      delay(100);
      if (!WirelessMenu.isEditing()) {
        WaitForRelease();
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

      Altimiter.read();

      AltCorrection = (int16_t)WirelessMenu.value[WirelessMenuOption5];
      GPSAltCorrection = (int16_t)WirelessMenu.value[WirelessMenuOption6];

      Altitude = (Altimiter.altitude() * METERS_TO_FEET) + AltCorrection;
      AtmPressure = Altimiter.pressure();
      AmbTemp = Altimiter.temperature();
      AmbTemp = (AmbTemp * 1.8) + 32.0 + AmbTempCF;

      Display.setTextColor(C_WHITE, C_BLACK);
      Display.setCursor(5, 170);
      Display.print(F("Coordinates"));
      Display.setCursor(5, 190);
      Display.print(F("Start coord"));
      Display.setCursor(5, 210);
      Display.print(F("Alt/GPSAlt/Sat"));

      GPSAltitude = (GPS.altitude.meters() * METERS_TO_FEET) + GPSAltCorrection;

      GPSLat = GPS.location.lat();
      GPSLon = GPS.location.lng();

      GPSSatellites = GPS.satellites.value();
      GPSStatus = GPS.location.isValid();

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
      Display.print(GPSStartLon, 2);
      Display.print(F(" / "));
      Display.print(GPSStartLat, 2);

      Display.setCursor(140, 210);
      Display.print(Altitude, 0);
      Display.print(F(" / "));
      Display.print(GPSAltitude, 0);
      Display.print(F(" / "));
      Display.print(GPSSatellites);

      // force a retest of valid GPS
      GPSStatus = false;
    }
  }

  AltCorrection = (int16_t)WirelessMenu.value[WirelessMenuOption5];
  GPSAltCorrection = (int16_t)WirelessMenu.value[WirelessMenuOption6];
  RadioUpdate = (uint8_t)WirelessMenu.value[WirelessMenuOption1];
  RadioChannel = (uint8_t)WirelessMenu.value[WirelessMenuOption2];
  AirDataRate = (uint8_t)WirelessMenu.value[WirelessMenuOption3];
  RadioPower = (uint8_t)WirelessMenu.value[WirelessMenuOption4];
  ResetEBYTE = (uint8_t)WirelessMenu.value[WirelessMenuOption7];

  // bounds check GPS start delay

  SetupGPS();

  if ((oAirDataRate != AirDataRate) || (oRadioChannel != RadioChannel) || (oRadioPower != RadioPower)) {
    Radio.SetChannel(RadioChannel);
    Radio.SetTransmitPower(RadioPower);
    Radio.SetAirDataRate(AirDataRate);
    Radio.SaveParameters(PERMANENT);
  }

  // save stuff to eeprom
  EEPROM.put(45, AltCorrection);
  EEPROM.put(47, GPSAltCorrection);
  EEPROM.put(60, RadioUpdate);
  if (RadioUpdate > 0) {
    if (!Radio.GetModel() || ResetEBYTE == 1) {
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
        if (Radio.init()) {
          Display.setCursor(20, 200);
          Display.print(F("ResetEBYTE OK"));
          break;
        }
      }
    }
    ExitStartTime = millis();
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
  uint8_t OldButtonPressed = 0;
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

    VoltageSlope = SensorMenu.value[SensorMenuOption1];   // volt slope
    VoltageOffset = SensorMenu.value[SensorMenuOption2];  // volt offset
    mVPerAmp = SensorMenu.value[SensorMenuOption3];       // amp slope
    VMid = SensorMenu.value[SensorMenuOption4];           // amp offset
    ThermResMotor = SensorMenu.value[SensorMenuOption5];  // temp offset
    ThermResAux = SensorMenu.value[SensorMenuOption6];    // temp offset
    AmbTempCF = SensorMenu.value[SensorMenuOption7];      // temp offset
    AirSpeedOffset = SensorMenu.value[SensorMenuOption12];
    Counter++;
    if (RPM.available()) {
      RPMSum = RPMSum + RPM.read();
      RPMCount++;
    }

    if ((millis() - caltime) > 1000) {
      caltime = millis();
      ComputeSpeed();
      ComputeData();

      Altimiter.read();
      Altitude = (Altimiter.altitude() * METERS_TO_FEET) + AltCorrection;
      AtmPressure = Altimiter.pressure();
      AmbTemp = Altimiter.temperature();
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
      Display.print(WRPM);
      Display.setCursor(240, 220);
      Display.print(AirSpeed, 1);

      // need to get volts / amps and display
      Counter = 0;
      vVolts = 0.0f;
      aVolts = 0.0f;
      thmVolts = 0.0f;
      thxVolts = 0.0f;
      WRPM = 0;
      RPMSum = 0;
      RPMCount = 0;
    }

    ButtonPressed = WhatButtonWasPressed();

    if (OldButtonPressed != ButtonPressed) {
      OldButtonPressed = ButtonPressed;
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
      Pickups = (uint8_t)SensorMenu.value[SensorMenuOption9];  // pickups

      if ((MenuOption == SensorMenuOption1) || (MenuOption == SensorMenuOption2)) {
        VoltSensorCalibration = true;
      }
      if ((MenuOption == SensorMenuOption3) || (MenuOption == SensorMenuOption4)) {
        AmpSensorCalibration = true;
      }
      if ((MenuOption == SensorMenuOption5) || (MenuOption == SensorMenuOption6) || (MenuOption == SensorMenuOption7)) {
        TempSensorCalibration = true;
      }
    }
  }

  RPMSum = 0;
  RPMCount = 0;

  VoltageSlope = SensorMenu.value[SensorMenuOption1];      // volt slope
  VoltageOffset = SensorMenu.value[SensorMenuOption2];     // volt offset
  mVPerAmp = SensorMenu.value[SensorMenuOption3];          // amp slope
  VMid = SensorMenu.value[SensorMenuOption4];              // amp offset
  ThermResMotor = SensorMenu.value[SensorMenuOption5];     // temp thermistor voltage divider ext
  ThermResAux = SensorMenu.value[SensorMenuOption6];       // temp thermistor voltage divider int
  AmbTempCF = SensorMenu.value[SensorMenuOption7];         // temp offset
  TempWarning = SensorMenu.value[SensorMenuOption8];       // temp warning
  Pickups = (uint8_t)SensorMenu.value[SensorMenuOption9];  // pickups
  AutoCurrentCal = (bool)SensorMenu.value[SensorMenuOption10];
  EnableAirFlowSensor = (bool)SensorMenu.value[SensorMenuOption11];
  AirSpeedOffset = SensorMenu.value[SensorMenuOption12];

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
  EEPROM.put(140, TempWarning);
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
  uint8_t OldButtonPressed = 0;
  uint16_t SensorCalibrationDate = 0;
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

    if (OldButtonPressed != ButtonPressed) {
      OldButtonPressed = ButtonPressed;
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
      EnableAutoExit = (int)ClockMenu.value[ClockMenuOption6];
      ExitStartTime = millis();
    }
  }

  years = (int)ClockMenu.value[ClockMenuOption1];
  months = (int)ClockMenu.value[ClockMenuOption2];
  days = (int)ClockMenu.value[ClockMenuOption3];
  hours = (int)ClockMenu.value[ClockMenuOption4];
  minutes = (int)ClockMenu.value[ClockMenuOption5];
  EnableAutoExit = (int)ClockMenu.value[ClockMenuOption6];

  seconds = 1;

  setTime(hours, minutes, seconds, days, months, years);

  Teensy3Clock.set(now());
}

void ProcessGForceMenu() {
  uint8_t OldButtonPressed = 0;
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
    if (OldButtonPressed != ButtonPressed) {
      OldButtonPressed = ButtonPressed;
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
      ASensorDirection = (uint8_t)GForceMenu.value[GForceMenuOption8];  //direction ID
      if (GForceMenu.value[GForceMenuOption7] == 1) {
        AccelSensorCalibration = true;
        CalibrateAccererometer();
        GForceMenu.SetItemValue(GForceMenuOption7, 0);
        GForceMenu.drawRow(GForceMenuOption7);
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
      } else if (MenuOption == GForceMenuOption1) {
        GForceRange = (int16_t)GForceMenu.value[GForceMenuOption1];
        SetupAccelerometer();
      } else if (MenuOption == GForceMenuOption2) {
        AccelCalX = (int16_t)GForceMenu.value[GForceMenuOption2];
        SetupAccelerometer();
      } else if (MenuOption == GForceMenuOption3) {
        AccelCalY = (int16_t)GForceMenu.value[GForceMenuOption3];
        SetupAccelerometer();
      } else if (MenuOption == GForceMenuOption4) {
        AccelCalZ = (int16_t)GForceMenu.value[GForceMenuOption4];
        SetupAccelerometer();
      }
      ExitStartTime = millis();
    }
    if (ASensorDirection == 0) {
      // +X
      ay = ASensor.getAccelerationX() / ASensorBits;
      ax = ASensor.getAccelerationY() / ASensorBits;
      az = ASensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 1) {
      // -X
      ay = -(ASensor.getAccelerationX()) / ASensorBits;
      ax = ASensor.getAccelerationY() / ASensorBits;
      az = ASensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 2) {
      // +Y
      ax = ASensor.getAccelerationX() / ASensorBits;
      ay = -(ASensor.getAccelerationY()) / ASensorBits;
      az = ASensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 3) {
      // -Y
      ax = ASensor.getAccelerationX() / ASensorBits;
      ay = ASensor.getAccelerationY() / ASensorBits;
      az = ASensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 4) {
      // +Z
      ax = ASensor.getAccelerationX() / ASensorBits;
      az = ASensor.getAccelerationY() / ASensorBits;
      ay = ASensor.getAccelerationZ() / ASensorBits;
    } else if (ASensorDirection == 5) {
      // -Z
      ax = ASensor.getAccelerationX() / ASensorBits;
      az = -(ASensor.getAccelerationY()) / ASensorBits;
      ay = ASensor.getAccelerationZ() / ASensorBits;
    }

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
  }

  AccelCalX = (int16_t)GForceMenu.value[GForceMenuOption2];         // x cal
  AccelCalY = (int16_t)GForceMenu.value[GForceMenuOption3];         // y cal
  AccelCalZ = (int16_t)GForceMenu.value[GForceMenuOption4];         // z cal
  AccelLPFilter = (uint8_t)GForceMenu.value[GForceMenuOption5];     //LP filter DLPF_CFG
  AccelHPFilter = (uint8_t)GForceMenu.value[GForceMenuOption6];     //HP Filtern DHPF_CFG
  ASensorDirection = (uint8_t)GForceMenu.value[GForceMenuOption8];  //direction ID

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
  uint8_t OldButtonPressed = 0;
  MenuOption = 1;

  CyborgMenu.SetItemValue(CyborgMenuOption1, CyborgFirstLimit);
  CyborgMenu.SetItemValue(CyborgMenuOption8, CyborgSecondLimit);
  CyborgMenu.SetItemValue(CyborgMenuOption3, Kp);
  CyborgMenu.SetItemValue(CyborgMenuOption4, Ki);
  CyborgMenu.SetItemValue(CyborgMenuOption5, Kd);

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
    if (OldButtonPressed != ButtonPressed) {
      OldButtonPressed = ButtonPressed;
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

      ExitStartTime = millis();
    }
  }

  CyborgFirstLimit = CyborgMenu.value[CyborgMenuOption1];
  EnableCyborg = CyborgMenu.value[CyborgMenuOption2];
  CyborgThresholdVolts = CyborgMenu.value[CyborgMenuOption7];
  AveragingUpdateTime = (uint8_t)CyborgMenu.value[CyborgMenuOption9];
  PredictionCompensation = CyborgMenu.value[CyborgMenuOption10];
  LimitTolerance = CyborgMenu.value[CyborgMenuOption11];

  if (EnableCyborg) {
    MaxDisplayID = 8;
  } else {
    MaxDisplayID = 7;
  }

  Kp = (double)CyborgMenu.value[CyborgMenuOption3];
  Ki = (double)CyborgMenu.value[CyborgMenuOption4];
  Kd = (double)CyborgMenu.value[CyborgMenuOption5];

  CyborgUpdateTime = (uint16_t)CyborgMenu.value[CyborgMenuOption6];
  CyborgSecondLimit = CyborgMenu.value[CyborgMenuOption8];

  CyborgPID.SetTunings(Kp, Ki, Kd);

  Setpoint = CyborgFirstLimit;

  EEPROM.put(385, CyborgThresholdVolts);
  EEPROM.put(485, CyborgFirstLimit);
  EEPROM.put(490, EnableCyborg);
  EEPROM.put(492, AveragingUpdateTime);
  EEPROM.put(495, CyborgUpdateTime);
  EEPROM.put(500, Kp);
  EEPROM.put(510, Ki);
  EEPROM.put(520, Kd);
  EEPROM.put(525, LimitTolerance);
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
      } else if (SSDMenu.item == SSDMenuOption1) {
        DrawInfoScreen();
      }
      if (SSDMenu.item > SSDMenuOption1) {
        DrawDownloadInfoScreen();
      }
      WaitForRelease();
      ExitStartTime = millis();
    }
    if (ButtonPressed == R_BUTTON) {
      SSDMenu.MoveDown();
      if (SSDMenu.item == 0) {
        ShowJEDECScreen();
      } else if (SSDMenu.item == SSDMenuOption1) {
        DrawInfoScreen();
      }
      if (SSDMenu.item > SSDMenuOption1) {
        DrawDownloadInfoScreen();
      }
      WaitForRelease();
      ExitStartTime = millis();
    }
    if (ButtonPressed == C_BUTTON) {
      MenuOption = SSDMenu.selectRow();
      WaitForRelease();
      if (MenuOption == SSDMenuOption1) {

        Display.fillRect(0, 160, 320, 80, C_WHITE);
        Display.setFont(FONT_14);
        Display.setTextColor(C_RED, back_color);
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
          Display.setTextColor(C_RED, back_color);
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
          Point = 0;
          Display.setCursor(10, 220);
          Display.print(F("Process complete."));
          ExitStartTime = millis();
        }
        delay(500);
        DrawInfoScreen();
      }
      if (MenuOption == SSDMenuOption3) {  //all
        CurrentRecord = SSD.getCurrentRecord();
        DownloadRaceData(2);
        DownloadGPSData(1);
        DownloadEEPROM();
        DrawDownloadInfoScreen();
        SSD.gotoRecord(CurrentRecord);
        ExitStartTime = millis();
      }
      if (MenuOption == SSDMenuOption4) {
        CurrentRecord = SSD.getCurrentRecord();
        DownloadRaceData(1);
        DrawDownloadInfoScreen();
        SSD.gotoRecord(CurrentRecord);
        ExitStartTime = millis();
      }
      if (MenuOption == SSDMenuOption5) {
        CurrentRecord = SSD.getCurrentRecord();
        DownloadGPSData(1);
        DrawDownloadInfoScreen();
        SSD.gotoRecord(CurrentRecord);
        ExitStartTime = millis();
      }
      if (MenuOption == SSDMenuOption6) {
        DownloadEEPROM();
        DrawDownloadInfoScreen();
        ExitStartTime = millis();
      }
    }

    if (SSDMenu.item > SSDMenuOption1) {
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
  PlaybackMenu.setLimits(PlayBackOption9, 0, MaxRecordSets, 1, 0);

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
      RPBPlotVolts = (bool)PlaybackMenu.value[PlayBackOption2];
      RPBPlotAmps = (bool)PlaybackMenu.value[PlayBackOption3];
      RPBPlotLapAmps = (bool)PlaybackMenu.value[PlayBackOption8];
      RPBPlotSpeed = (bool)PlaybackMenu.value[PlayBackOption4];
      RPBPlotMTemp = (bool)PlaybackMenu.value[PlayBackOption5];
      RPBPlotAltitude = (bool)PlaybackMenu.value[PlayBackOption10];
      RPBCyborgIn = (bool)PlaybackMenu.value[PlayBackOption6];
      RPBCyborgOut = (bool)PlaybackMenu.value[PlayBackOption7];
      RPBPlotMPEnergy = (bool)PlaybackMenu.value[PlayBackOption11];

      if (PlaybackMenu.value[PlayBackOption1] == 1) {
        PlotRaceData();
        PlaybackMenu.SetItemValue(PlayBackOption1, 0);
        PlaybackMenu.drawRow(PlayBackOption1);
        PlaybackMenu.draw();
      }

      if (PlaybackMenu.value[PlayBackOption9] > 0) {
        PlotRaceLines(PlaybackMenu.value[PlayBackOption9]);
        PlaybackMenu.SetItemValue(PlayBackOption9, 0);
        PlaybackMenu.drawRow(PlayBackOption9);
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

  Display.fillScreen(back_color);

  Display.setFont(FONT_14);
  Display.setTextColor(fore_color, back_color);
  Display.setCursor(5, 10);
  Display.print(F("# "));
  Display.print(recID);

  ColStart = 40;
  // kris
  if (RPBPlotVolts) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_VOLTS, back_color);
    Display.print(F("Volts"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Volts");
  }
  if (RPBPlotAmps) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_AMPS, back_color);
    Display.print(F("Amps"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Amps");
  }
  if (RPBPlotLapAmps) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_LAMPS, back_color);
    Display.print(F("LAmps"));
    ColStart = 2 + ColStart + Display.measureTextWidth("LAmps");
  }
  if (RPBPlotSpeed) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_SPEED, back_color);
    Display.print(F("Speed"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Speed");
  }
  if (RPBPlotMTemp) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_TEMP, back_color);
    Display.print(F("Tmp/5"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Tmp/5");
  }
  if (RPBPlotAltitude) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_ALT, back_color);
    Display.print(F("Alt[m]/100"));
    ColStart = 2 + ColStart + Display.measureTextWidth("Alt[m]/100");
  }
  if (RPBCyborgIn) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_CBGIN, back_color);
    Display.print(F("In/4"));
    ColStart = 2 + ColStart + Display.measureTextWidth("In/4");
  }
  if (RPBCyborgOut) {
    Display.setCursor(ColStart, 10);
    Display.setTextColor(GCOLOR_CBGOUT, back_color);
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

  SetAspectRatio(ScreenTop, ScreenBottom, ScreenLeft, ScreenRight);

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

void SetAspectRatio(float Top, float Bottom, float Left, float Right) {
  float AR = 0.0f;
  AR = abs(Top - Bottom) / abs(Left - Right);
  if (AR < 1) {
    PlotWide = 320 - 50;
    PlotHigh = (240 - 30) * AR;
  } else {
    PlotWide = (320 - 50) / AR;
    PlotHigh = (240 - 30);
  }
}

void DownloadRaceData(uint32_t Count) {

  bool SDCardStatus = false, PitStop = false;
  uint8_t temp = 0, next = 0, rt = 0, tWS = 0, tMS = 0, PitCount = 0;
  int16_t OldLap = -1;
  uint32_t i = 0, StatusBarWidth = 0;
  char FileName[27] = "C_RRR_YYYY-MM-DD_NNN.csv";
  uint16_t tPoint = 0;
  bool OKtoClose = false;
  uint32_t HeaderRecord = 0, CurrentRecord = 0;
  float TempEnergy = 0.0f, EnergyOffset = 0.0f, CurrentEnergy = 0.0f;
  uint32_t SummaryTableLine = 0;
  CurrentRecord = SSD.getCurrentRecord();

  Display.setFont(FONT_14);
  Display.fillRect(10, 167, 300, 25, C_WHITE);

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

  StatusBarCounter = 0;

  for (i = 1; i <= LastRecord; i++) {

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

        FileName[17] = (int)((next / 100) % 10) + '0';
        FileName[18] = (int)((next / 10) % 10) + '0';
        FileName[19] = (int)(next % 10) + '0';

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
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.write(DATA_DELIMITER);
      SDDataFile.write(DATA_DELIMITER);
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
      SDDataFile.print(SSD.getField(AirSpeed, frAirSpeed), 1);  //
      // print the summary block
      if (SummaryTableLine <= 60) {
        SSD.gotoRecord(HeaderRecord);
        // Leave columns between data and header
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.write(DATA_DELIMITER);
        SDDataFile.write(DATA_DELIMITER);
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
            SDDataFile.print(F("Pressure [psi]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(TirePressure, hrTirePressureID));
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
            SDDataFile.print(SSD.getHeaderField(StartAltitude, hrStartAltitude));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(StartAltitude, hrStartAltitude));
            break;
          case 15:
            SDDataFile.print(F("PERFORMANCE STATISTICS"));
            break;
          case 16:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Available Energy [whr]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(TotalEnergy, hrEnergy));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Pit 1 Time [s]"));
            SDDataFile.write(DATA_DELIMITER);
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
            break;
          case 17:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Energy Used [whr]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(J2: J13000)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=AL18 / AL17 * 100"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Pit 2 Time [s]"));
            SDDataFile.write(DATA_DELIMITER);
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
            SDDataFile.print(F("=COUNTIF(X2:X13000, "));
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
            SDDataFile.print(F("Distance [mi]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(N2: N13000)"));
            break;
          case 20:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Official Laps"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(0);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("<-Enter data here"));
            break;
          case 21:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Official Lap Distance [mi]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(0);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("<-Enter data here"));
            break;
          case 22:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Actual Distance [mi]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= AL21 * AL22"));
            break;
          case 23:
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
          case 24:
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
          case 25:
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
          case 26:
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
          case 27:
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
          case 28:
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
          case 29:
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
          case 30:
            // get just the first 5 minutes as we really only need 1 lap
            // otherwise we get wrong readings as pressure changes over 90 min
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Altitude Max [ft]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(Q2:Q600)"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Altitude Min [ft]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=MIN(Q2:Q600)"));

            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Altitude Change [ft]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=AL31 - AN31"));
            break;
          case 32:
            SDDataFile.print(F("DRIVER STATISTICS"));
            break;

          case 33:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Driver,1,2,3"));  // driver 1 ID is actually 0, but to make easier for the data analysist driver 1 = 1
            break;

          case 34:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Time [min]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                 // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=COUNTIF(D2:D13000,AL34)/120"));  // driver time
            SDDataFile.write(34);                                 // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                 // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=COUNTIF(D2:D13000,AM34)/120"));  // driver time
            SDDataFile.write(34);                                 // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                 // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=COUNTIF(D2:D13000,AN34)/120"));  // driver time
            SDDataFile.write(34);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Total [min]:"));  // driver time
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=AL35+AM35+AN35"));  // driver time
            break;
          case 35:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Energy [whr]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                        // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(J2:J13000, D2:D13000, AL34)"));  // driver time //MAXIFS(A2:A7,B2:B7,1)
            SDDataFile.write(34);                                        // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                             // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(J2:J13000, D2:D13000, AM34)-AL36"));  // driver time
            SDDataFile.write(34);                                             // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                                  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(J2:J13000, D2:D13000, AN34)-AL36-AM36"));  // driver time
            SDDataFile.write(34);                                                  // write end " to keep comma in formula
            break;
          case 36:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Laps"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                        // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(C2:C13000, D2:D13000, AL34)"));  // driver time
            SDDataFile.write(34);                                        // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                             // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(C2:C13000, D2:D13000, AM34)-AL37"));  // driver time
            SDDataFile.write(34);                                             // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                                  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(C2:C13000, D2:D13000, AN34)-AL37-AM37"));  // driver time
            SDDataFile.write(34);                                                  // write end " to keep comma in formula
            break;
          case 37:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Distance [mi]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                        // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(N2:N13000, D2:D13000, AL34)"));  // driver time
            SDDataFile.write(34);                                        // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                             // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(N2:N13000, D2:D13000, AM34)-AL38"));  // driver time
            SDDataFile.write(34);                                             // write end " to keep comma in formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);                                                  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=MAXIFS(N2:N13000, D2:D13000, AN34)-AL38-AM38"));  // driver time
            SDDataFile.write(34);                                                  // write end " to keep comma in formula
            break;
          case 38:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Watts / Mile"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AL38>0,AL36/AL38,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AM38>0,AM36/AM38,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AN38>0,AN36/AN38,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            break;
          case 39:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Watts / Minute"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AL35>0,AL36/AL35,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AM35>0,AM36/AM35,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            SDDataFile.print(F("=IF(AN35>0,AN36/AN35,0)"));
            SDDataFile.write(34);  // write a " that way the comma in the formula will stay in the formula
            break;
          case 41:
            SDDataFile.print(F("G-FORCE MEASUREMENTS"));
            break;
          case 42:
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
          case 43:
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
          case 44:
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
          case 45:
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
          case 47:
            SDDataFile.print(F("DATABASE STATISTICS"));
            break;
          case 48:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Race Time [min]"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("= MAX(B2:B13000)"));
            break;
          case 49:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Points Recorded"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("=COUNTA(A2:A13000)"));
            break;
          case 50:
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(F("Datapoint Averages"));
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.write(DATA_DELIMITER);
            SDDataFile.print(SSD.getHeaderField(AverageCounter, hrCounter) * (1000 / UPDATE_LIMIT));
            break;
          case 51:
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
          case 52:
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
  char FileName[31] = "C_RRR_YYYY-MM-DD_NNN_GPS.csv";
  bool OKtoClose = false;

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

      sprintf(FileName, "C_%03d_%04d-%02d-%02d_000_GPS.csv",
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

        FileName[17] = (int)((next / 100) % 10) + '0';
        FileName[18] = (int)((next / 10) % 10) + '0';
        FileName[19] = (int)(next % 10) + '0';

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

  char SetupFileName[28] = "C_EEPROM_YYYY-MM-DD_NNN.txt";
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
    sprintf(buf, "Report date: %d:%02d:%02d, %d/%d/%d", hour() - 12, minute(), second(), month(), day(), year());
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
    SDDataFile.println(F("Total recordsets: 0"));
  } else {
    SDDataFile.print(F("Total recordsets: "));
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

  SDDataFile.println(F("Software versions"));
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
  SDDataFile.println(EBYTE_H_LIB_VER);
  SDDataFile.print(F("MS_5837: "));
  //SDDataFile.println(MS_5837_VERSION);
  SDDataFile.print(F("MCP3208: "));
  SDDataFile.println(MPC3208_VER);
  SDDataFile.print(F("XGZP6897D Address: "));
  SDDataFile.println(I2C_device_address, HEX);


  SDDataFile.println();

  StatusBarWidth = ((float)(6 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

  SDDataFile.println(F("Calibration information"));

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
  SDDataFile.print(F("Altitude correction [ft]: "));
  SDDataFile.println(AltCorrection);
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
  SDDataFile.println(F("CYBORG Setup"));
  SDDataFile.print(F("Cyborg Enabled: "));
  SDDataFile.println(EnableCyborg ? "Yes" : "No");
  SDDataFile.print(F("Prediction sample size: "));
  SDDataFile.println(PREDICT_SAMPLES);
  SDDataFile.print(F("Prediction Averaging Scheme: "));
  SDDataFile.print(F("Current Bias Update Time [s]: "));
  SDDataFile.println(AveragingUpdateTime);
  SDDataFile.print(F("Linear Regression Compensation [A]: "));
  SDDataFile.println(PredictionCompensation, 0);

  // make sure we get the saved one and not any bias applied by Cyborg
  EEPROM.get(485, CyborgFirstLimit);
  SDDataFile.print(F("Cyborg 1st current limit: "));
  SDDataFile.println(CyborgFirstLimit);
  SDDataFile.print(F("Cyborg 2nd current limit: "));
  SDDataFile.println(CyborgSecondLimit);
  SDDataFile.print(F("Cyborg update time: "));
  SDDataFile.println(CyborgUpdateTime);
  SDDataFile.print(F("Cyborg On Threshold: "));
  SDDataFile.println(CyborgThresholdVolts);
  SDDataFile.print(F("Kp Tuning Parameter: "));
  SDDataFile.println(Kp);
  SDDataFile.print(F("Ki Tuning Parameter: "));
  SDDataFile.println(Ki);
  SDDataFile.print(F("Kd Tuning Parameter: "));
  SDDataFile.println(Kd);
  SDDataFile.println();

  StatusBarWidth = ((float)(7 * 293.0) / 10) + 2;
  Display.fillRoundRect(13, 195, StatusBarWidth, 36, 2, C_GREEN);

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
    SDDataFile.print(F("Pressure [psi]: "));
    SDDataFile.println(TirePressure);
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
  SDDataFile.println(F("Race settings information"));
  SDDataFile.print(F("Total Energy: "));
  SDDataFile.println(TotalEnergy);
  SDDataFile.print(F("Battery 1: "));
  SDDataFile.println(Battery1);
  SDDataFile.print(F("Battery 2: "));
  SDDataFile.println(Battery2);
  SDDataFile.print(F("Battery Warning [volts]: "));
  SDDataFile.println(BatWarning);
  SDDataFile.print(F("Lap Amp Warning: "));
  SDDataFile.println(MaxLapAmps);
  SDDataFile.print(F("Temp Warning [deg F]: "));
  SDDataFile.println(TempWarning);
  SDDataFile.println();

  SDDataFile.println(F("Wireless information"));

  sprintf(buf, "0x%02x", Radio.GetModel());
  SDDataFile.print(F("Transceiver Model: "));
  SDDataFile.println(buf);
  SDDataFile.print(F("Transceiver address: "));
  SDDataFile.println((Radio.GetAddressH() << 8) | (Radio.GetAddressL()));
  SDDataFile.print(F("Transceiver air data rate: "));

  if (Radio.GetAirDataRate() < ((sizeof(AirRateText) / sizeof(AirRateText[0])))) {
    SDDataFile.println(AirRateText[Radio.GetAirDataRate()]);
  }

  SDDataFile.print(F("Transceiver radio power: "));
  if (Radio.GetTransmitPower() < ((sizeof(PowerText) / sizeof(PowerText[0])))) {
    SDDataFile.println(PowerText[Radio.GetTransmitPower()]);
  }

  SDDataFile.print(F("Transceiver channel: "));
  SDDataFile.println(Radio.GetChannel());
  SDDataFile.print(F("Transceiver options DEC / BIN: "));
  SDDataFile.print(Radio.GetOptions());
  SDDataFile.print(F(" / "));
  SDDataFile.println(Radio.GetOptions(), BIN);
  SDDataFile.print(F("Transceiver speed DEC / BIN: "));
  SDDataFile.print(Radio.GetSpeed());
  SDDataFile.print(F(" / "));
  SDDataFile.println(Radio.GetSpeed(), BIN);
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
    back_color = C_BLACK;
    fore_color = C_WHITE;
  } else {
    // light, white background
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

  if (EnableCyborg) {
    MaxDisplayID = 8;
  } else {
    MaxDisplayID = 7;
  }

  if (DisplayID > MaxDisplayID) {
    DisplayID = 0;
  }

  EnergyG.setColors(fore_color, C_DKGREY, C_BLUE, back_color, back_color);
  TRemG.setColors(fore_color, back_color, C_BLUE, back_color, back_color);
  ERemG.setColors(fore_color, back_color, C_BLUE, back_color, back_color);
  MotorTempG.setColors(fore_color, back_color, C_BLUE, back_color, back_color);
  AuxTempG.setColors(fore_color, back_color, C_BLUE, back_color, back_color);
  AmbTempG.setColors(fore_color, back_color, C_BLUE, back_color, back_color);

  RedrawHeader = true;
}



void BuildDateStringMS(unsigned long val) {

  val = val / 1000;
  hr = (int)(val / 3600);
  mn = (int)((val - (hr * 3600)) / 60);
  sc = (int)(val % 60);
  sprintf(buf, "%02d:%02d:%02d", hr, mn, sc);
}

void BuildDateStringS(unsigned long val) {

  hr = (int)(val / 3600);
  mn = (int)((val - (hr * 3600)) / 60);
  sc = (int)(val % 60);
  sprintf(buf, "%02d:%02d:%02d", hr, mn, sc);
}

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

  // you cannot have more header fields that data fields
  // header fields can be any type
  // headers are simply the first record and can hold setup data (start time for example) to be printed later
  hrType = SSD.addHeaderField(&RecordType);              // "Record Type" 1
  hrID = SSD.addHeaderField(&RecordSETID);               // "Recordset ID" 2
  hrYear = SSD.addHeaderField(&Tyear);                   // "Year" 2
  hrMonth = SSD.addHeaderField(&Tmonth);                 // "Month" 2
  hrDay = SSD.addHeaderField(&Tday);                     // "Day" 2
  hrHour = SSD.addHeaderField(&Thour);                   // "Hour" 2
  hrMinute = SSD.addHeaderField(&Tminute);               // "Minute" 2
  hrMSprocket = SSD.addHeaderField(&MotorSprocket);      // "Motor Sprocket" 1
  hrWSprocket = SSD.addHeaderField(&WheelSprocket);      // "Wheel Sprocket" 1
  hrTirePressureID = SSD.addHeaderField(&TirePressure);  // "Tire Pressure" 1
  hrMotorID = SSD.addHeaderField(&MotorID);              // "Motor ID" 1
  hrTemp = SSD.addHeaderField(&AmbTemp);                 // "Amb Temp" 4
  hrStartAltitude = SSD.addHeaderField(&StartAltitude);  // "Start Alt" 4
  hrStartPressure = SSD.addHeaderField(&AtmPressure);    // "Start pressure"
  hrEnergy = SSD.addHeaderField(&TotalEnergy);           // "Energy"
  hrCounter = SSD.addHeaderField(&AverageCounter);       // "Counter"
  hrBattery1 = SSD.addHeaderField(&Battery1);            // "Battery 1"
  hrBattery2 = SSD.addHeaderField(&Battery2);            // "Battery 2"
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


  ASensor.setZeroMotionDetectionThreshold(2);
  ASensor.setDLPFMode(AccelLPFilter);
  ASensor.setDHPFMode(AccelHPFilter);
  ASensor.setFullScaleGyroRange((MPU6050_IMU::MPU6050_GYRO_FS_1000));
  ASensor.setFullScaleAccelRange(GForceRange);
  ASensor.setXAccelOffset(AccelCalX);
  ASensor.setYAccelOffset(AccelCalY);
  ASensor.setZAccelOffset(AccelCalZ);

  ASensorBits = AccelFSBits[GForceRange];

  delay(50);
}

void SetupGPS() {
  // bounds check GPS start delay
  if (StartGPSDelayID < ((sizeof(GPSReadTimeText) / sizeof(GPSReadTimeText[0])))) {
    StartGPSDelay = GPSReadTime[StartGPSDelayID];
  } else {
    StartGPSDelay = 0;
  }
}

/*---------------------------------------------------------*/
//INITIALIZATION
/*---------------------------------------------------------*/

void InitializeSensors() {

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

  ASensor.initialize();
  GForceStatus = ASensor.isConnected();
  delay(50);
  SetupAccelerometer();
  Display.setTextColor(C_GREEN);

  if (!GForceStatus) {
    Display.setTextColor(C_RED);
    Display.print(F("Fail: "));
    Warnings = Warnings | GFORCE_WARNING;
  }
  ax = ASensor.getAccelerationX() / ASensorBits;
  ay = ASensor.getAccelerationY() / ASensorBits;
  az = ASensor.getAccelerationZ() / ASensorBits;

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
    //delay(1);
  }
  vVolts = vVolts / 50.0f;
  vVolts = vVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);
  Volts = (vVolts * VoltageSlope) + VoltageOffset;

  Display.setCursor(STATUS_RESULT, 80);

  if ((Volts < BatWarning) || (Volts > 30.0f)) {
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
    SensorMenu.SetItemValue(SensorMenuOption4, VMid);
    Display.setTextColor(C_CYAN);
  } else {
    aVolts = 0;
    for (i = 0; i < 50; i++) {
      aVolts = aVolts + EXTADC.analogRead(EXTADC_AM_PIN);
      delay(10);
    }
    aVolts = aVolts / 50.0f;
    aVolts = aVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);
    SensorMenu.SetItemValue(SensorMenuOption4, VMid);
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
    //delay(10);
  }
  // test Motor temp sensor
  thmVolts = thmVolts / 50;
  thmVolts = thmVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);
  ThermistorResistence = (thmVolts * ThermResMotor) / (REFERENCE_VOLTAGE - thmVolts);
  TempK = 1.0 / (NTC_A + (NTC_B * (log(ThermistorResistence / 10000.0f))) + (NTC_C * pow(log(ThermistorResistence / 10000.0f), 2)) + (NTC_D * pow(log(ThermistorResistence / 10000.0f), 3)));
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
  thxVolts = 0;
  for (i = 0; i < 50; i++) {
    thxVolts = thxVolts + EXTADC.analogRead(EXTADC_THX_PIN);
    //delay(10);
  }
  thxVolts = thxVolts / 50;
  thxVolts = (EXTADC.analogRead(EXTADC_THX_PIN));
  thxVolts = thxVolts / (EXADC_BIT_CONVERSION / EXADC_VREF);
  ThermistorResistence = (thxVolts * ThermResAux) / (REFERENCE_VOLTAGE - thxVolts);
  TempK = 1.0 / (NTC_A + (NTC_B * (log(ThermistorResistence / 10000.0f))) + (NTC_C * pow(log(ThermistorResistence / 10000.0f), 2)) + (NTC_D * pow(log(ThermistorResistence / 10000.0f), 3)));
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

  AltimiterStatus = Altimiter.init();

  if (!AltimiterStatus) {
    Display.setTextColor(C_RED);
    Display.print(F("!ALT"));
  } else {
    Display.setTextColor(C_GREEN);
    Altimiter.setFluidDensity(997);
    Altimiter.read();
    Altitude = (Altimiter.altitude() * METERS_TO_FEET) + AltCorrection;
    AtmPressure = Altimiter.pressure();
    AmbTemp = Altimiter.temperature();
    AmbTemp = (AmbTemp * 1.8) + 32.0 + AmbTempCF;
    Display.print(F(" / "));
    Display.print(AmbTemp, 0);
    Display.print(F(" Alt. OK"));
  }

  Display.setCursor(STATUS_RESULT, 120);
  digitalWrite(LAPLED_PIN, HIGH);

  // test GPS
  GPSLapTimer = 0;
  // alert pit that it's time to test GPS

  GPSStatus = false;
  while (GPSLapTimer < 5000) {  // test for 5 sec 5000

    GPSRead();
    GPSLat = GPS.location.lat();
    GPSLon = GPS.location.lng();
    GPSStatus = GPS.location.isValid();
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

  // Setup speed sensor
  RPMStatus = RPM.begin(RPM_PIN);

  Display.setCursor(STATUS_RESULT, 140);

  if (!RPMStatus) {
    Display.setTextColor(C_RED);
    Display.print(F("!RPM / "));
    Warnings = Warnings | SPEED_FAIL;
  } else {
    Display.setTextColor(C_GREEN);
    Display.print(F("RPM OK / "));
  }

  if (EnableAirFlowSensor) {

    AirSpeed = 0;
    AirFlowSensorFound = false;
    if (ASSensor.begin()) {
      AirFlowSensorFound = true;
      if (ASSensor.readSensor(ASTemp, ASPressure)) {
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
      Warnings = Warnings | SPEED_FAIL;
    }
  } else {
    Display.setTextColor(C_RED);
    Display.print(F("!AFS"));
  }

  // test transceiver
  delay(100);

  if (RadioUpdate != 0) {
    RadioStatus = Radio.init(1);
    DataPacket.begin(details(Data), &ESerial);

    //if ((!RadioStatus) || (Radio.GetModel() != 0x44)) {
    if (!RadioStatus) {
      // RestoreEBYTEDefaults();
      Display.setTextColor(C_RED);
      Display.setCursor(STATUS_RESULT, 160);
      Display.print(F("FAIL"));
      Warnings = Warnings | EBYTE_FAIL;
    } else {
      RadioChannel = Radio.GetChannel();
      AirDataRate = Radio.GetAirDataRate();
      RadioPower = Radio.GetTransmitPower();

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

  // get set key statge
  if (EXTADC.analogRead(EXTADC_KEY_PIN) > KEY_ON_LIMIT) {
    KeyState = HIGH;
    OldKeyState = HIGH;
  } else {
    KeyState = LOW;
    OldKeyState = LOW;
  }

  if (KeyState == HIGH) {
    RestoreType = STATUS_OK;
    banner_back = C_DKRED;
  } else if (KeyState == LOW) {
    RestoreType = STATUS_PITSTOP;
    banner_back = C_DKGREEN;
    Warnings = Warnings | KEY_OFF;
  }

  OldKeyState = LOW;

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



  // set PID for CYBORG
  Input = 0.0f;
  Setpoint = CyborgFirstLimit;

  CyborgPID.SetMode(AUTOMATIC);
  CyborgPID.SetOutputLimits(800, 4096);
  CyborgPID.SetTunings(Kp, Ki, Kd);

  CyborgFL = EXTADC.analogRead(EXTADC_CYBORGFIRSTLIMIT_PIN) / (EXADC_BIT_CONVERSION / EXADC_VREF);
  CyborgFL = FloatMap(CyborgFL, 0.0, 3.3, 14.0, 24.0);

  CurrentBias = EXTADC.analogRead(EXTADC_CYBORGCURRENTBIAS_PIN) / (EXADC_BIT_CONVERSION / EXADC_VREF);
  CurrentBias = FloatMap(CurrentBias, 0.0, 3.3, 0.0, 0.2);
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
  Teensy3Clock.set(now());
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
    delay(10);  // was 2 Needed so we don't get repeated measures
  }
}

void CalibrateAccererometer() {

  uint16_t StatusBarWidth = 0;
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

  WaitForRelease();

  while (1) {

    Found = 0;

    ASensor.setXAccelOffset(ax_offset);
    ASensor.setYAccelOffset(ay_offset);
    ASensor.setZAccelOffset(az_offset);

    ASensor.setXGyroOffset(gx_offset);
    ASensor.setYGyroOffset(gy_offset);
    ASensor.setZGyroOffset(gz_offset);

    meansensors();

    GForceMenu.value[GForceMenuOption2] = ax_offset;
    GForceMenu.value[GForceMenuOption3] = ay_offset;
    GForceMenu.value[GForceMenuOption4] = az_offset;
    GForceMenu.drawRow(GForceMenuOption2);
    GForceMenu.drawRow(GForceMenuOption3);
    GForceMenu.drawRow(GForceMenuOption4);

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

void DisplayErrors() {

  if (year() < 2025) {
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
    Display.print("ACCELEROMETER");

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

    Display.setCursor(STATUS_RESULT, 220);

    if (ReturnCode == RR_ERROR) {
      Display.setTextColor(C_RED);
      Display.print(F("FAIL to restore"));
      delay(10);
      // consider this now a new race dataset
      Display.setFont(FONT_14);
      Display.setTextColor(C_CYAN);
      Display.setCursor(STATUS_RESULT, 220);
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
  }

  if (RaceStatus == RACE_NOTSTARTED) {
    // either the race hasn't started yet, or an attempt to restore the race failed
    // either way, treat this like it's a new race and move on
    // reset GPS start just in case race did not end naturally
    Display.setFont(FONT_14);
    Display.setTextColor(C_CYAN);
    Display.setCursor(STATUS_RESULT, 200);
    Display.print(F("New Race"));
    Display.setCursor(STATUS_RESULT, 220);
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

  if (DisplayID > MaxDisplayID) {
    DisplayID = 0;
  }
  //main menu trigger
  if (ButtonPressed == L_BUTTON) {

    DisplayID++;
    DriverChangeScreen = 4000;
    if (DisplayID > MaxDisplayID) {
      DisplayID = 0;
    }

    EEPROM.put(320, DisplayID);
    delay(50);
  } else if (ButtonPressed == R_BUTTON) {

    if (DisplayID == 0) {
      DisplayID = MaxDisplayID;
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

  B9_v5.2    Delmont 6/3/2025

  reworked display code got the bar to work

  B10_v1.2    Kasprzak 6/3/2025

  reworked to a new PCB board, external ADC, suppoort for T4.0 or T3.2, more USB inputs, 
  changed barametric pressure sensor, scaled back encryptor to send less data
  
  B10.32.050    Kasprzak 12/3/2025

  added energy prediction calculator, reworked encryptor and database schema

*/



/*
  END OF DATALOGGER CODE
  CODED BY:
  Jacob H., Ben Runyan, JOSHUA C., YASHAS G., THOMAS T, Delmont G.
*/
