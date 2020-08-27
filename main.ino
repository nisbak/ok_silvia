#include <UTFT.h> 
#include <URTouch.h>
#include <RBDdimmer.h>

//==== Creating Objects
UTFT    myGLCD(SSD1289,38,39,40,41); //Parameters should be adjusted to your Display/Schield model
URTouch  myTouch( 6, 5, 4, 3, 2);

//==== Defining Variables
extern uint8_t SmallFont[];
extern uint8_t BigFont[];

int x, y;
char currentPage, selectedUnit;

//==== Flow sensor
const int FLOW_COUNT_INPUT = 13
float mlPerFlowMeterPulse = 0.48f; // 0.42f;// ml/pulse
float mlPerFlowMeterPulsePreInfusion = 0.34f; // 0.42f;// ml/pulse

//==== Pressure sensor
const PRESSURE_SENSOR_INPUT A2 // A2 - 0-5V Pressure Input
const LOW_CALIBRATION_PRESSURE_READING 290 // 10-bit AD reading when low (3.0 bar) pressure is applied (between 0-1023)
const HIGH_CALIBRATION_PRESSURE_READING 788 // 10-bit AD reading when high (9.0 bar) pressure is applied (between 0-1023)
const LOW_CALIBRATION_PRESSURE 30 // x10 in bar - so for 3.0 bar write 30
const HIGH_CALIBRATION_PRESSURE 90 // x10 in bar - so for 9.0 bar write 90

//==== Pump controller
#define outputPin  12 
#define zerocross  5 // for boards with CHANGEBLE input pins

//dimmerLamp dimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
dimmerLamp dimmer(outputPin); //initialase port for dimmer for MEGA, Leonardo, UNO, Arduino M0, Arduino Zero


//************************************************************************
// PID parameters
//************************************************************************
const uint16_t PIDSampleTime = 25; // in mSec
const PRESSURE_AVERAGES 4
const FLOW_AVERAGES 5

// Pressure PID loop (pressure profiling)
double Kpp = 60, Kpi = 20, Kpd = 3; 
const PID_MIN_PRESSURE 4.0
const PID_MAX_PRESSURE 10.0
#define PRESSURE_PID_MIN_PWM 0 
#define PRESSURE_PID_MAX_PWM 220

// Flow PID loop (flow profiling)
double Kfp = 5, Kfi = 5, Kfd = 1;
#define PID_MIN_FLOW 0
#define PID_MAX_FLOW 150  // maximum debit in ml/min while in PI
#define FLOW_PID_MIN_PWM 10
#define FLOW_PID_MAX_PWM 150
#define STALL_FLOW_RATE 20 //ml/min

void setup()

pinMode(FLOW_COUNT_INPUT, INPUT_PULLUP);



//****************************************************************************
// Pull an Espresso - Setup
//***************************************************************************

attachInterrupt(digitalPinToInterrupt(FLOW_COUNT_INPUT), flowPulseReceived, FALLING);
