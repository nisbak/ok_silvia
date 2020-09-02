#include <RBDdimmer.h>
#include <PID_v1.h>
#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h> //NOTE: Has to be edited as per https://github.com/prenticedavid/MCUFRIEND_kbv/issues/9 which changes D2-->D10, D3-->D11
MCUFRIEND_kbv tft;
#include <average.h>
#include "TouchScreen.h" // only when you want to use touch screen 

//==== configure the Analog ports
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin
#define PRESSURE_SENSOR_INPUT A5 // A5 - 0-5V Pressure Input

//==== configure the Digital ports
//Note, LCD uses 0-13
const int FLOW_COUNT_INPUT = 18 //note: must be interrupt pin
const int PumpPwmPin = 15
const int GroupSolenoid = 16

//==== Flow sensor
float mlPerFlowMeterPulse = 0.48f; // 0.42f;// ml/pulse
float mlPerFlowMeterPulsePreInfusion = 0.34f; // 0.42f;// ml/pulse
Average<uint32_t> g_averageF(4);
EIFR = _BV (INTF4); // clear a cached interrupt (not very important)
attachInterrupt(digitalPinToInterrupt(FLOW_COUNT_INPUT), flowPulseReceived, FALLING)

//==== Pressure sensor
const LOW_CALIBRATION_PRESSURE_READING 290 // 10-bit AD reading when low (3.0 bar) pressure is applied (between 0-1023)
const HIGH_CALIBRATION_PRESSURE_READING 788 // 10-bit AD reading when high (9.0 bar) pressure is applied (between 0-1023)
const LOW_CALIBRATION_PRESSURE 30 // x10 in bar - so for 3.0 bar write 30
const HIGH_CALIBRATION_PRESSURE 90 // x10 in bar - so for 9.0 bar write 90
Average<float> g_averageP(6);

//==== Pump controller
//#define zerocross  5 // for boards with CHANGEBLE input pins. Note pin D2 for Mega
//dimmerLamp dimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
dimmerLamp PumpPwm(PumpPwmPin); //initialase port for dimmer for MEGA, Leonardo, UNO, Arduino M0, Arduino Zero

//****************************************************************************
// SETUP
//***************************************************************************

void setup() 
{
	uint16_t ID = tft.readID();
  tft.begin(ID);
  #ifdef DISPLAY_ROTATION
	tft.setRotation(0);
  #endif
  //PumpPwm.begin(NORMAL_MODE, OFF); //dimmer initialisation: name.begin(MODE, STATE) 
  
  pinMode(GroupSolenoid, OUTPUT);
	pinMode(FLOW_COUNT_INPUT, INPUT_PULLUP);
  

  
	attachInterrupt(digitalPinToInterrupt(FLOW_COUNT_INPUT), flowPulseReceived, FALLING);

	Serial.begin(115200);
	Serial.println("OK Silvia is starting....");

	pinMode(GroupSolenoid, OUTPUT);
	pinMode(FLOW_COUNT_INPUT, INPUT_PULLUP);

//****************************************************************************
// Pull an Espresso - Setup
//***************************************************************************

  


