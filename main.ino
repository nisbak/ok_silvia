#include <RBDdimmer.h>
#include <PID_v1.h>
#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h> //NOTE: Has to be edited as per https://github.com/prenticedavid/MCUFRIEND_kbv/issues/9 which changes D2-->D10, D3-->D11
MCUFRIEND_kbv tft;
#include <average.h>
#include "TouchScreen.h" // only when you want to use touch screen 

//==== configure the Analog ports for the LCD
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

//==== configure other Analog ports for the LCD
#define PRESSURE_SENSOR_INPUT A5 // A5 - 0-5V Pressure Input

//==== configure the Digital ports
//Note, LCD uses 0-13
const int FLOW_COUNT_INPUT = 18 //note: must be interrupt pin
const int PumpPwmPin = 15
const int GroupSolenoid = 16


//*************************************************************************
// Sensors and controllers
//*************************************************************************	
	
//==== Flow sensor
float mlPerFlowMeterPulse = 0.48f; // ml/pulse
float mlPerFlowMeterPulsePreInfusion = 0.34f; // 0.42f;// ml/pulse
Const int STALL_FLOW_RATE = 20
Average<uint32_t> g_averageF(4);
EIFR = _BV (INTF4); // clear a cached interrupt (not very important)
//attachInterrupt(digitalPinToInterrupt(FLOW_COUNT_INPUT), flowPulseReceived, FALLING) - commented out as now under setup

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
const MinPWM = 30

//*************************************************************************
// PID setup 
//*************************************************************************	
	
// Define 2 PID loops - one for pressure and one for flow....
double g_PIDSetpoint_F, g_PIDInput_F, g_PIDOutput_F, g_PIDInput_P, g_PIDOutput_P, g_PIDSetpoint_P;
PID pressurePID(&g_PIDInput_P, &g_PIDOutput_P, &g_PIDSetpoint_P,Kpp,Kpi,Kpd, DIRECT);
PID flowPID(&g_PIDInput_F, &g_PIDOutput_F, &g_PIDSetpoint_F,Kfp,Kfi,Kfd, DIRECT);

// Pressure PID loop (pressure profiling)
double Kpp = 60, Kpi = 20, Kpd = 3; 
#define PID_MIN_PRESSURE 4.0
#define PID_MAX_PRESSURE 10.0
#define PRESSURE_PID_MIN_PWM 0 
#define PRESSURE_PID_MAX_PWM 220

// Flow PID loop (flow profiling)
double Kfp = 5, Kfi = 5, Kfd = 1;
#define PID_MIN_FLOW 0
#define PID_MAX_FLOW 150  // maximum debit in ml/min while in PI
#define FLOW_PID_MIN_PWM 10
#define FLOW_PID_MAX_PWM 150
#define STALL_FLOW_RATE 20 //ml/min

//*************************************************************************
// Interrupt handlers 
//*************************************************************************

void flowPulseReceived(bool preInfusion) // receives flow pulses from flow sensor
{
	g_flowPulseCount++;
	g_flowPulseMillis = millis();
}

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
	
  PumpPwm.begin(NORMAL_MODE, OFF); //dimmer initialisation: name.begin(MODE, STATE) 
  
 

  
	attachInterrupt(digitalPinToInterrupt(FLOW_COUNT_INPUT), flowPulseReceived, FALLING);

	Serial.begin(115200);
	Serial.println("OK Silvia is starting....");

	pinMode(GroupSolenoid, OUTPUT);
	pinMode(FLOW_COUNT_INPUT, INPUT_PULLUP);
	PinMode(PRESSURE_SENSOR_INPUT, INPUT);

	//readSWParametersfromEEPROM(); to be implemented
	//readProfilesfromEEPROM(); to be implemented

//********************************************************************
// Main Loop
//********************************************************************

void loop(void) 
{
	uint32_t lastFlowPulseCount, pullStartTime, pullTimer, lastFlowPulseMillis;
	bool preInfusion = false;
	uint32_t stallTime = mlPerFlowMeterPulse * 60 * 1000 / STALL_FLOW_RATE; // 72mSec if 0.024ml/pulse & STALL_FLOW_RATE = 20ml/min 
	
//****************************************************************************
// Idle - Wait for Serial trigger or Group Solenoid interrupt... 
//***************************************************************************
	while (!g_newPull && !g_activePull)
	{
		//measurePressure() function to be defined
		//measureTemp() /function to be defined
		//displayHomeScreen
		}


//****************************************************************************
// Pull an Espresso - Setup
//***************************************************************************
	
		if (g_newPull && !g_flushCycle && !g_activePull)
	{
		Serial.println("Starting new pull...");
		g_newPull = false; // debounce...
		pullStartTime = millis();

		// Clear flowmeter counters and attach interrupt 
		g_flowPulseCount = 0; 
		g_flowPulseCountPreInfusion = 0;
		lastFlowPulseCount = 0;

		// Attach flowmeter interrupt...
		EIFR = _BV (INTF4); // clear a cached interrupt (not very important)
		attachInterrupt(digitalPinToInterrupt(FLOW_COUNT_INPUT), flowPulseReceived, FALLING);
			
		// Set initial parameters
		countOffCycles = debounceCount;
		profileIndex = 0;
		lastProfileIndex = 1; // force a first screen update
		pumpPWM = 0;
		sumFlowProfile = 0;
		ExtractionPhase = 1;

		// Clear existing averages in the stack...
		g_averageF.clear();
		g_averageP.clear();
			
		// Initialize the different manual and automated pull modes 
		selectPIDbyMode();
		selectandDrawProfilebyMode();

		//if (!g_cleanCycle)
		preInfusion = true; // start in preinfusion mode
	
		ledColor('g');
		lastFlowPulseMillis = millis();
		PumpPwm.setState(ON);
		PumpPwm.setPower(MinPWM);
		g_activePull = true; // Time to rock & roll
	}    

//****************************************************************************
// Pull an Espresso - Percolation
//***************************************************************************
	while (g_activePull)
	{
		
		// Time the pull and calculate the profile index (there are two profile points per second)
		pullTimer = millis() - pullStartTime;
		profileIndex = pullTimer / 500; // index advances in 500mSec steps
		
		// Measure current pressure in boiler and flow rate 
		double currentPressure = measurePressure(); // We need the current pressure for the PID loop and stored for rolling average; and displayed

		// flow measurement processing - capture two volatile variables before they change 
		uint32_t capturedFlowPulseCount = g_flowPulseCount; 
		uint32_t capturedFlowPulseMillis = g_flowPulseMillis;
		
		//Gicar flowmeters have very low pulse rates. To enhance resolution we will calculate the reciprocal of the time between pulses (1/T).
		if (capturedFlowPulseMillis > lastFlowPulseMillis) // If there is a new flow meter pulse send the timing for flow rate calculation
		{	
			if (preInfusion) 
				g_flowPulseCountPreInfusion = capturedFlowPulseCount; 
			g_averageF.push(capturedFlowPulseMillis - lastFlowPulseMillis);
			lastFlowPulseMillis = capturedFlowPulseMillis;
		}
		else
			if(millis() > g_flowPulseMillis + stallTime) // Stall crowbar - should the pull stall - insert escalating time into the Average to egg on the PID... 
				g_averageF.push(millis() - g_flowPulseMillis);
		
		displayPressureandWeight();

		//Flow PID's target is the accumulated number of pulses. However, in profile we store pulses per 500mSec. So we accumulate.
		if (profileIndex != lastProfileIndex)
			sumFlowProfile += g_flowProfile[profileIndex] >> 1;
