#include <avr/sleep.h>

// NOTE: the timing assumes the CPU main clock speed is 8Mhz or 16Mhz
// NOTE: serial port runs at 38400

// TODO:
// -abandon I of PID because P coefficient is so large so errors will be too small to care about?
// -model water xfer coeff getting as high as 40 when at full flow
// -single cup steam mode: lower temperature?
// -track boiler volume (it's full when pump stalls and empties in steam mode at rate of power/LATENT_HEAT_VAPORISATION_100) and fill when needed instead of pulsing the pump
// -model the increase of ambient temperature (as far as at least 70°C!)
// -introduce latency between switching pump off and closing solenoid to reduce need for backflushing
// -replace uses of switchStateNew with logic around activity
// -after inactivity timeout, return to preheat mode? (or reboot?)
// -protect pump from overheating by preventing it from running for too long or on an empty tank (zero flow)
// -if there is a temperature probe problem, use a beep warning code
// -fix single cup mode (I've increased the volume - need to test whether flow rate works)
// -measure mains AC voltage to better control power
// -inject water more gently in steam mode (but how?)
// -guard global variables with cli()?
// -???use push/release vs push/on tests so that buttons can set mode while on
// -???slowly pump water through the system all the time to improve temp stability

// operating parameters
// 4.6ml/s at 92.6°C gives good head but boring taste - very little body and no bitterness
// 4.6ml/s at 94°C gives good head with reasonable taste - bit of a sharp/bitter aftertaste, though
// at 130°C, steam is emitted at a rate of 22g in 30s (measured by weighing jug of cold water before/after)
static const double BREW_SETPOINT = 96.0;       // LJ uses 92.6°C and 9Bar for 20-22s with UNION Revelation
static const double PREHEAT_SETPOINT = 122.0;   // Hot enough to warm the brew head up when we flush (which we'll do down to brew temp).
static const double STEAM_SETPOINT = 140.0;     // Gaggia machine cuts out anywhere from 135°C to 160°C depending on duration, but we can handle that with fresh injection of water
static const double MAX_FLOW_RATE = 10.0;       // ml/s well over what the pump can produce just to be sure
static const double BREW_FLOW_RATE_2CUP = 3.0;  // ml/s
static const double BREW_FLOW_RATE_1CUP = 2.5;  // ml/s half of BREW_FLOW_RATE_2CUP would be good, but it's a little close to the flow meter's min rate
static const double MIN_FLOW_RATE = 2.0;        // ml/s where the flow rate begins to be able to be controlled
static const double BREW_VOLUME_2CUP = 100.0;    // ml
static const double BREW_VOLUME_1CUP = 60.0;    // ml
static const unsigned INACTIVITY_TIMEOUT = 1200;// seconds with no activity before stopping heating activating the buzzer (20 minutes)
static const unsigned long BACKFLUSH_STALL_TIME = 1000000l; // microseconds to wait for while pump is stalled before flushing
static const unsigned long BACKFLUSH_FLUSH_TIME = 2500000l; // microseconds to wait for while pump is flushing before restarting
static const double STEAM_PUMP_PULSE_TEMP = 110.0;// temperature at which pulsing the pump begins in steam mode.
static const double STEAM_PUMP_PULSE_DUTY = 0.08;// duty cycle for pulsing the pump in steam mode.

// physical characteristics of the temperature probe circuit (which allows us to measure from below 0°C to just over 150°C)
// the numbers for RESISTANCE_FACTOR and RESISTANCE_OFFSET come from doing a calibration run measuring known resistances
// against readings on the temperature probe pin - we can do this because we know the relationship is linear
static const double RESISTANCE_FACTOR = (146.7 - 97.9) / (825 - 58);
static const double RESISTANCE_OFFSET = 146.7 - 825 * RESISTANCE_FACTOR;
static const double PT100_A = -0.00005775;
static const double PT100_B = 0.39083;
static const double PT100_C = 100.0;

// characteristics of the temperature controller
// NOTE: we do not model (or factor into calibration) the fact that the ambient temperature in the boiler housing
//       increases over time. BREWHEAD_AMBIENT_XFER_COEFF below is for when machine begins to get hot.
static const double MAX_HEATER_POWER = 1350.0;             // W at 240V (43 Ohm elements in semi-parallel)
static const double SAFE_HEATER_POWER = 1350.0;            // W power beyond which I dare not drive the beast
static const double AMBIENT_TEMPERATURE = 20.0;            // °C this does not need to be exact
static const double RESERVOIR_TEMPERATURE = 20.0;          // °C the reservoir gets heated a little by the machine over time - model this?
static const double LATENT_HEAT_VAPORISATION_100 = 2257.0; // J/g latent heat of vaporisation of water at 100°C
static const double SPEC_HEAT_WATER_100 = 4.216;           // J/g/K specific heat of water at 100°C
static const double SPEC_HEAT_ALUMINIUM = 0.9;             // J/g/K specific heat of aluminium (i.e. boiler shell)
static const double SPEC_HEAT_BRASS = 0.38;                // J/g/K specific heat of brass (i.e. brew head)
static const double BOILER_VOLUME = 100.0;                 // ml volume of water in boiler when full (I measured it)
static const double MASS_BOILER_SHELL = 609.0;             // g mass of the aluminium boiler shell (I measured it)
static const double MASS_BREW_HEAD = 1172.0;               // g mass of the brew head (I measured it)
static const double MASS_PORTAFILTER = 450.0;              // g mass of the brew head (I measured it)
static const double HEAT_CAPACITY_BODY = 395.0;            // J/K heat capacity of the body/housing of the machine (i.e. what is lost once-off during startup)
static const double BREWHEAD_AMBIENT_XFER_COEFF = 0.55;    // W/K power lost from brew head to ambient air
static const double BOILER_WATER_XFER_COEFF_NOFLOW = 14.7; // W/K rate at which boiler shell heats water per °C difference in boiler and water temperature
static const double BOILER_BREWHEAD_XFER_COEFF = 3.6;      // W/K ditto for the brewhead (calculated from measuring brewhead temperature at 60s of full power=14.2K+ambient and boiler temp=67.21K+ambient and rate of change of brewhead temp=0.43K/s)
static const double ELEMENT_SHELL_XFER_COEFF = 14.0;       // W/K rate at which heat transfers from element half of boiler to shell half
static const double BOILER_BODY_XFER_COEFF = 1.8;          // W/K rate at which heat transfers into the body/housing of the machine

// characteristics of the pump controller
static const double PUMP_PID_P = -0.03;         // a low enough value to avoid oscillation even with no back-pressure

// characteristics of the mains sense subsystem
static const uint8_t TEN_MS_ON_TIMER_COUNTER2 = 0.01 * F_CPU / 1024; // 10ms in timer/counter2 ticks

// pin mappings
static const uint8_t SOLENOID_PIN = 10;         // = PB2 = OC1B
static const uint8_t PUMP_POWER_PIN = 3;        // = PD3 = OC2B
static const uint8_t HEATER_POWER_PIN = 9;      // = PB1
static const uint8_t STEAM_SW_PIN = 11;         // = PB3
static const uint8_t PUMP_SW_PIN = 12;          // = PB4
static const uint8_t FLOW_METER_PIN = 18;       // = PC4 = PCINT12
static const uint8_t MAINS_SENSE_PIN = 7;       // = PD7 = AIN1
static const uint8_t AIN1_HYSTERESIS_PIN = 8;   // = PB0
static const uint8_t PIEZO_BUZZER_PIN = 2;      // = PD2
static const uint8_t TEMPERATURE_ADCPIN = 0x00; // = PC0
static const uint8_t BANDGAP_ADCPIN = 0x0e;

// globals - for communication with interrupts
volatile double g_pumpPower = 0.0;
volatile unsigned int g_tcnt2Offset = 0;
volatile double g_mainsHalfCycleLength = TEN_MS_ON_TIMER_COUNTER2;
volatile unsigned long g_lastFlowPulseTime = 0;
volatile double g_flowHalfPeriods[2] = {1.0e6, 1.0e6};
volatile double g_totalVolume = 0.0;
volatile boolean g_mainsZeroCrossed = false;
volatile double g_heaterPower = 0.0;
volatile double g_energyError = 0.0;

double setOcr2a(unsigned char value)
{
    OCR2A = value;

    // linearise g_pumpPower: less than 0.25 and more than 0.7 have no significant effect - in fact flow rate slightly drops beyond 0.7!
    double linearisedPower = g_pumpPower <= 0.0 ? -0.5 : g_pumpPower * 0.45 + 0.25;

    // set OCR2B with respect to OCR2A and g_pumpPower
    OCR2B = (unsigned char)((1.0 - linearisedPower) * OCR2A + 0.5);
}

// invoked at mains voltage zero-crossing
SIGNAL(ANALOG_COMP_vect)
{
    // eliminate noise by ignoring changes to OCR that weren't expected and by putting the known good
    // state of OCR onto AIN1_HYSTERESIS_PIN which pulls MAINS_SENSE_PIN up or down to create hysteresis
    static boolean expectedOcr = true;
    if (!!(ACSR & _BV(ACO)) != expectedOcr)
        return;
    digitalWrite(AIN1_HYSTERESIS_PIN, !expectedOcr);
    // change what we expect OCR to do next
    expectedOcr = !expectedOcr;

    // if this is a zero-crossover heading towards negative (i.e. if OCR is high)
    if (ACSR & _BV(ACO))
    {
        // stop timer/counter2
        unsigned char tccr2b = TCCR2B;
        TCCR2B &= ~_BV(CS22) & ~_BV(CS21) & ~_BV(CS20);

        // if we have arrived here with a pending TIMER2 OVF interrupt, then we need to update g_tcnt2Offset and reset the flag
        if (TIFR2 & _BV(TOV2))
        {
            g_tcnt2Offset += OCR2A + 1;
            TIFR2 |= _BV(TOV2);
        }

        // update our record of the mains cycle length, keeping a running average and rejecting
        // values that are wildly improbable (which could happen if, for example, a false
        // crossover was triggered or the mains was off while the controller was powered by USB)
        unsigned int ticksSinceLastTime = g_tcnt2Offset + TCNT2;
        double halfCycleLength = ticksSinceLastTime / 2.0;
        if (halfCycleLength >= TEN_MS_ON_TIMER_COUNTER2 * 0.5 && halfCycleLength <= TEN_MS_ON_TIMER_COUNTER2 * 1.5)
            g_mainsHalfCycleLength = 0.2 * (ticksSinceLastTime / 2.0) + 0.8 * g_mainsHalfCycleLength;

        // we can't update TCNT2 (it messes everything up - far worse than the docs suggest), so to deal
        // with phase errors we need to extend or shrink the cycle a little until we're in sync

        // work out the difference to apply to OCR2A (requiring TCNT=5 on overflow so PWM pulses switch off before the zero crossover)
        static const int expectedTcnt2 = 5;
        int signedCount = ((int)TCNT2 + OCR2A / 2) % OCR2A - OCR2A / 2;
        int phaseError = signedCount - expectedTcnt2;
        int phaseDiff = phaseError > 10 ? 2 : phaseError > 1 ? 1 : phaseError < -10 ? -2 : phaseError < -1 ? -1 : 0;

        // finally, set OCR2A (and OCR2B)
        setOcr2a(g_mainsHalfCycleLength - 0.5 + phaseDiff);

        // start the offset at -TCNT2 so that g_tcnt2Offset + TCNT2 == 0
        g_tcnt2Offset = -TCNT2;

        // restore timer/counter2 to original working mode
        TCCR2B = tccr2b;
    }
}

// once the AC phase measurement is stabilised, timer2 overflows just about at zero crossover
SIGNAL(TIMER2_OVF_vect)
{
    // for the next half mains cycle the heater is on or off depending on whether we've had enough energy through the system
    // and also depending on which cycle last had power: we need to keep it even for the case where the double power diode trick is used
    static bool thiscycle = true;
    bool on = thiscycle && g_energyError < 0;
    digitalWrite(HEATER_POWER_PIN, on);
    if (!on) thiscycle = !thiscycle;
    g_energyError += ((on ? MAX_HEATER_POWER : 0) - g_heaterPower) * g_mainsHalfCycleLength / F_CPU * 1024;

    // keep track of total count on timer/counter2 even though we've reset at OC2A
    g_tcnt2Offset += OCR2A + 1;

    // ensure that phase difference adjustments last only one cycle
    setOcr2a(g_mainsHalfCycleLength - 0.5);

    // let the rest of the world know we have crossed zero
    g_mainsZeroCrossed = true;
}

boolean readSteamSwitch()
{
    return !digitalRead(STEAM_SW_PIN);
}

boolean readPumpSwitch()
{
    return !digitalRead(PUMP_SW_PIN);
}

// called when ADC interrupt occurs - we need an empty handler so the interrupt vector table can be populated
SIGNAL(ADC_vect)
{
}

int readAdcPin(uint8_t pin)
{
    // set the analog reference to external, select the channel and clear ADLAR
    ADMUX = pin & 0x1f;
    // the bandgap voltage takes some time to stabilise
    if (pin == BANDGAP_ADCPIN)
        delay(1);

    // use external reference voltage
    ADMUX &= ~_BV(REFS0);

    // enable ADC conversion complete interrupt and go to sleep - we'll wake up when there is data
    ADCSRA |= _BV(ADIE);
    set_sleep_mode(SLEEP_MODE_IDLE);

    // now sleep for a low noise measurement
    sleep_mode();
    // make sure the converstion is finished and we didn't wake up early for some reason
    while (ADCSRA & _BV(ADSC));

    ADCSRA &= ~_BV(ADIE);

    // we have to read ADCL first; doing so locks both ADCL
    // and ADCH until ADCH is read.  reading ADCL second would
    // cause the results of each conversion to be discarded,
    // as ADCL and ADCH would be locked when it completed.
    uint8_t low = ADCL;
    uint8_t high = ADCH;

    // combine the two bytes
    return (high << 8) | low;
}

double getTemperature()
{
    // first take a reading of internal 1.1V bandgap pin to confirm there is current in the circuit
    // and AREF pin has a non-zero voltage
    if (readAdcPin(BANDGAP_ADCPIN) > 0x300)
        // probe error so return an unsafe temperature
        return 1000.0;

    // always read temperature at around mains zero crossing to help eliminate noice
    g_mainsZeroCrossed = false;
    while(!g_mainsZeroCrossed);

    // take a number of samples for smoothing
    double avgReading = readAdcPin(TEMPERATURE_ADCPIN);
    delay(1);
    avgReading += readAdcPin(TEMPERATURE_ADCPIN);
    avgReading /= 2;

    // extreme readings probably indicate wiring problems to the probe
    if (avgReading < 5.0)
        // Temperature probe not detected or it's colder than I'm willing to believe
        return -1000.0;

    if (avgReading > 1018.0)
        // Temperature probe not detected or it's hotter than is safe - either way we want an error condition
        return 1000.0;

    double resistance = RESISTANCE_FACTOR * avgReading + RESISTANCE_OFFSET;
    double temperature = (-PT100_B + sqrt(PT100_B*PT100_B - 4.0 * PT100_A * (PT100_C - resistance))) / (2.0 * PT100_A);

    return temperature;
}

double getMlsPerPulse(double pulseRate)
{
    // The flow meter is not linear at low flow rates.  This function returns the ratio of mls
    // to pulses at the given pulse rate.  If the pulse rate is too low, then a value of NaN is
    // returned.

    // The following python code generates the lookup table from experimentally determined
    // relationships between flow and numbers of pulses per ml.  Above the given data, the
    // curve is linear with a factor of 1.925ish pulses per ml.  Below the given data, the
    // pulses cease, so the flow cannot be determined.
/*
import scipy.interpolate
# measured data
pulsesPerMl=(0.74, 1.22, 1.35, 1.55, 1.7, 1.84, 1.925, 1.925)
flowRates=(1.02, 1.14, 1.22, 1.38, 1.61, 2.01, 2.6, 3.0)
# from the known ratio of pulse count to volume at given flow rates, build the related array of pulse rates
pulseRates = tuple([x * y for x, y in zip(pulsesPerMl, flowRates)])
# interpolate (it turns out that interpolating pulses per ml gives a much better curve than interpolating mls per pulse - so we'll invert later)
func = scipy.interpolate.interp1d(pulseRates, pulsesPerMl, kind='quadratic')
xarray = [pulseRates[0] + (pulseRates[-1] - pulseRates[0]) / 20 * x for x in range(21)]
yarray = func(xarray)
print 'static const double pulseRates[] = {%s};' % ', '.join(['%.3f' % x for x in xarray])
print 'static const double mlPerPulse[] = {%s};' % ', '.join(['%.3f' % (1.0/y) for y in yarray])
*/

    static const double pulseRates[] = {0.755, 1.006, 1.257, 1.508, 1.759, 2.010, 2.261, 2.512, 2.763, 3.014, 3.265, 3.516, 3.767, 4.018, 4.269, 4.520, 4.771, 5.022, 5.273, 5.524, 5.775};
    static const double mlPerPulse[] = {0.884, 0.688, 0.591, 0.539, 0.506, 0.484, 0.470, 0.464, 0.466, 0.469, 0.473, 0.477, 0.477, 0.472, 0.467, 0.466, 0.466, 0.469, 0.471, 0.473, 0.475};
    static const int pulseTableLength = sizeof(pulseRates) / sizeof(pulseRates[0]);
    static const double interval = (pulseRates[pulseTableLength - 1] - pulseRates[0]) / (pulseTableLength - 1);

    // interpolate to get flowrate
    int tablePos = (pulseRate - pulseRates[0]) / interval;
    if (tablePos < 0)
        return NAN;
    if (tablePos >= pulseTableLength - 1)
        return mlPerPulse[pulseTableLength - 1];
    return mlPerPulse[tablePos] + (mlPerPulse[tablePos + 1] - mlPerPulse[tablePos]) * (pulseRate - pulseRates[tablePos]) / interval;
}

// PCINT1 is connected to the water flow meter - and the interrupt is called whenever state changes
SIGNAL(PCINT1_vect)
{
    // NOTE: there is a very small risk that there might be just over a multiple of 70ish minutes between
    // pulses, giving and artificial high flow rate due to overflow in micros().  I'm happy to accept the
    // problem because I don't anticipate running the beast for more than an hour.

    // record time now and get the length of the last half-cycle
    unsigned long now = micros();
    double timeSinceLastTick = 1.0 / 1000000.0 * (now - g_lastFlowPulseTime);
    g_lastFlowPulseTime = now;

    // record the length of both the last half-pulses (i.e. one whole cycle)
    g_flowHalfPeriods[0] = g_flowHalfPeriods[1];
    g_flowHalfPeriods[1] = timeSinceLastTick;

    // update the total volume which has been pumped through the system
    double mlsPerPulse = getMlsPerPulse(1.0 / (g_flowHalfPeriods[0] + g_flowHalfPeriods[1]));
    if (!isnan(mlsPerPulse))
    {
        g_totalVolume += mlsPerPulse / 2.0;
        return;
    }

    // if we get here, then flow must recently have started - try doubling the last half cycle's time
    // if that doesn't work, then we'll lose track of a very small quantity and that is okay
    mlsPerPulse = getMlsPerPulse(0.5 / g_flowHalfPeriods[1]);
    if (!isnan(mlsPerPulse))
        g_totalVolume += mlsPerPulse / 2.0;
}

double getFlowRate()
{
    // This function returns the flow rate as long as it is high enough to measure - otherwise it returns zero.
    // Given the usages of the function (controlling flow and testing for a significant flow), returning zero
    // is just as good as returning the actual value.
  
    // work out rate at which we're getting full pulses (from high to high or low to low - we don't care)
    // if the flow is slowing, give a better result by using the time since the last tick instead of the 0th
    // value (the one in the same half-cycle as we're in now)
    uint8_t oldSREG = SREG;
    cli();
    unsigned long now = micros();
    double timeSinceLastTick = 1.0 / 1000000.0 * (now - g_lastFlowPulseTime);
    double pulseRateFull = 1.0 / (max(g_flowHalfPeriods[0], timeSinceLastTick) + g_flowHalfPeriods[1]);
    double pulseRatePart = 0.5 / max(g_flowHalfPeriods[1], timeSinceLastTick);
    SREG = oldSREG;

    // calculate the flow rate and return it
    double mlsPerPulse = getMlsPerPulse(pulseRateFull);
    if (!isnan(mlsPerPulse))
        return mlsPerPulse * pulseRateFull;

    // if we got here, the pulse rate was too low to calculate a flow rate, so we'll try the less reliable
    // information from the most recent half-cycle - if this works, then there have been two pulses
    // since the flow started up after being stopped and we'll briefly return a less accurate number which
    // is nonetheless the very best we know how to manage
    mlsPerPulse = getMlsPerPulse(pulseRatePart);
    if (!isnan(mlsPerPulse))
        return mlsPerPulse * pulseRatePart;

    // if we got here, then the best, most recent information we have indicates a low flow rate - the best
    // we can do is say that flow has stopped
    return 0.0;
}

void switchBuzzer(boolean on)
{
    digitalWrite(PIEZO_BUZZER_PIN, on);
}

void switchSolenoid(boolean on)
{
    digitalWrite(SOLENOID_PIN, on);
}

// set heater element power in watts
void setHeaterPower(double power)
{
    // clip value in watts to range 0.0 to SAFE_HEATER_POWER
    g_heaterPower = min(SAFE_HEATER_POWER, max(0.0, power));
}

void setup()
{
    // all ports are digital inputs with pullups by default (but switch outputs low then disable pullups until the end of the set up)
    PORTB = PORTC = PORTD = 0x00;
    DDRB = DDRC = DDRD = PORTD = 0xff;
    MCUCR |= _BV(PUD);
    DDRB = DDRC = DDRD = PORTD = 0x00;
    PORTB = PORTC = PORTD = 0xff;

    // set up timer/counter2's prescaler to 1024 and mode to Fast PWM with TOP=OCR2A
    // now we get an 8 bit counter which can count up as far as 16.384ms or 32.768ms
    // OCR2A will determine the max (which we'll line up with the mains AC half-cycle)
    // OCR2B will generate a PWM signal on OC2B in inverting mode
    // initially, OCR2A = TEN_MS_ON_TIMER_COUNTER2 and OCR2B = 0xff (i.e. never switch on)
    OCR2A = TEN_MS_ON_TIMER_COUNTER2;
    OCR2B = 0xff;
    TCCR2B |= _BV(CS20);
    TCCR2B |= _BV(CS21);
    TCCR2B |= _BV(CS22);
    TCCR2A |= _BV(WGM20);
    TCCR2A |= _BV(WGM21);
    TCCR2B |= _BV(WGM22);
    TCCR2A |= _BV(COM2B0);
    TCCR2A |= _BV(COM2B1);
    TIMSK2 |= _BV(TOIE2);
    pinMode(PUMP_POWER_PIN, OUTPUT);

    // set up the flow meter to generate a PCINT1 when switching on or off
    PCICR = PCMSK1 = 0x00;
    PCICR |= _BV(PCIE1);
    PCMSK1 |= _BV(PCINT12);

    // set up the mains sense pin to generate an interrupt when passing through bandgap voltage (about 1.25V)
    ADCSRB &= ~_BV(ACME);
    ACSR &= ~_BV(ACD);
    ACSR |= _BV(ACBG);
    ACSR |= _BV(ACIE);
    ACSR &= ~_BV(ACIS0);
    ACSR &= ~_BV(ACIS1);
    DIDR1 |= _BV(AIN1D);

    // disable the digital circuitry on the temperature input pin
    if(TEMPERATURE_ADCPIN == 0x00)
        DIDR0 = _BV(ADC0D);
    if(TEMPERATURE_ADCPIN == 0x01)
        DIDR0 = _BV(ADC1D);
    if(TEMPERATURE_ADCPIN == 0x02)
        DIDR0 = _BV(ADC2D);
    if(TEMPERATURE_ADCPIN == 0x03)
        DIDR0 = _BV(ADC3D);
    if(TEMPERATURE_ADCPIN == 0x04)
        DIDR0 = _BV(ADC4D);
    if(TEMPERATURE_ADCPIN == 0x05)
        DIDR0 = _BV(ADC5D);
    digitalWrite(14 + TEMPERATURE_ADCPIN, 0);

    // set up comms
    Serial.begin(38400);

    // set up steam and pump switch pins - pull them high, the opto-couples will pull them low
    pinMode(STEAM_SW_PIN, INPUT);
    digitalWrite(STEAM_SW_PIN, true);
    pinMode(PUMP_SW_PIN, INPUT);
    digitalWrite(PUMP_SW_PIN, true);

    // set up flow meter pulse-signal pin - bias the pin high because meter supposedly is open collector
    pinMode(FLOW_METER_PIN, INPUT);
    digitalWrite(FLOW_METER_PIN, true);

    // set up mains cycle sense pin - note the pump output will be out of sync for up to about half a second (which
    // we don't mind because it just makes the pump higher power and initially we'll only use the pump for priming)
    pinMode(MAINS_SENSE_PIN, INPUT);
    digitalWrite(MAINS_SENSE_PIN, false);
    pinMode(AIN1_HYSTERESIS_PIN, OUTPUT);
    digitalWrite(AIN1_HYSTERESIS_PIN, false);

    // no power to the heater
    pinMode(HEATER_POWER_PIN, OUTPUT);
    digitalWrite(HEATER_POWER_PIN, false);

    // no power to the solenoid
    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, false);

    // buzzer off
    pinMode(PIEZO_BUZZER_PIN, OUTPUT);
    digitalWrite(PIEZO_BUZZER_PIN, 0);

    // now setup is done, enable pullups
    MCUCR &= ~_BV(PUD);

    // the first ADC result can be off because the reference voltage is being set to a new source, so do a read
    readAdcPin(TEMPERATURE_ADCPIN);

    // wait for mains zero-crossing to stabilise, but don't hold off starting the heating process
    unsigned long now = micros();
    while(micros() - now < 500000)
        digitalWrite(HEATER_POWER_PIN, getTemperature() < BREW_SETPOINT);
    digitalWrite(HEATER_POWER_PIN, false);
}

boolean CalibrationMode()
{
    // test for instruction to enter calibration
    if (!Serial.available()) return false;
    if (Serial.read() != 'C') return false;

    // switch all hardware off
    switchBuzzer(false);
    switchSolenoid(false);
    setHeaterPower(0.0);
    g_pumpPower = 0.0;

    while (Serial.available()) Serial.read();
    Serial.println(F("Please enter a calibration operation:"));
    Serial.println(F("1. Calibrate the temperature probe."));
    Serial.println(F("2. Calibrate the flow meter."));
    Serial.println(F("3. Calibrate the boiler thermodynamics."));
    Serial.println(F("4. Flush boiler with cold water."));
    Serial.println(F("5. Heat to specified temperature then cool."));
    Serial.println(F("X. Exit and return to main program."));
    while (!Serial.available());
    char option = Serial.read();

    if (option == 'X') return true;

    // calibrate the temperature probe
    if (option == '1')
    {
        Serial.println(F("Calibrating temperature probe."));

        // ensure tank full
        Serial.println(F("Ensure the water tank is full and then hit a key."));
        while(!Serial.available());
        while(Serial.available()) Serial.read();

        // flush until temperature stops decreasing
        g_pumpPower = 1.0;
        switchSolenoid(true);
        while (getTemperature() >= 70);
        g_pumpPower = 1.0;
        switchSolenoid(false);

        // request ambient temp
        Serial.println(F("Please enter the ambient temperature."));
        while(Serial.available()) Serial.read();
        Serial.parseFloat();

        // fill boiler
        Serial.println(F("filling boiler"));
        g_pumpPower = 1.0;
        unsigned long start = millis();
        while(millis() - start < 500 || millis() - start < 5000 && getFlowRate() >= 1.2);
        g_pumpPower = 0.0;
        Serial.println(F("filled boiler"));

        // heat slowly until steam comes out (this happens at X)
        // ask for altitude above sea level and current barometric pressure
        double lasttemp = 0.0;
        setHeaterPower(0.4 * SAFE_HEATER_POWER);
        switchSolenoid(true);
        while(Serial.available()) Serial.read();
        while(!Serial.available())
        {
            double temp = 0.0;
            for (int i = 0; i < 128; i++)
                temp += getTemperature();
            temp /= 128.0;
            lasttemp = temp;
            double delta = temp - lasttemp;
            lasttemp = temp;
            Serial.print(temp);
            Serial.print(",");
            Serial.print(delta);
            Serial.println();
            delay(500);
        }
        while (Serial.available()) Serial.read();
        setHeaterPower(0.0);
        switchSolenoid(false);
    }

    // calibrate the flow meter
    if (option == '2')
    {
        Serial.println(F("Calibrating flow meter..."));
    }

    // calibrate the boiler thermodynamics
    if (option == '3')
    {
        Serial.println(F("Calibrating boiler thermodynamics."));

        // ensure tank full
        Serial.println(F("Ensure that: 1. Water tank is full. 2. There is a jug to catch water. 3. Steam valve is closed."));
        Serial.println(F("Then hit a key."));
        while(!Serial.available());
        while(Serial.available()) Serial.read();

        // request ambient temp
        Serial.println(F("Please enter the ambient temperature (and ensure water in the tank is at ambient)."));
        while(Serial.available()) Serial.read();
        while(!Serial.available());
        double ambientTemp = Serial.parseFloat();
        Serial.println(ambientTemp);

        // fill boiler
        Serial.println(F("filling boiler"));
        g_pumpPower = 1.0;
        unsigned long start = millis();
        while(millis() - start < 500 || millis() - start < 5000 && getFlowRate() >= 1.2);
        g_pumpPower = 0.0;
        Serial.println(F("filled boiler"));

        Serial.println(F("Raising temperature to a steady 90 degrees."));

        // we need to track the total energy injected into the boiler and the integral of (temperature - ambient) dt
        double totalEnergy = 0.0;
        double totalTempTimesTime = 0.0;

        // simple PID (without the I and D and low value for P to give lots of stability)
        unsigned long lastTime = micros();
        double initialTemp = getTemperature();
        double temp = initialTemp;
        double tempRate = 1.0;
        double power = 0.0;
        while (true)
        {
            // update smoothed temperature
            double oldTemp = temp;
            temp = temp * 0.9 + getTemperature() * 0.1;
            double deltaTemp = temp - oldTemp;

            // update time
            unsigned long now = micros();
            double deltaTime = (double)(now - lastTime) / 1000000;
            lastTime = now;

            // update integrals
            totalEnergy += power * deltaTime;
            totalTempTimesTime += (temp - ambientTemp) * deltaTime;

            // update smoothed derivative of temperature and exit loop if it is low enough
            tempRate = tempRate * 0.9 + fabs(deltaTemp) / deltaTime * 0.1;
            if (tempRate <= 0.001) break;

            // set the heater power
            power = (90.0 - temp) * 25.0;
            power = min(SAFE_HEATER_POWER, max(0.0, power));
            setHeaterPower(power);

            // report and wait
            Serial.println(temp);
            delay(500);
        }
        setHeaterPower(0.0);

        // calculate boiler parameters
        double powerLossPerDegree = power * SAFE_HEATER_POWER / (temp - ambientTemp);
        double internalEnergyPerDegree = (totalEnergy - totalTempTimesTime * powerLossPerDegree) / (temp - initialTemp);
        double shellEnergyPerDegree = internalEnergyPerDegree - SPEC_HEAT_WATER_100 * BOILER_VOLUME;

        // report boiler parameters
        Serial.print(F("BOILER_AMBIENT_XFER_COEFF (in W/K) = "));
        Serial.println(powerLossPerDegree);
        Serial.print(F("SPEC_HEAT_BOILER (in J/K) = "));
        Serial.println(shellEnergyPerDegree);

        Serial.println(F("Switching to flow measure"));
        setHeaterPower(SAFE_HEATER_POWER);
        g_pumpPower = 1.0;
        switchSolenoid(true);

        double startVolume = g_totalVolume;
        double boilerHeatTransferCoeff = 0.0;
        while (g_totalVolume - startVolume < 1000)
        {
            // update smoothed temperature and flow rate
            temp = temp * 0.9 + getTemperature() * 0.1;
            double flowRate = getFlowRate();

            // calculate heat transfer coefficient from boiler shell to water
            double joulesPerMl = SAFE_HEATER_POWER / flowRate;
            double waterTemp = ambientTemp + joulesPerMl / SPEC_HEAT_WATER_100;
            boilerHeatTransferCoeff = SAFE_HEATER_POWER / (temp - waterTemp);

            Serial.print(temp);
            Serial.print(",");
            Serial.print(waterTemp);
            Serial.print(",");
            Serial.println(boilerHeatTransferCoeff);
            delay(500);
        }

        setHeaterPower(0.0);
        g_pumpPower = 0.0;
        switchSolenoid(false);

        Serial.print(F("BOILER_WATER_XFER_COEFF (in W/K) = "));
        Serial.println(boilerHeatTransferCoeff);
    }

    // flush the boiler with cold water
    if (option == '4')
    {
        // ensure tank full
        Serial.println(F("Ensure that: 1. Water tank is full. 2. There is a jug to catch water."));
        Serial.println(F("Then hit a key."));
        while(!Serial.available());
        while(Serial.available()) Serial.read();

        double startVolume = g_totalVolume;
        g_pumpPower = 1.0;
        switchSolenoid(true);
        while (g_totalVolume - startVolume < 500);
        g_pumpPower = 0.0;
        switchSolenoid(false);
    }

    // heat and then cool, reporting temperature curve
    if (option == '5')
    {
        Serial.println(F("Please enter maximum temperature."));
        while(Serial.available()) Serial.read();
        while(!Serial.available());
        double maxTemp = Serial.parseFloat();
        Serial.println(maxTemp);

        // fill boiler
        Serial.println(F("filling boiler"));
        g_pumpPower = 1.0;
        unsigned long start = millis();
        while(millis() - start < 500 || millis() - start < 5000 && getFlowRate() >= 1.2);
        g_pumpPower = 0.0;
        Serial.println(F("filled boiler"));

        Serial.println(F("Send any character to stop."));
        Serial.println("time,power,temp");

        unsigned long t0 = micros();
        unsigned long tn = t0;
        setHeaterPower(SAFE_HEATER_POWER);
        while(!Serial.available() && getTemperature() < maxTemp)
        {
            while(micros() - tn < 1000000);
            tn += 1000000;
            Serial.print((double)(micros()-t0) / 1000000);
            Serial.print(",");
            Serial.print(SAFE_HEATER_POWER);
            Serial.print(",");
            Serial.println(getTemperature());
        }
        setHeaterPower(0);

        while(Serial.available()) Serial.read();
        while(!Serial.available())
        {
            while(micros() - tn < 1000000);
            tn += 1000000;
            Serial.print((double)(micros()-t0) / 1000000);
            Serial.print(",");
            Serial.print(0);
            Serial.print(",");
            Serial.println(getTemperature());
        }
        while(Serial.available()) Serial.read();
	}
    return true;
}

void loop()
{
    // print hello world message
    Serial.println();
    Serial.println(F("Arduino Gaggia controller: type 'C' to enter calibration mode."));

    // clever trick: on startup, the pump switch tells us if we're in 1 cup mode and the steam switch tells us whether we're in backflush mode
    enum {BREW_1CUP, BREW_2CUP, BACKFLUSH} brewMode = BREW_2CUP;
    brewMode = readPumpSwitch() ? BREW_1CUP : readSteamSwitch() ? BACKFLUSH : BREW_2CUP;
    while(readPumpSwitch()) delay(1);
    while(readSteamSwitch()) delay(1);

    // initial value for temperature, etc.
    double shellTemp = getTemperature();
    double elementTemp = shellTemp;
    double waterTemp = shellTemp;
    // TODO: make brewHeadTemp the correct split between ambient and boiler temp - easy to calculate
    double brewHeadTemp = shellTemp;
    double bodyTemp = shellTemp;
    double pumpPowerRate = 0.0;
    double heaterPower = 0.0;

    // variable to remember whether we've done the one-time flush to heat the system (not needed if already hot at startup or backflushing)
    enum {PREHEAT_FILL, PREHEAT_HEAT, PREHEAT_VENT, PREHEAT_COOL, PREHEAT_DONE} preheatMode = PREHEAT_FILL;
    if (fabs(waterTemp - BREW_SETPOINT) < 5.0 || brewMode == BACKFLUSH)
        preheatMode = PREHEAT_DONE;

    while (true)
    {
        // if necessary, run calibration routines
        if (CalibrationMode()) break;

        // read physical state
        static boolean steamOn = false;
        static boolean pumpOn = false;
        boolean switchStateNew = steamOn != readSteamSwitch() || pumpOn != readPumpSwitch();
        steamOn = readSteamSwitch();
        pumpOn = readPumpSwitch();
        double flowRate = getFlowRate();

        // control variables
        double flowSetpoint = 0.0;
        double tempSetpoint = 0.0;
        static double brewStopVolume = 0.0;

        static unsigned long lastActiveTime = 0;
        if (switchStateNew)
            lastActiveTime = micros();

        // main branch tree code which decides what activity we're doing and sets solenoid, flow, temperature and buzzer accordingly...
        static enum {ACTIVITY_NONE, ACTIVITY_PREHEAT, ACTIVITY_STEAM, ACTIVITY_BACKFLUSH, ACTIVITY_BREW, ACTIVITY_READY} activity = ACTIVITY_NONE;

        // temperature stabilisation routine: drive water through the system once at temperature (but cancel preheat if any button is pressed)
        if (steamOn || pumpOn || brewMode == BACKFLUSH)
            preheatMode = PREHEAT_DONE;
        if (preheatMode != PREHEAT_DONE)
        {
            activity = ACTIVITY_PREHEAT;
            switchBuzzer(false);

            if (preheatMode == PREHEAT_FILL)
            {
                // make sure the boiler is full by pumping against a closed solenoid valve until there is no flow
                switchSolenoid(false);
                flowSetpoint = MAX_FLOW_RATE;
                tempSetpoint = STEAM_SETPOINT;

                // pump for at least half a quarter before checkiong flow rate and then move on if flow is less than 1.2 ml/s
                // NOTE: below 1.2 ml/s the flow meter can give very different results depending on where any bubbles
                // are in the system - indeed, it might never measure below 1 ml/s even with the minor dribble leak from the steam wand
                // TODO: it is an error condition if we pump more than a certain volume or for more than a certain time - how to handle?
                static unsigned long startTime = micros();
                if (micros() - startTime > 250000 && flowRate < 1.2)
                    preheatMode = PREHEAT_HEAT;
            }
            if (preheatMode == PREHEAT_HEAT)
            {
                // Open the solenoid to release pressure and heat up to the system preheat setpoint.
                // Releasing the pressure now is very important - otherwise later on we get massive and
                // unpredictable cooling as water flashes in the low pressure pulse.
                // But don't open the solenoid for too long - we don't want to cause unneccessary cooling by evaporation.
                static unsigned long startTime = micros();
                switchSolenoid(micros() - startTime < 1000000);
                flowSetpoint = 0.0;
                // heat as fast as we can (but don't overshoot the steam setoint)
                tempSetpoint = STEAM_SETPOINT;
                // heat until the water temperature exceeds the setpoint
                if (shellTemp >= PREHEAT_SETPOINT && waterTemp >= 100.0)
                    preheatMode = PREHEAT_VENT;
            }
            if (preheatMode == PREHEAT_VENT)
            {
                switchSolenoid(true);
                flowSetpoint = 0.0;
                tempSetpoint = STEAM_SETPOINT;
                static unsigned long startTime = micros();
                if (micros() - startTime >= 10000000)
                    preheatMode = PREHEAT_COOL;
            }
            if (preheatMode == PREHEAT_COOL)
            {
                switchSolenoid(true);
                flowSetpoint = MIN_FLOW_RATE;
                tempSetpoint = BREW_SETPOINT;
                // stop when total energy of water and boiler is right
                if ((waterTemp - BREW_SETPOINT) * SPEC_HEAT_WATER_100 * BOILER_VOLUME + ((shellTemp + elementTemp) / 2.0 - BREW_SETPOINT) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL <= 0.0)
                    preheatMode = PREHEAT_DONE;
            }
        }
        // steam mode (takes precedence over brew mode unless we're actually in the middle of an already started brew)
        else if (steamOn && !(pumpOn && g_totalVolume <= brewStopVolume))
        {
            activity = ACTIVITY_STEAM;
            switchBuzzer(false);

            // set the temperature to the steam temp
            tempSetpoint = STEAM_SETPOINT;

            switchSolenoid(false);

            // pulse pump in steam mode once a decent temperature is attained
            static boolean pulsePump = false;
            if (switchStateNew) pulsePump = false;
            pulsePump = pulsePump || waterTemp > STEAM_PUMP_PULSE_TEMP;
            flowSetpoint = pulsePump && (micros() % 5000000 < (unsigned long)5000000 * STEAM_PUMP_PULSE_DUTY) ? MAX_FLOW_RATE : 0.0;
        }
        // in backflush mode, repeatedly drive the pump until it stalls, then stop pump and close solenoid for a few seconds
        else if (pumpOn && brewMode == BACKFLUSH)
        {
            activity = ACTIVITY_BACKFLUSH;
            switchBuzzer(false);

            tempSetpoint = BREW_SETPOINT;

            static unsigned long stateChangeTime = 0;
            static bool pumping = false;
            if (switchStateNew)
                pumping = true;

            if (switchStateNew || pumping && flowRate > 1.2)
                stateChangeTime = micros() + BACKFLUSH_STALL_TIME;

            if ((long)micros() - (long)stateChangeTime > 0)
            {
                pumping = !pumping;
                stateChangeTime = micros() + (pumping ? BACKFLUSH_STALL_TIME : BACKFLUSH_FLUSH_TIME);
            }

            switchSolenoid(pumping);
            flowSetpoint = pumping ? MAX_FLOW_RATE : 0.0;
        }
        // in pump mode with 1 or 2 cup flow rate, we let the PID controller do the driving
        else if (pumpOn && brewMode != BACKFLUSH)
        {
            bool justStarted = activity != ACTIVITY_BREW;
            activity = ACTIVITY_BREW;
            switchBuzzer(false);

            tempSetpoint = BREW_SETPOINT;

            // calculate total volume of shot
            if (justStarted)
                brewStopVolume = g_totalVolume + (brewMode == BREW_2CUP ? BREW_VOLUME_2CUP : BREW_VOLUME_1CUP);

            // now control flow and solenoid based on whether enough has been pumped through
            flowSetpoint = brewMode == BREW_2CUP ? BREW_FLOW_RATE_2CUP : BREW_FLOW_RATE_1CUP;
// TODO: something like this for pre-infusion?
//            if (g_totalVolume < brewStopVolume - BREW_VOLUME_2CUP + 20)
//                flowSetpoint = MIN_FLOW_RATE;
//
            if (g_totalVolume > brewStopVolume)
            {
                switchSolenoid(false);
                flowSetpoint = 0.0;
            }
            else
                switchSolenoid(true);
        }
        else
        {
            activity = ACTIVITY_READY;

            switchSolenoid(false);
            flowSetpoint = 0.0;

            // guard against long periods of unuse - has the machine been left on accidentally?
            // NOTE: this is actually allowed in backflush mode where it is left pumping at brew temperature
            if (micros() - lastActiveTime > (unsigned long)INACTIVITY_TIMEOUT * 1000000 && !(brewMode == BACKFLUSH && pumpOn && !steamOn))
            {
                switchBuzzer(true);
                tempSetpoint = 0.0;
            }
            else
            {
                switchBuzzer(false);
                tempSetpoint = BREW_SETPOINT;
            }
        }

        // control the power to the pump based on the flow rate setpoint
        if (flowSetpoint <= 0.0)
            g_pumpPower = 0.0;
        else
        {
            static unsigned long lastControlTime = 0;
            double deltaTime = (double)(micros() - lastControlTime) / 1000000.0;
            if (g_pumpPower == 0.0 && flowRate > 0.0)
            {
                // when pumping is initiated, jump to full power - we need to fill the gap between the shower head and the puck
// TODO: for pre-infusion jump to 0.5
                g_pumpPower = 0.5;
                lastControlTime += deltaTime * 1000000;
            }
            // update power 50 times per second (i.e. about once every mains cycle)
            else if (deltaTime >= 0.02)
            {
                lastControlTime += deltaTime * 1000000;
  
                // express temperature as an error from flowSetpoint and pump_pid, which drives pumpPowerRate directly
                double error = flowRate - flowSetpoint;
                pumpPowerRate = fabs(error) * error * PUMP_PID_P;
                double pumpPower = g_pumpPower + deltaTime * pumpPowerRate;
  
                // clip pump power to sensible values
                g_pumpPower = min(1.0, max(0.0, pumpPower));
            }
        }

        // every approx 1 second go on to manage temperature
        {
            static unsigned long lastBoilerPidTime = 0;
            double deltaTime = (double)(micros() - lastBoilerPidTime) / 1000000.0;
            if (deltaTime < 1.0) continue;
            lastBoilerPidTime += deltaTime * 1000000;

            // handle reading errors by switching the heater off and moving on
            double readTemperature = getTemperature();
            if (isnan(readTemperature) || readTemperature < 0.0 || readTemperature > 200.0)
            {
                Serial.println(F("Failed to read sensible temperature"));
                setHeaterPower(0.0);
            }
            else
            {
                // instead of using the temperature as read, we model the temperature and adjust it continually to match the measured temperature
                // this helps greatly because there is about 8s latency between applying a step change to the power and seeing a result
    
                // How much heat is lost to water flowing out?
                // Assumptions: 1. Water flows out at the same rate as it flows in.
                //              2. Water flowing in is at ambient temperature.
                //              3. Between 100°C and 140°C, water flows out progressively more as steam and a lot of heat is lost to vapourisation
                //              4. Water enthalpies at 100°C are close enough for the other temperatures at which we operate.
                //              5. Density of water flowing in is 1;
                double waterToFlowPower = flowRate * (waterTemp - RESERVOIR_TEMPERATURE) * SPEC_HEAT_WATER_100;
                double fractionSteam = max(0.0, min(1.0, (waterTemp - 100.0) / 40.0));
                waterToFlowPower += fractionSteam * flowRate * LATENT_HEAT_VAPORISATION_100;

                // How much power is lost to the atmosphere from the brew head?
                double brewHeadToAmbientPower = (brewHeadTemp - AMBIENT_TEMPERATURE) * BREWHEAD_AMBIENT_XFER_COEFF;

                // How much power is transferred from the boiler to the water?
                double shellToWaterPower = (shellTemp - waterTemp) * BOILER_WATER_XFER_COEFF_NOFLOW / 2.0;
                double elementToWaterPower = (elementTemp - waterTemp) * BOILER_WATER_XFER_COEFF_NOFLOW / 2.0;

                // How much power is transferred from the boiler to the brew head?
                double shellToBrewHeadPower = (shellTemp - brewHeadTemp) * BOILER_BREWHEAD_XFER_COEFF / 2.0;
                double elementToBrewHeadPower = (elementTemp - brewHeadTemp) * BOILER_BREWHEAD_XFER_COEFF / 2.0;
// TODO: what about heat transfer from water to brewhead and vice versa?
                double waterFlowToBrewHeadPower = (waterTemp - brewHeadTemp) * flowRate * SPEC_HEAT_WATER_100;

                double elementToShellPower = (elementTemp - shellTemp) * ELEMENT_SHELL_XFER_COEFF;

                double shellToBodyPower = (shellTemp - bodyTemp) * BOILER_BODY_XFER_COEFF / 2.0;
                double elementToBodyPower = (elementTemp - bodyTemp) * BOILER_BODY_XFER_COEFF / 2.0;

                // Now work out the temperature, which comes from power that didn't go into heat loss or heating the incoming water.
                brewHeadTemp += deltaTime * (shellToBrewHeadPower + elementToBrewHeadPower + waterFlowToBrewHeadPower - brewHeadToAmbientPower) / (SPEC_HEAT_BRASS * (MASS_BREW_HEAD + MASS_PORTAFILTER));
                waterTemp += deltaTime * (shellToWaterPower + elementToWaterPower - waterToFlowPower) / (SPEC_HEAT_WATER_100 * BOILER_VOLUME);
                shellTemp = readTemperature;
                elementTemp += deltaTime * (heaterPower - elementToShellPower - elementToBrewHeadPower - elementToWaterPower - elementToBodyPower) / (SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0);
                bodyTemp += deltaTime * (shellToBodyPower + elementToBodyPower) / HEAT_CAPACITY_BODY;

                // calculate the error in total energy from setpoint for water and boiler
                double error = (shellTemp - tempSetpoint) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0;
                error += (elementTemp - tempSetpoint) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0;
                error += (waterTemp - tempSetpoint) * SPEC_HEAT_WATER_100 * BOILER_VOLUME;

                // arrange heater power so that we will be at correct total energy in 2 seconds (if possible)
                // the error term handles boiler shell and water - other known power sinks are added explicitly
                heaterPower = shellToBrewHeadPower + elementToBrewHeadPower + shellToBodyPower + elementToBodyPower - error / 2.0;
                heaterPower = min(SAFE_HEATER_POWER, max(0.0, heaterPower));

                setHeaterPower(heaterPower);
            }
        }

        // print column headers to the serial port periodically
        static int rowCount = 0;
        if (rowCount == 0)
        {
            Serial.println(F("time\tsteam \tpump  \theat \telemnt\tshell \twater\tb. head\tAC 1/2-\tpump \tdpump \tvolume\tflow \tflow"));
            Serial.println(F("\tswitch\tswitch\tpower\ttemp \ttemp \ttemp \ttemp \tcycle  \tpower\tpwr/dt\t      \tsetpt\trate"));
        }
        rowCount = (rowCount + 1) % 40;

        // print the data to any computer that happens to be listening
        Serial.print((double)micros() / 1000000.0);
        Serial.print("\t");
        Serial.print(steamOn ? "on" : "off");
        Serial.print("\t");
        Serial.print(pumpOn ? "on" : "off");
        Serial.print("\t");
        Serial.print(heaterPower);
        Serial.print("\t");
        Serial.print(elementTemp);
        Serial.print("\t");
        Serial.print(shellTemp);
        Serial.print("\t");
        Serial.print(waterTemp);
        Serial.print("\t");
        Serial.print(brewHeadTemp);
        Serial.print("\t");
        Serial.print(g_mainsHalfCycleLength);
        Serial.print("\t");
        Serial.print(g_pumpPower);
        Serial.print("\t");
        Serial.print(pumpPowerRate);
        Serial.print("\t");
        Serial.print(g_totalVolume);
        Serial.print("\t");
        Serial.print(flowSetpoint);
        Serial.print("\t");
        Serial.println(flowRate);
    }
}
