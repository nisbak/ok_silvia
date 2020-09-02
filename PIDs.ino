//************************************************************************
// PID setup
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

double g_PIDSetpoint_F, g_PIDInput_F, g_PIDOutput_F, g_PIDInput_P, g_PIDOutput_P, g_PIDSetpoint_P;
PID pressurePID(&g_PIDInput_P, &g_PIDOutput_P, &g_PIDSetpoint_P,Kpp,Kpi,Kpd, DIRECT);
PID flowPID(&g_PIDInput_F, &g_PIDOutput_F, &g_PIDSetpoint_F,Kfp,Kfi,Kfd, DIRECT);
