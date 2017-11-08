/**
 * ==========================================================================
 * Klaus' Twizy LiFEPO4 (LFP) BMS: Configuration
 * ==========================================================================
 */
#ifndef _KlausBMS_config_h
#define _KlausBMS_config_h

// Personalization:
#define KLAUS_BMS_NAME "KlausBMS"

// Serial baud rate:
// (Arduino supports up to 2 Mbit, but cannot send faster than 1 Mbit)
#define SERIAL_BAUD 1000000

// Bluetooth baud rate:
// (i.e. 57600 / 38400 / 19200 / 9600, Default of HC-05/06 is 9600)
#define BT_BAUD 57600

// Input calibration mode (inhibits normal operation):
#define CALIBRATION_MODE 1
// Note: calibration mode will still allow VirtualBMS state transitions.
// You can drive & charge in calibration mode, but no sensor data
// will be used to update the Twizy SOC & power status. So if you charge
// or drive in calibration mode, you need to monitor your voltages!

// Optional / development features:
#define FEATURE_CMD_ES 0


// --------------------------------------------------------------------------
// PORTS
// --------------------------------------------------------------------------

// Analog input port assignment:
#define PORT_VOLT     A0  // cell voltage MUX
#define PORT_TEMP_F   A1  // temperature sensor front
#define PORT_TEMP_R   A2  // temperature sensor rear
#define PORT_CURR     A3  // pack current (comment out to disable)

// MUX address pins:
#define PORT_MUX_S0   4
#define PORT_MUX_S1   5
#define PORT_MUX_S2   6
#define PORT_MUX_S3   7


// --------------------------------------------------------------------------
// OPERATION
// --------------------------------------------------------------------------

// Maximum charge current to use [A] (5…35)
// …at 20 °C and higher:
#define MAX_CHARGE_CURRENT      35
// …at 0 °C:
#define MAX_CHARGE_CURRENT_0C   20

// Charge current → power drawn from socket:
// 35 A = 2,2 kW
// 30 A = 2,1 kW
// 25 A = 1,7 kW
// 20 A = 1,4 kW
// 15 A = 1,0 kW
// 10 A = 0,7 kW
//  5 A = 0,4 kW

// Maximum driving & recuperation power limits to use [W] (500…30000)
// …at 20 °C and higher:
#define MAX_DRIVE_POWER       25000
#define MAX_RECUP_POWER       12500
// …at 0 °C:
#define MAX_DRIVE_POWER_0C    16000
#define MAX_RECUP_POWER_0C     6000


// Drive power cutback [%]:
// (100% at FULL → 100% at <SOC1>% → <LVL2>% at <SOC2>% → 0% at EMPTY)
#define DRV_CUTBACK_SOC1    50
#define DRV_CUTBACK_SOC2    25
#define DRV_CUTBACK_LVL2    70

// Charge power cutback by SOC [%]:
// (100% at EMPTY → 100% at <SOC>% → 0% at FULL)
#define CHG_CUTBACK_SOC     90

// Charge power cutback by charger temperature [°C]:
#define CHG_CUTBACK_TEMP      50
#define CHG_CUTBACK_TEMPMAX   65


// --------------------------------------------------------------------------
// VOLTAGE
// --------------------------------------------------------------------------

// Number of cells (max 16):
#define CELL_COUNT      16

// Voltage range for discharging [V]:
#define VMIN_DRV        2.90
#define VMAX_DRV        3.35

// Voltage range for charging [V]:
#define VMIN_CHG        2.90
#define VMAX_CHG        3.65

// Voltage smoothing [100ms samples] (min 1 = no smoothing):
#define SMOOTH_VOLT     20

// Port scaling utils:
#define VPORT           5.0 / 1024.0
#define VDIV(R1,R2)     (R1+R2) / R2

// Voltage divider analog input scaling:
// - scale = R_sum / R_probe * calibration
// - first cell is connected directly
const float SCALE_VOLT[CELL_COUNT] = {
    VPORT                 * 1.00000     //  3.2 V
  , VPORT * VDIV( 27, 47) * 1.00000     //  6.4 V
  , VPORT * VDIV( 68, 47) * 1.00000     //  9.6 V
  , VPORT * VDIV(100, 47) * 1.00000     // 12.8 V
  , VPORT * VDIV(150, 47) * 1.00000     // 16.0 V
  , VPORT * VDIV(180, 47) * 1.00000     // 19.2 V
  , VPORT * VDIV(220, 47) * 1.00000     // 22.4 V
  , VPORT * VDIV(270, 47) * 1.00000     // 25.6 V
  , VPORT * VDIV(330, 47) * 1.00000     // 28.8 V
  , VPORT * VDIV(330, 47) * 1.00000     // 32.0 V
  , VPORT * VDIV(390, 47) * 1.00000     // 35.2 V
  , VPORT * VDIV(390, 47) * 1.00000     // 38.4 V
  , VPORT * VDIV(470, 47) * 1.00000     // 41.6 V
  , VPORT * VDIV(470, 47) * 1.00000     // 44.8 V
  , VPORT * VDIV(560, 47) * 1.00000     // 48.0 V
  , VPORT * VDIV(560, 47) * 1.00000     // 51.2 V
};

// Voltage warning/error thresholds [V]:
// (Note: resolution of cell #16 is ~ 80 mV)
#define VOLT_DIFF_WARN        0.3
#define VOLT_DIFF_ERROR       0.6
#define VOLT_DIFF_SHUTDOWN    1.0

// SOC smoothing [1s samples] (min 1 = no smoothing):
#define SMOOTH_SOC_DOWN        60  // adaption to lower voltage
#define SMOOTH_SOC_UP_DRV      30  // adaption to higher voltage while driving
#define SMOOTH_SOC_UP_CHG      10  // adaption to higher voltage while charging


// --------------------------------------------------------------------------
// CURRENT & CAPACITY
// --------------------------------------------------------------------------

// Current analog input scaling:

// LEM HAC-600-S: -600 … +600 A → 0.072 … 4.002 V
//#define SCALE_CURR          (1200.0 / (4.002 - 0.072))
//#define BASE_CURR           (-600.0 - 0.072 * SCALE_CURR)

// Tamura L06P400S05: -400 … +400 A → 1.0 … 4.0 V
#define SCALE_CURR          (800.0 / (4.0 - 1.0))
#define BASE_CURR           (-400.0 - 1.0 * SCALE_CURR)

// If you need to reverse polarity, change to -1:
#define CURR_POLARITY_DRV   1
#define CURR_POLARITY_CHG   1

// Battery capacity:
#define CAP_NOMINAL_AH      120

// Capacity adjustment smoothing (min 100 = fastest adaption):
#define SMOOTH_CAP          200


// --------------------------------------------------------------------------
// HYBRID SOC
// --------------------------------------------------------------------------

// Prioritize voltage based SOC [%]:
#define SOC_VOLT_PRIO_ABOVE     90
#define SOC_VOLT_PRIO_BELOW     20

// Degrade coulomb based SOC [%]:
#define SOC_COUL_DEGR_ABOVE     90
#define SOC_COUL_DEGR_BELOW     20


// --------------------------------------------------------------------------
// TEMPERATURE
// --------------------------------------------------------------------------

// Temperature analog input scaling:
// LM35D: +2 .. +100°, 10 mV / °C => 100 °C = 1.0 V
#define SCALE_TEMP      (100.0 / 1.0)
#define BASE_TEMP       (+2.0)

// Temperature smoothing [samples]:
#define SMOOTH_TEMP     30

// Temperature warning/error thresholds [°C]:
#define TEMP_WARN       40
#define TEMP_ERROR      45
#define TEMP_SHUTDOWN   50

// Temperature front/rear difference warning/error thresholds [°C]:
#define TEMP_DIFF_WARN        3
#define TEMP_DIFF_ERROR       5
#define TEMP_DIFF_SHUTDOWN   10


#endif // _KlausBMS_config_h

