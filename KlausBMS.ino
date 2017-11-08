/**
 * ==========================================================================
 * Klaus' Twizy Lithium BMS
 * ==========================================================================
 * 
 * Hardware setup:
 *  - basically like Twizy-Battery-Part-List.md#example-arduino-wiring-scheme
 *  - supports packs with up to 16 cells (see CELL_COUNT in _config.h)
 * 
 * Author:
 *  - Michael Balzer        <dexter@dexters-web.de>
 * 
 * Ideas, support & testing:
 *  - Błażej Błaszczyk      <blazej.blaszczyk@pascal-engineering.com>
 *  - Klaus Zinser          <klauszinser@posteo.eu>
 * 
 * Libraries used:
 *  - TwizyVirtualBMS   https://github.com/dexterbg/Twizy-Virtual-BMS
 *  - MCP_CAN           https://github.com/coryjfowler/MCP_CAN_lib
 *  - FlexiTimer2       https://github.com/PaulStoffregen/FlexiTimer2
 *  - AltSoftSerial     https://github.com/PaulStoffregen/AltSoftSerial
 * 
 * License:
 *  This is free software under GNU Lesser General Public License (LGPL)
 *  https://www.gnu.org/licenses/lgpl.html
 *  
 */

#define KLAUS_BMS_VERSION    "V0.9.0 (2017-11-08)"

#include <EEPROM.h>
#include <util/crc16.h>

#include "TwizyVirtualBMS_config.h"
#include "TwizyVirtualBMS.h"
#include "AltSoftSerial.h"
#include "KlausBMS_config.h"

// VirtualBMS:
TwizyVirtualBMS twizy;

// Bluetooth software serial port:
// Note: AltSoftSerial uses fixed pins!
// i.e. Arduino Nano: RX = pin 8, TX = pin 9
//      Arduino Mega: RX = pin 48, TX = pin 46
AltSoftSerial bt;

Stream *com_channels[] = { &Serial, &bt };

char inputbuf[30] = "";     // shared command input buffer
byte inputpos = 0;

byte quiet = 0;             // status output inhibitor (seconds)
char temp_chg;              // charger temperature [°C]


// Current, capacity & charge (coulomb):
//  QA = quarter amps (1/4 A) (Twizy current resolution)
//  CS = centi seconds (1/100 s) (Twizy time resolution)
#define SCALE_CURR_QA       ((SCALE_CURR) * 4L)
#define BASE_CURR_QA        ((BASE_CURR) * 4L)
#define CAP_NOMINAL_QACS    ((CAP_NOMINAL_AH) * 4L * 3600L * 100L)
#define AMPS(qa)            ((float) (qa) / 4L)
#define AMPHOURS(qacs)      ((float) (qacs) / 4L / 3600L / 100L)


// --------------------------------------------------------------------------
// KlausBMS main object
//

class KlausBMS {

public:
  
  // --------------------------------------------------------------------------
  // State variables
  // 
  
  float
    temp_f,                       // pack front temperature [°C]
    temp_r,                       // pack rear temperature [°C]
    tdif;                         // difference front/rear temperature [°C]

  float
    vpack,                        // pack voltage [V]
    vstack[CELL_COUNT],           // stacked voltages [V]
    vcell[CELL_COUNT];            // cell voltages [V]

  float
    cmin,                         // minimum cell voltage [V]
    cmax,                         // maximum cell voltage [V]
    cdif;                         // difference min/max cell voltage [V]
  byte
    cmin_i,                       // cell number with min voltage
    cmax_i;                       // cell number with max voltage
  float
    cmin_soc,                     // minimum cell voltage based SOC [%]
    cmax_soc;                     // maximum cell voltage based SOC [%]

  float
    soc,                          // effective SOC [%] combining …
    soc_volt,                     // … voltage based SOC estimation [%]
    soc_coulomb;                  // … charge based SOC estimation [%]

  float
    soh;                          // state of health [%]

  unsigned int
    drvpwr,                       // drive power level [W]
    recpwr;                       // recuperation power level [W]
  byte
    chgcur;                       // charge current level [A]

  unsigned long
    error;                        // display error status (0 = OK, see setError())
  byte
    bms_error;                    // informational BMS error code (see TwizyBmsError / setInfoError())

  unsigned long
    cap_qacs,                     // battery charge capacity (self-adjusting) [1/400 As]
    avail_qacs;                   // available charge [1/400 As]
  
  signed long
    curr_qa;                      // momentary current (10 ms measurement) [1/4 A]

  float
    soc_chgstart;                 // charge start SOC [%]
  unsigned long
    avail_qacs_chgstart;          // charge start coulomb count [1/400 As]


  // --------------------------------------------------------------------------
  // Configuration variables
  // 
  
  // Maximum charge current to use [A]
  // …at 20 °C & higher:
  byte max_charge_current;
  // …at zero °C:
  byte max_charge_current_0c;

  // Maximum driving & recuperation power limits to use [W]
  // …at 20 °C & higher:
  unsigned int max_drive_power;
  unsigned int max_recup_power;
  // …at zero °C:
  unsigned int max_drive_power_0c;
  unsigned int max_recup_power_0c;

  // Drive power cutback [%]:
  // (100% at FULL → 100% at <SOC1>% → <LVL2>% at <SOC2>% → 0% at EMPTY)
  byte drv_cutback_soc1;
  byte drv_cutback_soc2;
  byte drv_cutback_lvl2;

  // Voltage range for discharging [V]:
  float vmin_drv;
  float vmax_drv;

  // Voltage range for charging [V]:
  float vmin_chg;
  float vmax_chg;

  // Prioritize voltage based SOC [%]:
  byte soc_volt_prio_above;
  byte soc_volt_prio_below;

  // Degrade coulomb based SOC [%]:
  byte soc_coul_degr_above;
  byte soc_coul_degr_below;

  // Charge stop at SOC [%]:
  byte chg_stop_soc;

public:
  
  // --------------------------------------------------------------------------
  // Init
  // 
  
  KlausBMS() {
    init();
  }

  void initSOC(float newsoc) {
    
    soc = newsoc;
    soc_volt = newsoc;
    
    #ifdef PORT_CURR
      soc_coulomb = newsoc;
      avail_qacs = cap_qacs / 100 * soc_coulomb;
    #endif

    float cv;
    
    if (twizy.inState(Charging)) {
      cv = VMIN_CHG + (VMAX_CHG - VMIN_CHG) * newsoc / 100.0;
    } else {
      cv = VMIN_DRV + (VMAX_DRV - VMIN_DRV) * newsoc / 100.0;
    }

    for (int i=0; i < CELL_COUNT; i++) {
      vcell[i] = cv;
      vstack[i] = cv * (i+1);
    }
    
    vpack = vstack[CELL_COUNT-1];
    
    cmin_soc = cmax_soc = newsoc;
    cmin = cmax = cv;
  }

  void initSOH(float newsoh) {
    
    soh = newsoh;
    
    #ifdef PORT_CURR
      cap_qacs = CAP_NOMINAL_QACS / 100 * soh;
      avail_qacs = cap_qacs / 100 * soc_coulomb;
    #endif
  }
  
  void init() {
    
    // State variables:
    
    temp_f = 20.0;
    temp_r = 20.0;
    tdif = 0;

    initSOC(99.0);
    initSOH(100.0);

    drvpwr = MAX_DRIVE_POWER;
    recpwr = MAX_RECUP_POWER;
    chgcur = MAX_CHARGE_CURRENT;

    error = TWIZY_OK;
    bms_error = bmsError_None;

    cap_qacs = CAP_NOMINAL_QACS;
    avail_qacs = 0.99 * CAP_NOMINAL_QACS;
  
    curr_qa = 0;

    soc_chgstart = 100;
    avail_qacs_chgstart = 0;
  

    // Configuration variables:
    
    max_charge_current = MAX_CHARGE_CURRENT;
    
    max_charge_current_0c = MAX_CHARGE_CURRENT_0C;
    
    max_drive_power = MAX_DRIVE_POWER;
    max_recup_power = MAX_RECUP_POWER;

    max_drive_power_0c = MAX_DRIVE_POWER_0C;
    max_recup_power_0c = MAX_RECUP_POWER_0C;

    drv_cutback_soc1 = DRV_CUTBACK_SOC1;
    drv_cutback_soc2 = DRV_CUTBACK_SOC2;
    drv_cutback_lvl2 = DRV_CUTBACK_LVL2;

    vmin_drv = VMIN_DRV;
    vmax_drv = VMAX_DRV;

    vmin_chg = VMIN_CHG;
    vmax_chg = VMAX_CHG;

    soc_volt_prio_above = SOC_VOLT_PRIO_ABOVE;
    soc_volt_prio_below = SOC_VOLT_PRIO_BELOW;

    soc_coul_degr_above = SOC_COUL_DEGR_ABOVE;
    soc_coul_degr_below = SOC_COUL_DEGR_BELOW;
    
    chg_stop_soc = 100;
  }
  
  void begin() {
    
    // Init I/O ports:
    
    pinMode(PORT_VOLT, INPUT);
    pinMode(PORT_TEMP_F, INPUT);
    pinMode(PORT_TEMP_R, INPUT);
    #ifdef PORT_CURR
      pinMode(PORT_CURR, INPUT);
    #endif
    
    pinMode(PORT_MUX_S0, OUTPUT);
    pinMode(PORT_MUX_S1, OUTPUT);
    pinMode(PORT_MUX_S2, OUTPUT);
    pinMode(PORT_MUX_S3, OUTPUT);
    
    // load state from EEPROM:
    
    loadState();
    
    // init VirtualBMS state:
    
    twizy.setInfoBmsType(bmsType_VirtualBMS);
    twizy.setInfoState1(twizy.state());
    twizy.setInfoState2(0);
    twizy.setInfoError(bms_error);
    twizy.setInfoBalancing(0);
    
    twizy.setPowerLimits(drvpwr, recpwr);
    twizy.setChargeCurrent(chgcur);
    twizy.setSOC(soc);
    twizy.setTemperature(temp_r, temp_f, true);
    twizy.setVoltage(vpack, true);
    twizy.setError(error);
    twizy.setSOH(soh);
    twizy.setCurrent(curr_qa);

    Serial.println(F("bms.begin: done"));
  }


  // --------------------------------------------------------------------------
  // EEPROM utility: save state
  // 
  void saveState() {
    uint16_t crc, i;
    uint8_t *data;
    
    // calculate CRC:
    crc = 0xffff;
    data = (uint8_t *) this;
    for (i = 0; i < sizeof(*this); i++) {
      crc = _crc16_update(crc, data[i]);
    }
    
    // write:
    EEPROM.put(0, crc);
    EEPROM.put(sizeof(crc), *this);
    
    Serial.println(F("bms.saveState: state saved"));
  }


  // --------------------------------------------------------------------------
  // EEPROM utility: load state
  // 
  void loadState() {
    uint16_t crc, savedcrc, i;
    
    // check CRC:
    EEPROM.get(0, savedcrc);
    crc = 0xffff;
    for (i = 0; i < sizeof(*this); i++) {
      crc = _crc16_update(crc, EEPROM[sizeof(crc) + i]);
    }
    
    if (crc != savedcrc) {
      Serial.println(F("bms.loadState: CRC mismatch, state not loaded!"));
    }
    else {
      // read:
      EEPROM.get(sizeof(crc), *this);
      Serial.println(F("bms.loadState: state loaded"));
    }
  }


  // --------------------------------------------------------------------------
  // Utility: output voltage alert/state details
  // 
  void printVoltAlert(FLASHSTRING *intro) {
    if (quiet) {
      return;
    }
    for (Stream *s : com_channels) {
      s->print(F("!!! ")); s->print(intro);
      s->print(F(": dif="));  s->print(cdif, 2);
      s->print(F(", min="));  s->print(cmin, 2);
      s->print(F(" #"));      s->print(cmin_i);
      s->print(F(", max="));  s->print(cmax, 2);
      s->print(F(" #"));      s->print(cmax_i);
      s->print(F(", avg="));  s->print(vpack/CELL_COUNT, 2);
      s->println();
    }
  }


  // --------------------------------------------------------------------------
  // Utility: output temperature alert/state details
  // 
  void printTempAlert(FLASHSTRING *intro) {
    if (quiet) {
      return;
    }
    for (Stream *s : com_channels) {
      s->print(F("!!! ")); s->print(intro);
      s->print(F(": dif="));     s->print(tdif, 1);
      s->print(F(", temp_f="));  s->print(temp_f, 1);
      s->print(F(", temp_r="));  s->print(temp_r, 1);
      s->println();
    }
  }


  // --------------------------------------------------------------------------
  // Utility: output space padded numbers
  // 
  
  // …integer:
  void print(Stream *s, char len, long ival) {
    unsigned long p = 10;
    
    // get length of ival:
    if (ival < 0) {
      len--;
      while (-ival >= p) {
        p *= 10;
        len--;
      }
    } else {
      while (ival >= p) {
        p *= 10;
        len--;
      }
    }
    
    // pad & print:
    while (--len > 0) {
      s->print(' ');
    }
    s->print(ival);
  }
  
  // …float:
  void print(Stream *s, char len, float fval, char prec) {
    long p;
    
    // get rounded integer part:
    p = 1;
    for (char i=0; i < prec; i++) {
      p *= 10;
    }
    long ival = ((long) round(fval * p)) / p;
    
    // get length of rounded integer part:
    p = 10;
    if (signbit(fval)) {
      len--;
      ival = -ival;
    }
    while (ival >= p) {
      p *= 10;
      len--;
    }
    
    // pad & print:
    while (--len > 0) {
      s->print(' ');
    }
    s->print(fval, prec);
  }
  
  
  // --------------------------------------------------------------------------
  // Utility: output BMS status
  // 
  
  void printStatus(Stream *s) {

    /*
      | 100.0 %SOC |   55.6 V  | 18000 Wd |  35 Ac | StartTrickle
      | 100.0 %Sv  | -500.0 A  |  8500 Wr | -10 Cc | [error]
      | 100.0 %Sc  |  120.0 Ah |   100 Cf |< 88 %V | 0.12 V
      | 100.0 %SOH |  120.0 Ah |   100 Cr |>100 %V |
      | 3.12 | 3.12 | 3.12 |<3.12 | 3.12 | 3.12 | 3.12 | 3.12 |
      | 3.12 | 3.12 | 3.12 | 3.12 |>3.12 | 3.12 | 3.12 | 3.12 |
    */
    
    s->println();
    
    s->print('|'); print(s, 4, soc, 1);                   s->print(F(" %SOC")); s->print(' ');
    s->print('|'); print(s, 5, vpack, 1);                 s->print(F(" V "));   s->print(' ');
    s->print('|'); print(s, 6, drvpwr);                   s->print(F(" Wd"));  s->print(' ');
    s->print('|'); print(s, 4, chgcur);                   s->print(F(" Ac"));  s->print(' ');
    s->print('|'); s->print(' '); s->print(twizy.stateName());
    s->println();

    s->print('|'); print(s, 4, soc_volt, 1);              s->print(F(" %Sv ")); s->print(' ');
    s->print('|'); print(s, 5, AMPS(curr_qa), 1);         s->print(F(" A "));   s->print(' ');
    s->print('|'); print(s, 6, recpwr);                   s->print(F(" Wr"));  s->print(' ');
    s->print('|'); print(s, 4, temp_chg);                 s->print(F(" Cc"));  s->print(' ');
    s->print('|');
    if (error) {
      s->print(' ');
      s->print(error, HEX);
    }
    s->println();
    
    s->print('|'); print(s, 4, soc_coulomb, 1);               s->print(F(" %Sc ")); s->print(' ');
    s->print('|'); print(s, 5, AMPHOURS(avail_qacs), 1);      s->print(F(" Ah"));   s->print(' ');
    s->print('|'); print(s, 6, temp_f);                       s->print(F(" Cf"));  s->print(' ');
    s->print('|'); s->print('<'); print(s, 3, (int)cmin_soc); s->print(F(" %V"));  s->print(' ');
    s->print('|'); print(s, 2, cdif, 2); s->print(F(" V"));
    s->println();
    
    s->print('|'); print(s, 4, soh, 1);                       s->print(F(" %SOH")); s->print(' ');
    s->print('|'); print(s, 5, AMPHOURS(cap_qacs), 1);        s->print(F(" Ah"));   s->print(' ');
    s->print('|'); print(s, 6, temp_r);                       s->print(F(" Cr"));  s->print(' ');
    s->print('|'); s->print('>'); print(s, 3, (int)cmax_soc); s->print(F(" %V"));  s->print(' ');
    s->print('|');
    s->println();
    
    // cell voltages:
    
    for (byte i=0; i < CELL_COUNT; i++) {
      if (i == CELL_COUNT/2) {
        s->println('|');
      }
      s->print('|');
      s->print((i == cmin_i) ? '<' : (i == cmax_i) ? '>' : ' ');
      s->print(vcell[i], 2);
      s->print(' ');
    }
    s->println('|');
    
  }  
  
  
  // --------------------------------------------------------------------------
  // Callback: handle state transition for BMS
  //  - called by twizy.enterState() after Twizy handling
  // Note: avoid complex operations, this needs to be fast.
  // 
  void enterState(TwizyState currentState, TwizyState newState) {
    
    static uint8_t tricklecnt = 30;
    
    #if TWIZY_DEBUG_LEVEL == 0
      Serial.print(F("bms.enterState: newState="));
      Serial.println(FS(twizyStateName[newState]));
    #endif
    
    
    #if CALIBRATION_MODE == 0
      
      // ----------------------------------------------------------------------
      // bms.enterState: lower SOC at switch-on to prevent immediate charge stop:
      //
      
      if (currentState == Init && newState == Ready) {
        if (soc > 99) {
          soc -= 0.01;
          chgcur = 5;
          twizy.setSOC(soc);
          twizy.setChargeCurrent(chgcur);
          Serial.print(F("bms.enterState: SOC lowered to "));
          Serial.println(soc, 1);
        }
      }
      
      
      // ----------------------------------------------------------------------
      // bms.enterState: battery capacity adjustment by charging:
      //
      // Principle of operation:
      //  SOC estimation combines pack voltage _and_ coulomb counting.
      //  Voltage limits are hard limits overriding coulomb count, so
      //  100% SOC means 100% voltage. So we can check if the charge
      //  sum matches the SOC difference and adjust the capacity by the
      //  difference.
      //

      if (newState == StartCharge) {
        
        #ifdef PORT_CURR
          // remember start SOC & charge:
          soc_chgstart = soc;
          avail_qacs_chgstart = avail_qacs;
        #endif // #ifdef PORT_CURR
        
      }
      
      else if (newState == StopCharge) {
    
        #ifdef PORT_CURR
          
          // adjust capacity & SOH when charged at least 50% SOC difference:
          float soc_charged = soc - soc_chgstart;
          if (soc_charged >= 50) {

            // the charged SOC difference should account for...
            unsigned long expected_chgd_qacs = cap_qacs / 100 * soc_charged;
            // we actually got...
            unsigned long real_chgd_qacs = avail_qacs - avail_qacs_chgstart;
            
            // adjust capacity:
            unsigned long oldcap = cap_qacs;
            unsigned long newcap = real_chgd_qacs / soc_charged * 100;
            // ...smoothed, weighted by soc_charged:
            cap_qacs = (oldcap / SMOOTH_CAP) * (SMOOTH_CAP - soc_charged)
                    + (newcap / SMOOTH_CAP) * soc_charged;
            
            // calculate SOH:
            soh = constrain((float) cap_qacs / CAP_NOMINAL_QACS * 100, 0, 100);
            twizy.setSOH(soh);
            
            // output adjustment info:
            for (Stream *s : com_channels) {
              s->println();
              s->println(F("bms.enterState: capacity/SOH adjustment:"));
              s->print(F("- SOC charged = ")); s->println(soc_charged, 1);
              s->print(F("- expected Ah = ")); s->println(AMPHOURS(expected_chgd_qacs), 1);
              s->print(F("-  charged Ah = ")); s->println(AMPHOURS(real_chgd_qacs), 1);
              s->print(F("-  old cap Ah = ")); s->println(AMPHOURS(oldcap), 1);
              s->print(F("-  new cap Ah = ")); s->println(AMPHOURS(cap_qacs), 1);
              s->print(F("-   new SOH % = ")); s->println(soh, 1);
              s->println();
            }
          }
          
          // adjust available charge based on SOC:
          avail_qacs = cap_qacs / 100 * soc;
          
          // …or just limit to cap?
          //if (soc > 99.99)
          //  avail_qacs = cap_qacs;
          //else if (avail_qacs > cap_qacs)
          //  avail_qacs = cap_qacs;
        
        #endif // PORT_CURR
        
        // save state to EEPROM:
        saveState();
        tricklecnt = 30;
        
      } // if (newState == StopCharge)
      
      else if (newState == StopDrive) {
        
        // save state to EEPROM:
        saveState();
        tricklecnt = 30;
        
      }

      else if (newState == StopTrickle) {
        
        // to avoid high wear on the EEPROM, only save state to EEPROM
        // after 30 successive trickle charges:
        if (--tricklecnt == 0) {
          saveState();
          tricklecnt = 30;
        }
        
      }
      
    #endif // CALIBRATION_MODE == 0
    
    twizy.setInfoState1(newState);

  } // bms.enterState()


  // --------------------------------------------------------------------------
  // Callback: timer ticker
  //  - called every 10 ms by twizy.ticker() after twizy handling
  //  - clockCnt cyclic range: 0 .. 2999 = 30 seconds (reset to 0 on Off/Init)
  // Note: avoid complex operations, this needs to be fast.
  // 
  void ticker(unsigned int clockCnt) {
    
    int i;
    
    
    // ----------------------------------------------------------------------
    // bms.ticker: read current & count coulomb every 10 ms while on:
    //
    
    #ifdef PORT_CURR
      
      if (!twizy.inState(Off)) {
        
        // read current level:
        curr_qa = BASE_CURR_QA + analogRead(PORT_CURR) * VPORT * SCALE_CURR_QA;
        curr_qa = constrain(curr_qa, -2000, 2000);

        if (twizy.inState(Charging, StopCharge)) {
          curr_qa *= CURR_POLARITY_CHG;
        } else {
          curr_qa *= CURR_POLARITY_DRV;
        }
        
        #if CALIBRATION_MODE == 0
          // update VirtualBMS model:
          twizy.setCurrentQA(curr_qa);
        #endif
        
        // account for discharge/charge:
        avail_qacs += curr_qa;
        
      }
      
    #endif // PORT_CURR
    
    
    // ----------------------------------------------------------------------
    // bms.ticker: Read stacked cell voltages
    //
    
    if (!twizy.inState(Off) && clockCnt % 10 == 0) {
      
      for (i=0; i < CELL_COUNT; i++) {
        
        // select MUX input for PORT_VOLT:
        digitalWrite(PORT_MUX_S0, (i & 1) ? HIGH : LOW);
        digitalWrite(PORT_MUX_S1, (i & 2) ? HIGH : LOW);
        digitalWrite(PORT_MUX_S2, (i & 4) ? HIGH : LOW);
        digitalWrite(PORT_MUX_S3, (i & 8) ? HIGH : LOW);
        
        // read stacked voltage:
        float vc = analogRead(PORT_VOLT) * SCALE_VOLT[i];
        
        // …smooth:
        vstack[i] = ((vstack[i] * (SMOOTH_VOLT-1)) + vc) / SMOOTH_VOLT;
      }
      
      // derive single cell voltages from stacked voltages:
      vpack = vstack[CELL_COUNT-1];
      for (i=CELL_COUNT-1; i>0; i--) {
        vcell[i] = max(vstack[i] - vstack[i-1], 0);
      }
      vcell[0] = vstack[0];
      
      #if CALIBRATION_MODE == 0
        
        // update VirtualBMS model:
        #if CELL_COUNT >= 14
          twizy.setVoltage(vpack, false);
          for (i=1; i<=CELL_COUNT; i++) {
            twizy.setCellVoltage(i, vcell[i-1]);
          }
        #else
          twizy.setVoltage(vpack, true);
        #endif
        
      #endif // CALIBRATION_MODE == 0
    }
    
    
    // ----------------------------------------------------------------------
    // bms.ticker: more processing / status output?
    //
    
    #if CALIBRATION_MODE == 1
      
      // 10 seconds interval:
      if (clockCnt % 1000 != 0) {
        return;
      }
      
      Serial.println(F("\r\n*** CALIBRATION INFO [10s interval] ***"));
      
      for (i=0; i < CELL_COUNT; i++) {
        Serial.print(F("<    c"));
        if (i<10) Serial.print('0');
        Serial.print(i);
        Serial.print(F(" = "));
        print(&Serial, 3, vstack[i], 3);
        Serial.print(F("  [ "));
        print(&Serial, 3, vcell[i], 3);
        Serial.println(F(" ]"));
      }
      
    #else // CALIBRATION_MODE == 0
      
      // 1 second interval:
      if (twizy.inState(Off) || clockCnt % 100 != 0) {
        return;
      }

      #if TWIZY_DEBUG_LEVEL >= 1
        if (!quiet) {
          Serial.println(F("\r\nbms.ticker:"));
        }
      #endif
      
    #endif // CALIBRATION_MODE
    
    
    error = TWIZY_OK;
    bms_error = bmsError_None;
    
    
    // ------------------------------------------------------------
    // bms.ticker: find min/max cell voltages
    //
    
    cmin = 5.0;
    cmax = 0.0;
    
    for (i=0; i < CELL_COUNT; i++) {
      if (vcell[i] < cmin) {
        cmin = vcell[i];
        cmin_i = i;
      }
      if (vcell[i] > cmax) {
        cmax = vcell[i];
        cmax_i = i;
      }
    }
    
    cdif = cmax - cmin;
    
    
    // ----------------------------------------------------------------------
    // bms.ticker: calculate voltage based SOC
    // 
    // - newsoc_volt = direct SOC in operation mode voltage range
    // - soc_volt = smoothed operation mode SOC
    //    (used to derive drive & recup power & charge current)
    // … accordingly for cmin_soc & cmax_soc
    //
    
    float newsoc_volt, newsoc_cmin, newsoc_cmax;
    
    // voltage range depends on operation mode:
    if (twizy.inState(Charging)) {
      newsoc_volt = (vpack - (vmin_chg * CELL_COUNT)) / ((vmax_chg - vmin_chg) * CELL_COUNT) * 100.0;
      newsoc_cmin = (cmin - vmin_chg) / (vmax_chg - vmin_chg) * 100.0;
      newsoc_cmax = (cmax - vmin_chg) / (vmax_chg - vmin_chg) * 100.0;
    }
    else {
      newsoc_volt = (vpack - (vmin_drv * CELL_COUNT)) / ((vmax_drv - vmin_drv) * CELL_COUNT) * 100.0;
      newsoc_cmin = (cmin - vmin_drv) / (vmax_drv - vmin_drv) * 100.0;
      newsoc_cmax = (cmax - vmin_drv) / (vmax_drv - vmin_drv) * 100.0;
    }
    
    // smooth...
    if (newsoc_volt < soc_volt) {
      // slow adaption to lower voltages:
      soc_volt = ((soc_volt * (SMOOTH_SOC_DOWN-1)) + newsoc_volt) / SMOOTH_SOC_DOWN;
      cmin_soc = ((cmin_soc * (SMOOTH_SOC_DOWN-1)) + newsoc_cmin) / SMOOTH_SOC_DOWN;
      cmax_soc = ((cmax_soc * (SMOOTH_SOC_DOWN-1)) + newsoc_cmax) / SMOOTH_SOC_DOWN;
    }
    else {
      if (twizy.inState(Charging)) {
        // fast adaption while charging:
        soc_volt = ((soc_volt * (SMOOTH_SOC_UP_CHG-1)) + newsoc_volt) / SMOOTH_SOC_UP_CHG;
        cmin_soc = ((cmin_soc * (SMOOTH_SOC_UP_CHG-1)) + newsoc_cmin) / SMOOTH_SOC_UP_CHG;
        cmax_soc = ((cmax_soc * (SMOOTH_SOC_UP_CHG-1)) + newsoc_cmax) / SMOOTH_SOC_UP_CHG;
      }
      else {
        // slow adaption while driving:
        soc_volt = ((soc_volt * (SMOOTH_SOC_UP_DRV-1)) + newsoc_volt) / SMOOTH_SOC_UP_DRV;
        cmin_soc = ((cmin_soc * (SMOOTH_SOC_UP_DRV-1)) + newsoc_cmin) / SMOOTH_SOC_UP_DRV;
        cmax_soc = ((cmax_soc * (SMOOTH_SOC_UP_DRV-1)) + newsoc_cmax) / SMOOTH_SOC_UP_DRV;
      }
    }
    
    // sanitize...
    soc_volt = constrain(soc_volt, 0.0, 100.0);
    cmin_soc = constrain(cmin_soc, 0.0, 100.0);
    cmax_soc = constrain(cmax_soc, 0.0, 100.0);


    #ifdef PORT_CURR
      
      // ----------------------------------------------------------------------
      // bms.ticker: calculate coulomb based SOC
      //
      
      #if CALIBRATION_MODE == 1
        Serial.print(F("<   curr = ")); Serial.println((float) curr_qa/4, 1);
      #endif
      
      soc_coulomb = (float) avail_qacs / cap_qacs * 100;
      soc_coulomb = constrain(soc_coulomb, 0, 100);
      
      
      // ----------------------------------------------------------------------
      // bms.ticker: combine coulomb & voltage based SOC (hybrid SOC):
      // - prioritize soc_volt over soc_coulomb …
      //    a) …when soc_volt approaches 0/100%
      //    b) …when soc_coulomb approaches 0/100%
      //
      
      float soc_volt_prio = 0, soc_coul_degr = 0;

      if (twizy.inState(Charging) || curr_qa > 0) {
        // Charging: prioritize voltage when approaching 100%
        if (soc_volt > soc_volt_prio_above)
          soc_volt_prio = (soc_volt - soc_volt_prio_above) / (100 - soc_volt_prio_above);
        if (soc_coulomb > soc_coul_degr_above)
          soc_coul_degr = (soc_coulomb - soc_coul_degr_above) / (100 - soc_coul_degr_above);
      } else {
        // Discharging: prioritize voltage when approaching 0%
        if (soc_volt < soc_volt_prio_below)
          soc_volt_prio = (soc_volt_prio_below - soc_volt) / soc_volt_prio_below;
        if (soc_coulomb < soc_coul_degr_below)
          soc_coul_degr = (soc_coul_degr_below - soc_coulomb) / soc_coul_degr_below;
      }
      
      soc_volt_prio = max(soc_volt_prio, soc_coul_degr);
      
      soc = (soc_volt * soc_volt_prio) + (soc_coulomb * (1 - soc_volt_prio));
      soc = constrain(soc, 0, 100);
      
    #else // PORT_CURR undefined
      
      soc = soc_volt;
      
    #endif // PORT_CURR
    
    
    // ----------------------------------------------------------------------
    // bms.ticker: Derive power limits & charge current from SOC
    //

    // scale down drive power for low SOC:
    //  100% at FULL → 100% at <SOC1> → <LVL2> at <SOC2> → 0% at EMPTY
    
    // Note: minimum cell voltage SOC has priority if below <SOC2>,
    //  so if there is a bad cell in the pack, it will be protected
    //  from over discharge.
    
    #define soc2_drive_power ((drv_cutback_lvl2 / 100.0f) * max_drive_power)
    
    if (cmin_soc <= drv_cutback_soc2) {
      float factor = cmin_soc / drv_cutback_soc2;
      drvpwr = factor * soc2_drive_power;
    }
    else if (soc <= drv_cutback_soc2) {
      float factor = soc / drv_cutback_soc2;
      drvpwr = factor * soc2_drive_power;
    }
    else if (soc <= drv_cutback_soc1) {
      float factor = ((cmin_soc - drv_cutback_soc2) / (drv_cutback_soc1 - drv_cutback_soc2));
      drvpwr = soc2_drive_power + (factor * (max_drive_power - soc2_drive_power));
    }
    else {
      drvpwr = max_drive_power;
    }
    
    // scale down recuperation power & charge current for high SOC:
    //  0% at FULL → 100% at <CHG_CUTBACK_SOC> → 100% at EMPTY
    
    // Note: stop is controlled by overall pack SOC,
    //  current reduction is controlled first by maximum cell voltage.
    //  So the charger will enter the balancing phase when the first cell
    //  is getting full, but won't stop until the pack is full.
    
    if (soc > chg_stop_soc - 0.01) {
      // stop charge & reduce recuperation at 100% pack SOC / charge stop:
      recpwr = 500; // TODO: should(?) be 0 when driving, but affects D/R change
      chgcur = 0;
    }
    else if (cmax_soc >= CHG_CUTBACK_SOC) {
      // keep min 500W / 5A below 100% SOC:
      float factor = ((100 - cmax_soc) / (100 - CHG_CUTBACK_SOC));
      recpwr = 500 + (factor * (max_recup_power - 500));
      chgcur = 5 + (factor * (max_charge_current - 5));
    }
    else if (soc >= CHG_CUTBACK_SOC) {
      // keep min 500W / 5A below 100% SOC:
      float factor = ((100 - soc) / (100 - CHG_CUTBACK_SOC));
      recpwr = 500 + (factor * (max_recup_power - 500));
      chgcur = 5 + (factor * (max_charge_current - 5));
    }
    else {
      recpwr = max_recup_power;
      chgcur = max_charge_current;
    }
    
    
    // ------------------------------------------------------------
    // bms.ticker: Check cell voltage difference (min - max)
    //

    #if CALIBRATION_MODE == 0
    
      // check voltages:
      if (cdif >= VOLT_DIFF_SHUTDOWN) {
        // cell difference is critical: emergency shutdown
        printVoltAlert(F("VOLT_SHUTDOWN"));
        error |= TWIZY_SERV_BATT | TWIZY_SERV_STOP;
        bms_error = bmsError_VoltageDiff;
        twizy.enterState(Error);
      }
      else if (cdif >= VOLT_DIFF_ERROR) {
        // cell difference is high: set STOP signal, reduce drive power, stop recuperation & charge:
        printVoltAlert(F("VOLT_ERROR"));
        error |= TWIZY_SERV_BATT | TWIZY_SERV_STOP;
        bms_error = bmsError_VoltageDiff;
        drvpwr /= 4;
        recpwr /= 4;
        chgcur = 0;
      }
      else if (cdif >= VOLT_DIFF_WARN) {
        // cell difference detected: reduce power & charge levels:
        printVoltAlert(F("VOLT_WARN"));
        error |= TWIZY_SERV_BATT;
        bms_error = bmsError_VoltageDiff;
        drvpwr /= 2;
        recpwr /= 2;
        chgcur = min(chgcur, 5);
      }
      
    #endif // CALIBRATION_MODE
    
    
    // ----------------------------------------------------------------------
    // bms.ticker: Read & check battery temperature
    //
    
    float newtemp_f, newtemp_r;

    // dual read _seems_ to yield better results (LM35D issue?)
    newtemp_f = analogRead(PORT_TEMP_F);
    newtemp_f = BASE_TEMP + analogRead(PORT_TEMP_F) * VPORT * SCALE_TEMP;
    newtemp_r = analogRead(PORT_TEMP_R);
    newtemp_r = BASE_TEMP + analogRead(PORT_TEMP_R) * VPORT * SCALE_TEMP;

    // smooth...
    temp_f = (temp_f * (SMOOTH_TEMP-1) + newtemp_f) / SMOOTH_TEMP;
    temp_r = (temp_r * (SMOOTH_TEMP-1) + newtemp_r) / SMOOTH_TEMP;
    
    tdif = abs(temp_f - temp_r);
    
    #if CALIBRATION_MODE == 1
      
      Serial.print(F("< temp_f = ")); Serial.println(newtemp_f, 1);
      Serial.print(F("< temp_r = ")); Serial.println(newtemp_r, 1);
    
    #else // CALIBRATION_MODE == 0
    
      // check temperatures:
      if (max(temp_f, temp_r) > TEMP_SHUTDOWN || tdif > TEMP_DIFF_SHUTDOWN) {
        // battery is burning: emergency shutdown
        printTempAlert(F("TEMP_SHUTDOWN"));
        error |= TWIZY_SERV_TEMP | TWIZY_SERV_STOP;
        bms_error = bmsError_TemperatureHigh;
        twizy.enterState(Error);
      }
      else if (max(temp_f, temp_r) > TEMP_ERROR || tdif > TEMP_DIFF_ERROR) {
        // battery very hot: set STOP signal, stop recuperation, stop charge:
        printTempAlert(F("TEMP_ERROR"));
        error |= TWIZY_SERV_TEMP | TWIZY_SERV_STOP;
        bms_error = bmsError_TemperatureHigh;
        drvpwr /= 4;
        recpwr = 0;
        chgcur = 0;
      }
      else if (max(temp_f, temp_r) > TEMP_WARN || tdif > TEMP_DIFF_WARN) {
        // battery hot, show warning, reduce recuperation, reduce charge current:
        printTempAlert(F("TEMP_WARN"));
        error |= TWIZY_SERV_TEMP;
        bms_error = bmsError_TemperatureHigh;
        drvpwr /= 2;
        recpwr /= 2;
        chgcur = min(chgcur, 5);
      }
      else if (min(temp_f, temp_r) < 20) {
        // reduce power levels linear by temperature:
        float dt = 20 - min(temp_f, temp_r);
        float dy;
        dy = (max_drive_power - max_drive_power_0c) / 20;
        drvpwr = max(drvpwr - dt * dy, 0);
        dy = (max_recup_power - max_recup_power_0c) / 20;
        recpwr = max(recpwr - dt * dy, 0);
        dy = (max_charge_current - max_charge_current_0c) / 20;
        chgcur = max(chgcur - dt * dy, 5);
      }
      
    #endif // CALIBRATION_MODE


    // ----------------------------------------------------------------------
    // bms.ticker: Read & check charger temperature
    //

    temp_chg = twizy.getChargerTemperature();
    
    if (temp_chg > CHG_CUTBACK_TEMP) {
      float cutback = (float) (CHG_CUTBACK_TEMPMAX - temp_chg) / (CHG_CUTBACK_TEMPMAX - CHG_CUTBACK_TEMP);
      chgcur = max(chgcur * cutback, 5.0);
      bms_error = bmsError_ChargerTemperatureHigh;
	}

    
    // ----------------------------------------------------------------------
    // bms.ticker: Update VirtualBMS model
    //

    #if CALIBRATION_MODE == 0
    
      twizy.setTemperature(min(temp_f, temp_r), max(temp_f, temp_r), false);
      for (i=1; i<=7; i++) {
        // virtual module temperature = linear interpolation front→rear:
        twizy.setModuleTemperature(i, temp_f + ((float) (i-1) / 6 * (temp_r - temp_f)));
      }
      
      twizy.setSOC(soc);
      
      twizy.setPowerLimits(drvpwr, recpwr);
      twizy.setChargeCurrent(chgcur);
      
      twizy.setError(error);
      twizy.setInfoError(bms_error);

    #endif // CALIBRATION_MODE == 0

    
    // ----------------------------------------------------------------------
    // bms.ticker: Output state to serial ports
    //

    #if CALIBRATION_MODE == 0
      
      if (!quiet && clockCnt % 300 == 0) {
        printStatus(&Serial);
      }
      else if (!quiet && clockCnt % 300 == 100) {
        printStatus(&bt);
      }
      
      // Countdown quiet mode:
      if (quiet)
        quiet--;
      
    #endif // CALIBRATION_MODE == 0


  } // bms.ticker()
  
  
  // --------------------------------------------------------------------------
  // Command interpreter:
  // 
  void executeCommand(char *cmd) {

    if (!*cmd) {
      // no command: return to log mode
      quiet = 0;
      return;
    }

    for (Stream *s : com_channels) {
      s->print(F("\r\nbms.executeCommand: "));
      s->println(cmd);
      s->println();
    }
    
    char *arg = strtok(cmd, " ");
    
    if (strcasecmp(cmd, "soh") == 0) {
      // soh <prc>
      if (arg = strtok(NULL, " ")) {
        initSOH(constrain(atoi(arg), 0, 100));
      }
    }
    
    else if (strcasecmp(cmd, "soc") == 0) {
      // soc <prc>
      if (arg = strtok(NULL, " ")) {
        initSOC(constrain(atoi(arg), 0, 100));
      }
    }
    
    else if (strcasecmp(cmd, "mcc") == 0) {
      // mcc <cur>
      if (arg = strtok(NULL, " ")) {
        max_charge_current = constrain(atoi(arg), 5, 35);
      }
    }
    else if (strcasecmp(cmd, "mcc0") == 0) {
      // mcc0 <cur>
      if (arg = strtok(NULL, " ")) {
        max_charge_current_0c = constrain(atoi(arg), 5, 35);
      }
    }
    
    else if (strcasecmp(cmd, "sc") == 0) {
      // sc [<prc>]
      if (arg = strtok(NULL, " ")) {
        chg_stop_soc = constrain(atoi(arg), 1, 100);
      }
      else {
        twizy.setChargeCurrent(0); // stop charge
      }
    }
    
    else if (strcasecmp(cmd, "mpw") == 0) {
      // mpw <drv> <rec> <drv0c> <rec0c>
      if (arg = strtok(NULL, " ")) {
        max_drive_power = constrain(atoi(arg), 500, 30000);
      }
      if (arg = strtok(NULL, " ")) {
        max_recup_power = constrain(atoi(arg), 500, 30000);
      }
    }
    else if (strcasecmp(cmd, "mpw0") == 0) {
      // mpw0 <drv> <rec>
      if (arg = strtok(NULL, " ")) {
        max_drive_power_0c = constrain(atoi(arg), 500, 30000);
      }
      if (arg = strtok(NULL, " ")) {
        max_recup_power_0c = constrain(atoi(arg), 500, 30000);
      }
    }
    
    else if (strcasecmp(cmd, "dcb") == 0) {
      // dcb <soc1> <soc2> <lvl2>
      if (arg = strtok(NULL, " ")) {
        drv_cutback_soc1 = constrain(atoi(arg), 0, 100);
      }
      if (arg = strtok(NULL, " ")) {
        drv_cutback_soc2 = constrain(atoi(arg), 0, 100);
      }
      if (arg = strtok(NULL, " ")) {
        drv_cutback_lvl2 = constrain(atoi(arg), 0, 100);
      }
    }
    
    else if (strcasecmp(cmd, "vrd") == 0) {
      // vrd <min> <max>
      if (arg = strtok(NULL, " ")) {
        vmin_drv = constrain(atof(arg), 0.0, 5.0);
      }
      if (arg = strtok(NULL, " ")) {
        vmax_drv = constrain(atof(arg), 0.0, 5.0);
      }
    }
    
    else if (strcasecmp(cmd, "vrc") == 0) {
      // vrc <min> <max>
      if (arg = strtok(NULL, " ")) {
        vmin_chg = constrain(atof(arg), 0.0, 5.0);
      }
      if (arg = strtok(NULL, " ")) {
        vmax_chg = constrain(atof(arg), 0.0, 5.0);
      }
    }
    
    else if (strcasecmp(cmd, "svp") == 0) {
      // svp <top> <bot>
      if (arg = strtok(NULL, " ")) {
        soc_volt_prio_above = constrain(atoi(arg), 0, 100);
      }
      if (arg = strtok(NULL, " ")) {
        soc_volt_prio_below = constrain(atoi(arg), 0, 100);
      }
    }
    
    else if (strcasecmp(cmd, "scd") == 0) {
      // scd <top> <bot>
      if (arg = strtok(NULL, " ")) {
        soc_coul_degr_above = constrain(atoi(arg), 0, 100);
      }
      if (arg = strtok(NULL, " ")) {
        soc_coul_degr_below = constrain(atoi(arg), 0, 100);
      }
    }
    
    else if (strcasecmp(cmd, "init") == 0) {
      init();
    }
    else if (strcasecmp(cmd, "load") == 0) {
      loadState();
    }
    else if (strcasecmp(cmd, "save") == 0) {
      saveState();
    }
    
    #if FEATURE_CMD_ES == 1
      else if (strcasecmp(cmd, "es") == 0) {
        // es <statenr>
        if (arg = strtok(NULL, " ")) {
          twizy.enterState((TwizyState) constrain(atoi(arg), Off, StopTrickle));
        }
      }
    #endif // FEATURE_CMD_ES
    
    else {
      // HELP:
      for (Stream *s : com_channels) {
        s->println(F("HELP:\r\n"
          " soh <prc> -- set SOH%\r\n"
          " soc <prc> -- set SOC%\r\n"
          " mcc <cur> -- set max charge current @ 20°C\r\n"
          " mcc0 <cur> -- set max charge current @ 0°C\r\n"
          " sc [<prc>] -- stop charge / set stop SOC\r\n"
          " mpw <drv> <rec> -- set max power @ 20°C\r\n"
          " mpw0 <drv> <rec> -- set max power @ 0°C\r\n"
          " dcb <soc1> <soc2> <lvl2> -- set drive cutback\r\n"
          " vrd <min> <max> -- set voltage range discharging\r\n"
          " vrc <min> <max> -- set voltage range charging\r\n"
          " svp <top> <bot> -- set SOC voltage prioritize\r\n"
          " scd <top> <bot> -- set SOC coulomb degrade\r\n"
          " init      -- reset to default config\r\n"
          " load      -- load state from EEPROM\r\n"
          " save      -- save state to EEPROM\r\n"
          #if FEATURE_CMD_ES == 1
            " es <nr>   -- enter a VBMS state (0..12)\r\n"
          #endif
        ));
      }
    }
    
    // output status info:
    for (Stream *s : com_channels) {
      
      s->print(F("- SOH%   = ")); s->println(soh, 1);
      s->print(F("- SOC%   = ")); s->println(soc, 1);
      #ifdef PORT_CURR
        s->print(F("-    cap = ")); s->println(AMPHOURS(cap_qacs), 1);
        s->print(F("-  avail = ")); s->println(AMPHOURS(avail_qacs), 1);
      #endif
      s->println();
      
      s->print(F("- MCC    = ")); s->println(max_charge_current);
      s->print(F("- MCC0   = ")); s->println(max_charge_current_0c);
      s->print(F("- SC     = ")); s->println(chg_stop_soc);
      s->print(F("- MPW    = ")); s->print(max_drive_power);
        s->print(' '); s->println(max_recup_power);
      s->print(F("- MPW0   = ")); s->print(max_drive_power_0c);
        s->print(' '); s->println(max_recup_power_0c);
      s->print(F("- DCB    = ")); s->print(drv_cutback_soc1);
        s->print(' '); s->print(drv_cutback_soc2);
        s->print(' '); s->println(drv_cutback_lvl2);
      s->print(F("- VRD    = ")); s->print(vmin_drv, 3);
        s->print(' '); s->println(vmax_drv, 3);
      s->print(F("- VRC    = ")); s->print(vmin_chg, 3);
        s->print(' '); s->println(vmax_chg, 3);
      s->print(F("- SVP    = ")); s->print(soc_volt_prio_above);
        s->print(' '); s->println(soc_volt_prio_below);
      s->print(F("- SCD    = ")); s->print(soc_coul_degr_above);
        s->print(' '); s->println(soc_coul_degr_below);
      s->println();
    }
    
    // Be quiet for 10 seconds:
    quiet = 10;
    
  } // bms.executeCommand()


} bms; // class KlausBMS


// --------------------------------------------------------------------------
// Callback object wrappers
// 

void bmsEnterState(TwizyState currentState, TwizyState newState) {
  bms.enterState(currentState, newState);
}

void bmsTicker(unsigned int clockCnt) {
  bms.ticker(clockCnt);
}


// -----------------------------------------------------
// SETUP
// 

void setup() {
  
  // Init communication:
  
  Serial.begin(SERIAL_BAUD);
  bt.begin(BT_BAUD);

  for (Stream *s : com_channels) {
    s->println(F(KLAUS_BMS_NAME " " KLAUS_BMS_VERSION));
  }
  
  // Init VirtualBMS & KlausBMS:
  
  twizy.begin();
  bms.begin();
  
  twizy.attachTicker(bmsTicker);
  twizy.attachEnterState(bmsEnterState);
  
  #if TWIZY_CAN_SEND == 0
    Serial.println(F("*** DRY RUN (NO CAN TX) ***"));
    twizy.enterState(Init);
  #endif
  
  #if CALIBRATION_MODE == 1
    Serial.println(F("*** CALIBRATION MODE ***"));
    twizy.enterState(Init);
  #endif
  
} // setup()


// -----------------------------------------------------
// MAIN LOOP
// 

void loop() {
  
  // VirtualBMS main loop:
  
  twizy.looper();
  
  // check for Serial/Bluetooth command:
  // (note: using shared input buffer for both to save space)

  static char lastc = 0;
  char c = 0;
  if (Serial.available()) {
    c = (char) Serial.read();
  }
  else if (bt.available()) {
    c = (char) bt.read();
  }
  
  if (c == '\r' || c == '\n') {
    if (c == '\r' || lastc != '\r') {
      inputbuf[inputpos] = 0;
      bms.executeCommand(inputbuf);
      inputpos = 0;
    }
    lastc = c;
  }
  else if (c >= 32 && inputpos < sizeof(inputbuf)-1) {
    inputbuf[inputpos++] = c;
    lastc = c;
  }
  
}

