# KlausBMS

This is a battery monitoring system for Renault Twizy based on [TwizyVirtualBMS](https://github.com/dexterbg/Twizy-Virtual-BMS/) supporting Lithium packs with 14 (LiPo) or 16 (LiFe) cells.

This is not a fully featured BMS yet (in terms of M=*management*), it currently relies on an additional standard BMS for cell protection and balancing. The BMS can be a very simple one without any communication though, as KlausBMS does the monitoring itself.

KlausBMS is capable of monitoring up to 16 cell voltages, two temperature sensors and a current sensor.

The hardware setup is following the [VirtualBMS example](https://github.com/dexterbg/Twizy-Virtual-BMS/blob/master/extras/Twizy-Battery-Part-List.md#example-arduino-wiring-scheme). See [Voltage-Dividers.ods](extras/Voltage-Dividers.ods) for the default voltage probe resistor values. Both LEM HAC-600-S and Tamura L06P400S05 current sensors have been used successfully, others may work as well (see configuration).

The BMS will raise warnings and errors based on voltage and temperature anomalies detected. Drive and recuperation power levels as well as charging current can be controlled following the battery state and condition.

Using the (optional but recommended) current sensor it provides SOC and SOH/capacity measurements based on a (simple) hybrid calculation combining coulomb counting and voltage readings.

Log output can be monitored in parallel at the USB serial port and via Bluetooth (if BT module is used). Both log channels also support controlling and configuring the BMS by sending text commands.

KlausBMS is currently used in Twizy battery projects by:
  - Błażej Błaszczyk (LiPo)     <blazej.blaszczyk@pascal-engineering.com>
  - Klaus Zinser (LiFePO4)      <klauszinser@posteo.eu>


## Installation

To download, click the DOWNLOADS button in the top right corner, download the ZIP file. Extract to your Arduino sketchbook folder, start the Arduino IDE and open the sketch.

You will also need these libraries:
  - [Twizy-Virtual-BMS](https://github.com/dexterbg/Twizy-Virtual-BMS)
  - [MCP_CAN_lib by Cory Fowler](https://github.com/coryjfowler/MCP_CAN_lib)
  - …and one of…
    - [TimerOne by Paul Stoffregen](https://github.com/PaulStoffregen/TimerOne)
    - [FlexiTimer2 by Paul Stoffregen](https://github.com/PaulStoffregen/FlexiTimer2)
    - [TimerThree by Paul Stoffregen](https://github.com/PaulStoffregen/TimerThree)


## Configuration & calibration

All configuration directives are located in the files "TwizyVirtualBMS_config.h" and "KlausBMS_config.h", read these files first to get an overview of the configuration features. When installing a KlausBMS update later on, you should be able to mostly keep your configuration files and just replace the "KlausBMS.ino" file, but take care to check for configuration changes.

Most operational configuration values are just initial defaults used when the Arduino EEPROM is empty/invalid. You can also restore the system to these values any time by issuing the "init" command (see "Runtime commands" below).

Configure your hardware characteristics and sensor setup first and enable the calibration mode (is enabled on a fresh copy). Update the voltage sensor scaling if you use other divider resistors.

In calibration mode, the KlausBMS will just output all sensor readings every 10 seconds. Upload the sketch and check for necessary input scaling adjustments. Adjust scaling factors and check again, until the Arduino readings match the real values, then disable calibration mode.

Hint: to do a dry run now, set `TWIZY_CAN_SEND` to 0. The KlausBMS will then enter the VirtualBMS "Init" and "Ready" state on startup even if the Twizy is not switched on (or not even connected).


## Operation

During operation, KlausBMS logs the battery state every three seconds on both communication channels like this:

    |  33.8 %SOC |   52.6 V  |  9598 Wd |  35 Ac | Driving
    |  55.1 %Sv  |   -1.2 A  |  3240 Wr |  29 Cc |
    |  33.8 %Sc  |   40.6 Ah |    19 Cf |< 48 %V | 0.06 V
    | 100.0 %SOH |  120.0 Ah |    18 Cr |> 61 %V |
    | 3.29 | 3.28 | 3.30 | 3.28 | 3.29 | 3.31 | 3.27 | 3.29 |
    |>3.31 |<3.26 | 3.29 | 3.29 | 3.29 | 3.29 | 3.29 | 3.28 |

The layout needs a fixed spacing font, so make sure your (bluetooth) terminal is configured appropriately.

The upper four lines show the battery pack state followed by two lines of up to 16 cell voltages.

The `<` and `>` mark the cells with lowest and highest voltages. Their respective voltage based SOC is shown as `%V` above, along with the voltage difference between them (`0.06 V` in this example).

Pack state:

  - `%SOC`: combined (effective) SOC of battery pack
  - `%Sv`, `%Sc`: voltage and coulomb based SOC of battery pack
  - `%SOH`: state of health (SOH) of battery pack (equivalent to relative capacity)
  - `V`, `A`: momentary pack voltage and current
  - `Ah`: upper/first value = available charge, lower value = absolute pack capacity in amp hours
  - `Wd`, `Wr`: available drive and recuperation power levels (W)
  - `Cf`, `Cr`: temperatures of front and rear sensors (°C)
  - `Ac`: available/configured charge current (A)
  - `Cc`: charger temperature (read from CAN) (°C)
  - `Driving`: the current VirtualBMS state, error codes will be displayed below this field


## Log analysis

Capture the output to a file. Be aware the standard Arduino will do a reset on a new USB connection, so make sure your laptop will stay awake, or apply one of the hardware patches to disable this behaviour. Of course the bluetooth channel can be used as well, take care the terminal app is capable of receiving large logs (some will eat up your memory and become unusable).

The bash script [`logconv.sh`](extras/logconv.sh) can be used to convert the captured output into CSV and ODS format. You'll need some standard linux setup and the OpenOffice or LibreOffice "headless" package.

Alternatively, you can use the [logconv web service](https://dexters-web.de/logconv) on my site.

The resulting table will contain all of the log values in separate numerical columns, so can be used directly for analysis and to generate charts.


## Runtime commands

Set your serial monitor or terminal program to send CR, LF or both.

After sending any command, the KlausBMS will respond with the current configuration state and inhibit standard log output for 10 seconds. To return to standard log output immediately, send an empty command. Any unknown / misspelled command will output a command overview.

| Command | Function |
| --- | --- |
| `soh <prc>` | set SOH% |
| `soc <prc>` | set SOC% |
| `mcc <cur>` | set max charge current |
| `sc [<prc>]` | stop charge / set charge stop SOC |
| `mpw <drv> <rec>` | set max power |
| `dcb <soc1> <soc2> <lvl2>` | set drive cutback |
| `vrd <min> <max>` | set voltage range discharging |
| `vrc <min> <max>` | set voltage range charging |
| `svp <top> <bot>` | set SOC voltage prioritize |
| `scd <top> <bot>` | set SOC coulomb degrade |
| `init`      | reset to default config |
| `load`      | load state from EEPROM |
| `save`      | save state to EEPROM |

**Notes:**

  - All parameters are validated and constrained to their respective ranges.
  - The max current and power levels define the upper limits, the system will use lower values as necessary / configured.
  - To stop a running charge, simply issue `sc` without a parameter.
  - Voltage ranges are defined at the cell voltage level.
  - `save` is done automatically at every drive & charge end.
  - `load` is done automatically after every Arduino start/reset.
  - `init` is done automatically if the EEPROM is empty or the checksum is invalid.

The EEPROM checksum will become invalid when a KlausBMS update introduces new state variables, so it's a good idea to note the last state before installing an update.


## SOC hybrid algorithm parameters

The effectice pack SOC is calculated from both pack voltage and coulomb (Ah) counting. The pack voltage marks the absolute limits of the battery for "empty" and "full", but is a bad base for the SOC in between. Especially LiFe chemistry has a nearly constant voltage between 20 and 80%. Coulomb counting alone on the other hand would introduce cumulating measurement errors.

So the algorithm uses coulomb counting most of the time, but prioritizes the voltage measurement over the coulomb counting when approaching 0% or 100% relative voltage and/or degrades the coulomb count when approaching 0% or 100% respectively.

This process is defined by the voltage ranges and the threshold values:

  - Voltage range discharging / charging (commands `vrd` and `vrc`)
  - SOC voltage prioritize top / bottom (command `svp`)
  - SOC coulomb degrade top / bottom (command `scd`)

The ranges and thresholds apply depending on the state: the "top" thresholds apply on charging/recuperating, the "bottom" ones on discharging.

The voltage thresholds always take precedence over the coulomb degradation, as voltage is a hard measurement.

The effects of wrong/bad voltage ranges and/or thresholds parameters are nonlinear SOC slopes. If you encounter sudden jumps or fast changes when approaching 0/100%, you need to adjust the SOC parameters.

Keep in mind you can do manual corrections of the SOC by issuing `soc`.


## SOH measurement

The SOH is the percentage of the measured usable capacity in relation to the nominal capacity as configured (`CAP_NOMINAL_AH`).

The usable capacity measurement is again based on the coulomb counting, so a properly calibrated current sensor is crucial.

The system adjusts the usable capacity at every charge stop after charging at least 50% SOC difference. The adjustment is calculated based on the difference between the charge amount reflecting the SOC change and the actual measured charge amount. The difference is applied smoothed by `SMOOTH_CAP` and weighted by the SOC difference charged: the more you charge, the more weight is put on the new measurement.

The results are written to the serial ports like this:

    bms.enterState: capacity/SOH adjustment:
    - SOC charged = 72%
    - expected Ah = 86.4
    -  charged Ah = 84.7
    -  old cap Ah = 120.0
    -  new cap Ah = 119.2
    -   new SOH % = 99.3

Keep in mind you can do manual corrections of the SOH by issuing `soh`.


## Author

KlausBMS has been written and is maintained by Michael Balzer (<dexter@dexters-web.de> / https://dexters-web.de/).

A lot of ideas and refinements have been contributed by Błażej Błaszczyk, who has been using this software for multiple Twizy battery replacements beginning with the [Build #2 using Nissan Leaf cells](https://github.com/dexterbg/Twizy-Virtual-BMS/blob/master/extras/Blazej2-Leaf-Cells.md), and by Klaus Zinser, who is using the system for his 120 Ah LiFePO4 battery pack and heads for an upgrade to 240 Ah.


## Donations

**Donations** to support my efforts and further development are very welcome.  
Please send donations via **Paypal** to: `dexter@dexters-web.de`  
**Thanks! :-)**


## License

This is free software; you can redistribute it and/or modify it under the terms of the [GNU Lesser General Public License](https://www.gnu.org/licenses/lgpl.html) as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with this software; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110- 1301  USA


**Have fun!**
