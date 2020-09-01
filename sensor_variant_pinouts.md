Lura Health pivoted from producing just an intra-oral pH sensor to producing
intra-oral pH, Na+, and K+ temperature sensors, and an analyte array which can
sense pH, Na+ and K+ at the same time. Lura Health also pivoted from producing
one form factor of the tooth-mounted sensor to producing a buccal tooth-mounted
sensor, a buccal and buccal-lingual tooth-mounted sensor, a sensor mounted on
a clear lower retainer, and a sensor embedded within a Hawley retainer.

As such, firmware needs to be expanded to handle varying BLE packet formats,
calibration protocols, and temperature compensations across the varying
analytes sensors. Firmware also needs to be adapted to handle variations in
GPIO mappings across all PCB versions. This document lists GPIO mappings.

**R_WIN_A**

        V_TEMP (thermistor output) : P0.03/AIN1
        VOUT (sensor output)       : P0.04/AIN2
        BT_ADC (battery voltage)   : P0.05/AIN3
        PWR_EN (LDO enable logic)  : P0.08
        POWER_ON (switch logic)    : P0.12
       
**R_WIN_L**

        V_TEMP (thermistor output) : P0.03/AIN1
        VOUT (sensor output)       : P0.04/AIN2
        BT_ADC (battery voltage)   : P0.05/AIN3
        PWR_EN (LDO enable logic)  : P0.08
        POWER_ON (switch logic)    : P0.12

**R_MICRO_A**

        V_TEMP (thermistor output) : P0.03/AIN1
        VOUT (sensor output)       : P0.04/AIN2
        BT_ADC (battery voltage)   : P0.05/AIN3
        PWR_EN (LDO enable logic)  : P0.08
        POWER_ON (switch logic)    : P0.12

**R_WEL_A**

        *In progress*

**R_IMEC_A**

        V_TEMP (thermistor output) : P0.03/AIN1
        VOUT (sensor output)       : P0.04/AIN2
        BT_ADC (battery voltage)   : P0.05/AIN3
        PWR_EN (LDO enable logic)  : P0.08
        POWER_ON (switch logic)    : P0.12
        A0 (TMUX logic, LSB)       : P0.18
        A1 (TMUX logic, MSB)       : P0.17
        1104 EN (TMUX enable)      : P0.16
        Ind. electrode 1           : A1: 0 | A0 0
        Ind. electrode 2           : A1: 0 | A0 1
        Ind. electrode 3           : A1: 1 | A0 0
        Ind. electrode 4           : A1: 1 | A0 1

**M1_WIN_A**

        V_TEMP (thermistor output) : P0.03/AIN1
        VOUT (sensor output)       : P0.04/AIN2
        BT_ADC (battery voltage)   : P0.05/AIN3
        PWR_EN (LDO enable logic)  : P0.08
        POWER_ON (switch logic)    : P0.12

**M1_WIN_L**

        V_TEMP (thermistor output) : P0.03/AIN1
        VOUT (sensor output)       : P0.04/AIN2
        BT_ADC (battery voltage)   : P0.05/AIN3
        PWR_EN (LDO enable logic)  : P0.08
        POWER_ON (switch logic)    : P0.12

**M1_TEMP_L**

        V_TEMP (thermistor output) : P0.03/AIN1
        BT_ADC (battery voltage)   : P0.05/AIN3
        PWR_EN (LDO enable logic)  : P0.08
        POWER_ON (switch logic)    : P0.12

**M2_WIN_A**

        *In progress*

**M2_WIN_L**

        *In progress*

**M2_MICRO_A**

        *In progress*

**M2_WEL_A**

        *In progress*

**M2_TEMP_L**

        *In progress*

**H_WIN_AL**

        *In progress*

**H_IMEC_AL**

        V_TEMP (thermistor output) : P0.03/AIN1
        VOUT (sensor output)       : P0.04/AIN2
        BT_ADC (battery voltage)   : P0.05/AIN3
        PWR_EN (LDO enable logic)  : P0.08
        POWER_ON (switch logic)    : P0.12
        A0 (TMUX logic, LSB)       : P0.18
        A1 (TMUX logic, MSB)       : P0.17
        1104 EN (TMUX enable)      : P0.16
        Ind. electrode 1           : A1: 0 | A0 0
        Ind. electrode 2           : A1: 0 | A0 1
        Ind. electrode 3           : A1: 1 | A0 0
        Ind. electrode 4           : A1: 1 | A0 1
