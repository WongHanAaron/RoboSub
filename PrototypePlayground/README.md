# PrototypePlayground

PrototypePlayground is a project where new members can learn the basics of Exodus and seasoned members can develop new drivers.

* 1. **.launches**: Holds the Launch Configuration for this project using Code Composer Studio
* 2. **.settings**: Settings for Code Composer Studio
* 3. **targetConfigs**: Target Configuration for flashing the code onto the TI
* 4. **main.cpp**: The file that contains the "main" function and provides a "bare-bones" version of Exodus to mess around with
* 5. **Libraries**: TivaWare Libraries to interact with the TI
* 5. (a) **driverlib.lib**: Library to interact with the TI's functionality (PWM, ADC, Timers, etc.)
* 5. (b) **sensorlib.lib**: Library to interact with specified sensors and configure the microcontroller for specific I2C functionalities
* 6. **Miscellaneous**: Files that pertain to running the code composer project
* 6. (a) **.ccsproject**
* 6. (b) **.cproject**
* 6. (c) **.project**
* 6. (d) **tm4c123gh6pm.cmd**
* 6. (e) **tm4c123gh6pm_startup_ccs.c**
