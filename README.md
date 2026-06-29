# IndiFlight-Mark-1---Flight-Computer
A arduino based Flight Data Logger + Recovery System for hobby rockets


Version 1.2 FC_Code Known Bugs (unresolved)

##Known Bugs
**Bug 1: Incorrect Fail-Safe Trigger on Pyro Circuit Continuity**

Description: The Flight Computer’s pyro line fail-safe is incorrectly triggering during open-continuity checks.

Details: The system currently interprets the connection of the pyro line battery as a condition for continuity. However, the continuity check should rely solely on the ignitor circuit integrity. The battery state is currently influencing the continuity logic, leading to false-positive triggers for open-circuit faults. As per the design, the battery line should have no correlation with the pyro line continuity sensing. 

**Bug 2: Infinite Loop During Sensor Disconnection in Logging State**

Description: The system enters an unrecoverable infinite loop if a sensor is disconnected while the flight computer is in the "Logging" state.

Details: Following the successful completion of the self-test, a sensor failure results in a system hang. This is characterized by the rapid flashing of the status LED (Arduino Nano) and the flash storage module, accompanied by a complete cessation of serial terminal data output. The system fails to implement an error-handling routine or recovery mechanism, requiring a hard reset.

**Bug 3: Lack of Data Logging in IDLE State**

Description: The system fails to initiate or record data telemetry while in the "IDLE" operational mode.

Details: Data logging functionality is currently inactive when the device is in the IDLE state. Please verify if this is intended behavior or if logging should be active across all operational states to ensure comprehensive diagnostic coverage.
