# obd-tracker-imp
Electric Imp device and agent code for logging realtime OBD and GPS data to Firebase

#### obd-tracker.device.nut

Device code for logging data from OBD, GPS and accelerometer to the agent.
Logged data includes:
- Vehicle speed
- Engine RPM
- Throttle position
- Intake air temperature
- Coolant temperature
- Fuel pressure
- Accelerometer x,y,z
- GPS lat/lon

#### obd-tracker.device.nut

Agent code for receiving data from the device and posting to Firebase.
Configure with your Firebase name and key.