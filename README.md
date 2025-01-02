# Fenix_ICM_20948_Cal
 Calibration of Fenix autopilot using IMU ICM_20948

# Installation guide:
Prerequisites:
This is a Python 3.12.4 application. Python IDLE 3.12 has to be installed in your computer.

To install the different modules required type in cmd: 
pip install tk
pip install pyserial

# Quick Start Guide:
1. Connect Autopilot with IMU configuration: ICM_20948 (Sparkfun)
2. Press "Scan Serial Ports" to search for a COM port where Fenix is connected. Press "Connect".
3. "Press receive raw data from sensors". For each sensor a different procedure shall be followed to collect raw data.  Raw data will be stored in files automatically.
G - Gyroscope: Keep sensor steady and accumulate some data (10 points should be enough). Close the graphics window.
A - Accelerometer: Turn sensor to different positions, keeping each steady for a few seconds. All moves shall be smooth to avoid drifting values. About 300 points should be enough.Close the graphics window.
M - Magnetometer: Turn sensor 360º to point at the different cardinal points. About 300 points should be enough. Close the graphics window.
4. Press "Calculate offsets". One graphics window per sensor will be opened with the raw data stored in the files. Close each window to generate the required offsets. A NMEA sentence per sensor will be stored in new files.
5. Press "Send offsets to autopilot". 3 NMEA sentences (1 per sensor) will be sent to Fenix autopilot.
6. Press "Save offsets sent to autopilot". Offsets will be permanently stored into Fenix autopilot.
