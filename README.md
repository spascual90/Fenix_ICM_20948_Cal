# Fenix_ICM_20948_Cal
 Calibration of Fenix autopilot using IMU ICM_20948

# Installation guide:
Prerequisites:
This is a Python 3.12.4 application. Python IDLE 3.12 has to be installed in your computer.

To install the different modules you need:

1. Open the command prompt (in Windows, go to the Start Menu and type "cmd").
2. Run pip for each module you need to install:

pip install tk

pip install pyserial

pip install numpy

pip install matplotlib

pip install scipy

Another way to install is to use requirements.txt file to install everything.
1. Open the command prompt (in Windows, go to the Start Menu and type "cmd").
2. Change to the project directory (for example, type "cd C:\Workspaces\Fenix_ICM_20948_Cal").
3. Run pip with the requirements.txt file (type "pip -r requirements.txt").

# Quick Start Guide:
1. Connect Autopilot with IMU configuration: ICM_20948 (Sparkfun)
2. Press "Scan Serial Ports" to find a COM port where Fenix is connected. Press "Connect".
3. Then press "Receive raw data from sensors". Each sensor has its own way of collecting raw data.  The raw data will be stored in files automatically.

- G - Gyroscope: Keep the sensor steady and collect some data (10 points should be enough). Then close the graphics window.

- A - Accelerometer: Turn the sensor to different positions, keeping each steady for a few seconds. Make sure you move smoothly to avoid inaccurate results. About 300 points should be enough.Close the graphics window.

- M - Magnetometer: Turn the sensor 360º to point at the different cardinal points. About 300 points should be enough. Finally, close the graphics window.

4. Press "Calculate offsets". This will open one graphics window for each sensor, storing the raw data in the files. Close each window to generate the required offsets. Each sensor will now have a new file with an NMEA sentence.
5. Then press "Send offsets to autopilot". This will send three NMEA sentences (one for each sensor) to the Fenix autopilot.
6. Press "Save offsets sent to autopilot". The offsets will then be stored permanently in the Fenix autopilot.
