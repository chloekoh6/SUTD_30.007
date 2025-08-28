# SUTD_30.007

Codes to control a Raspberry Pi that controls a wall cleaning robot and interacts with an ESP32 to control the position of the robot. 

Run the web.py file in a Python environment for Raspberry Pi, as some extensions are not available on Raspberry Pi.
The Arduino Code in the Only_Y_axis1 folder is for the ESP32, which controls the winches.
Website UI is in index.html file in the templates folder.

Init_actuator.py is a file to initialize a database for the actuator, which can be used for initializing a stepper database as well.
