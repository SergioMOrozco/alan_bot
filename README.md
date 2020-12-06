# alan_bot

## Description
The objective of this project is to create a sandbox environment intended for implementing various methods of automated driving, computer vision, localization and mapping all in one robot.

## Required Hardware
- Raspberry Pi (preferably 4 Model B)
- Arduino (Uno is fine)
- 2 DC Motors

## Required Software
- arduino-cli (with ros_lib installed)
- ROS Noetic (with rosserial and rosserial-arduino package installed)
## Instructions To Get Started
- run ```catkin_make``` in alan_ws.
- compile and upload alan_wheels to your arduino:
```
//Compile Example:
arduino-cli compile -b arduino:avr:uno ~/alan_bot/alan_controllers/alan_wheels/alan_wheels.ino

//Upload Example: 
arduino-cli upload -b arduino:avr:uno -p /dev/ttyACM0 ~/alan_bot/alan_controller/alan_wheels/alan_wheels.ino
```

- You should now be able to move your robot using the ```alan_core``` package found in ```alan_ws```.
