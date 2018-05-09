# precision_landing
Repository containing python code used for precision landing a quad copter. 

The main four files for running the code are Image_Processor, MyCopter, guided_land_nogps_no_bar, and target.

target.py: contains the physical characteristics of each of the targets including sizes and physical locations with respect to the true target

guided_land_no_gps_no_bar.py: The 'main' function of a typical flight test. Has several helper methods for different types of tests I would like to conduct. Creates instances of the copter and image_processor classes. Has debug mode and 'indoor' (safe, props dont spin) mode. Usage: python guided_land_no_gps_no_bar.py

Image_Processor.py: Contains the code used for image processing. The main function is loop() which checks for targets and (potentially) tells the MyCopter class to send a landing command over the MAVlink. There are lots of different options for flight embedded in this class, the most important of which are the image resolutions and lots of arbitrary confidence parameters we have set.

MyCopter.py: Contains code responsible for sending commands to the drone and interfacing with the dronekit library. Whenever I figured I was going to be regularly sending a certain type of command to the drone, I added it as a method in here.




Also included in this directory is:
ArduCopter: cloned from the original repository (https://github.com/ardupilot/Ardupilot). I made edits to this library as I saw necessary. This is cloned from an old version and ArduCopter gets updated very often so it is probably deprecated in some parts.

chessboard: these directories contain chessboards used in camera calibration. Probably not very useful anymore. 

old_stuff: This directory contains an ammalgation of random files we used as tests over the year. For example, we made a script to test the frequency with which the pi got new barometer readings from the Pixhawk (it was ~8Hz). The most notable set of scripts in here are the multi_processing ones which could be useful if the opencv people decide to update their code so that it works with multi_processing....

boards: This directory contains the target boards we used frequently, placed here for convenient printing.