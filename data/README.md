## cfg
The cfg folder contains configuration files to show the data in Robotic Operating System (ROS) Rviz.

## training
The training folder contains data for training. The data is in ROS .bag format. The training data does not include video to save the disk space. As our method is unsupervised approach, so we do not need ground truth during trainning.

## testing
The testing folder contains data for test. The testing data contains video as ground truth.

## data_intersection
The real-world data collected at a traffic intersection. The total data time duration is about 45 mins. Both the camera video and radar data were saved.

## Data Collection Instructions
1.	Set up a WiFi router to build a local area network (LAN).
2.	Plug in the WiFi module to Nvidia Jetson Nano.
3.	At the first time, power up Nvidia Jetson Nano using HDMI LCD screen, keyboard and mouse. And connect to the WiFi router using correct password. And then install the NoMachine software, which is for remote control, on Nvidia Jetson Nano. Set NoMachine to automatically start when the system starts. Note that, this step is only once. 
4.	Power off Nvidia Jetson Nano and remove LCD screen, keyboard and mouse, but keep the WiFi module on it.
5.	Connect the TI AWR1843BOOST radar sensor and camera to the Nvidia Jetson Nano through USB cable.
6.	Power on Nvidia Jetson Nano. Then it will automatically connect to the WiFi router using stored password.
7.	Power on radar sensor.
8.	On laptop side, install NoMachine software and connect to the same WiFi router.
9.	On laptop side, open NoMachine and search for the Nvidia Jetson Nano which is in the same LAN. Connect to Jetson Nano through NoMachine, and it will show a small screen of Jetson Nano.
10.	Now, you can control Jetson Nano remotely.
11.	To open the radar, execute the commands:
> sudo chmod 666 /dev/ttyACM0
> sudo chmod 666 /dev/ttyACM0
> roslaunch traffic_monitoring traffic_monitoring.launch
12.	To record the radar data on USB stick, you first need to insert the USB stick in Jetson Nano. And switch to USB stick folder. And then execute the command:
> rosbag record /traffic_monitoring/radar_scan /traffic_monitoring/radar_scan_markers /usb_webcam/image_raw/compressed -b -0 -O file_name.bag
