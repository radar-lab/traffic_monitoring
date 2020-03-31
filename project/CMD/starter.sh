#!/bin/sh

# Start the radar
sudo chmod 666 /dev/ttyACM0 & 
sudo chmod 666 /dev/ttyACM1

xterm -title "Radar Module" -e "source ~/.bashrc; cd ~/FJ/traffic_monitoring_debug/ti_ros; roslaunch traffic_monitoring traffic_monitoring.launch"&

sleep 5
