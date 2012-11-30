#!/bin/bash
export ROS_MASTER_URI=http://localhost:11311
/etc/youbot/youbot_battery_monitor /dev/youbot/lcd_display eth1 wlan0 &
