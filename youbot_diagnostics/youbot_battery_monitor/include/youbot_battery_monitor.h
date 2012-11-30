/*
 * youbot_battery_monitor.h
 *
 *  Created on: Nov 30, 2012
 *      Author: Frederik Hegger, Jan Paulus
 */

#ifndef YOUBOT_BATTERY_MONITOR_H_
#define YOUBOT_BATTERY_MONITOR_H_

#include <ros/ros.h>
#include <pr2_msgs/PowerState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <iostream>
#include <string>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdlib>


#define MIN_VOLTAGE     				18  // Volt
#define MAX_VOLTAGE    					25  // Volt

#define BATTERY_PERCENTAGE_THRESHOLD    30

namespace youbot
{

enum DisplayLine { line2 = 0x02, line3 = 0x03 };
enum VoltageSource { battery1 = 0x04, battery2 = 0x05, powersupply = 0x0c };

class YoubotBatteryMonitor
{
public:
	YoubotBatteryMonitor();
	~YoubotBatteryMonitor();

	bool connect(std::string port);
	bool disconnect();

	/* returns the voltage in [Volt] */
	double getVoltage(VoltageSource source);

	/* set text on the youBot LCD display */
	bool setYoubotDisplayText(DisplayLine line, std::string text);

	/* returns the IP address for given device names */
	void getIPAddresses(std::string lan_name, std::string wlan_name, std::string& lan_ip, std::string& wlan_ip);

	void publishStatusInformation(std::string lan_device_name, std::string wlan_device_name);

private:
	void configureSerialPort();

	bool ros_node_initialized_;
	ros::NodeHandle* nh_;

	ros::Publisher pub_dashboard_battery_status_;
	ros::Publisher pub_diagnostics_;

	pr2_msgs::PowerState dashboard_battery_message_;
	diagnostic_msgs::DiagnosticArray diagnostic_array_;
	diagnostic_msgs::DiagnosticStatus diagnostic_state_;

	int serial_file_description_;
	bool is_connected_;
};

} /* namespace youbot */
#endif /* YOUBOT_BATTERY_MONITOR_H_ */
