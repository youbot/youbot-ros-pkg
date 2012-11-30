/*
 * youbot_battery_monitor.h
 *
 *  Created on: Nov 30, 2012
 *      Author: Frederik Hegger, Jan Paulus
 */

#include "youbot_battery_monitor.h"

using namespace youbot;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "youbot_battery_monitor");

	YoubotBatteryMonitor* yb_battery_monitor = new YoubotBatteryMonitor();

	if (argc != 4)
	{
	      std::cout << "invalid arguments \n ./youbot_battery_monitor 'serial port' 'ethernet' 'wlan'" << std::endl;
	      return 0;
	}

	// if connecting fails, retry every 2 seconds
	do
	{
		std::cout << "try to connect to serial port: " << argv[1] << std::endl;
		sleep(2);
	}while(!yb_battery_monitor->connect(argv[1]));


	while(true)
	{
		yb_battery_monitor->publishStatusInformation(argv[2], argv[3]);
		sleep(2);
	}

	yb_battery_monitor->disconnect();

	return (0);
}

