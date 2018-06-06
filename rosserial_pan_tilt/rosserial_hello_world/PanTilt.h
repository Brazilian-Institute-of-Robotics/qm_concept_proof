#include "stdafx.h"
#include <string>
#include <stdio.h>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <windows.h>

#include <stdlib.h>
#include <string>
#include <wchar.h>
#include <locale.h>
#include <../ximc/ximc.h>

#ifndef _WIN32
#include <syslog.h>
#else
#include <io.h>
#include <fcntl.h>
#endif

class PanTilt
{
	const int probe_flags = ENUMERATE_PROBE | ENUMERATE_ALL_COM;
	const char *enumerate_hints = NULL;

public:
	PanTilt();
	void init();
	~PanTilt();
private:
	device_t _yStage, _xStage;
	calibration_t _calibration;
	result_t _result;
	get_position_t _getPosition;
	home_settings_calb_t _xHomeSettings, _yHomeSettings;
	status_calb_t _xStatus, _yStatus;
	move_settings_calb_t _xMoveSettings;
	device_enumeration_t _devenum;

	ros::NodeHandle _nh;

	int _namesCount;
	char _yStageName[256];
	char _xStageName[256];
	int _deviceSpecified;
	char _ximcVersionStr[32];

};

