#include "PanTilt.h"



PanTilt::PanTilt()
{
	_deviceSpecified = 0;
	_calibration.A = 0.015; // 1 step = 0.015 degree
	_calibration.MicrostepMode = MICROSTEP_MODE_FULL;
}

void PanTilt::init() 
{
	int i;
	std::string str;
	_devenum = enumerate_devices(probe_flags, enumerate_hints);
	if (!_devenum)
	{
		_nh.logerror("error enumerating devices");
		wprintf(L"error enumerating devices\n");
		_namesCount = 0;
	}

	_namesCount = get_device_count(_devenum);

	if (_namesCount == -1)
	{
		_nh.logerror("error enumerating device");
		wprintf(L"error enumerating device\n");
		_namesCount = 0;
	}

	wprintf(L"\n");

	for (i = 0; i < _namesCount; ++i)
	{
		str = "device" + std::to_string(i) + get_device_name(_devenum, i);
		_nh.loginfo(str.c_str());
		wprintf(L"device%d: %hs\n", i, get_device_name(_devenum, i));
	}
	wprintf(L"\n");
}

PanTilt::~PanTilt()
{
}
