#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#include <locale.h>

#if defined(__APPLE__) && !defined(NOFRAMEWORK)
// include path to framework
#include <libximc/ximc.h>
#else
#include <ximc.h>
#endif

#ifndef _WIN32
#include <syslog.h>
#else
#include <io.h>
#include <fcntl.h>
#endif

void print_state(status_t *state)
{
	wprintf(L" rpm: %d\n", state->CurSpeed);
	wprintf(L" pos: %d\n", state->CurPosition);
	wprintf(L" upwr: %d\n", state->Upwr);
	wprintf(L" ipwr: %d\n", state->Ipwr);
	wprintf(L" flags: %x\n", state->Flags);
	wprintf(L" mvsts: %x\n", state->MvCmdSts);
	if (state->Flags & STATE_ALARM)
		wprintf(L" ALARM");
	if (state->Flags & STATE_ERRC)
		wprintf(L" ERRC");
	if (state->Flags & STATE_ERRD)
		wprintf(L" ERRD");
	wprintf(L"\n");
}

const wchar_t *error_string(result_t result)
{
	switch (result)
	{
	case result_error:
		return L"error";
	case result_not_implemented:
		return L"not implemented";
	case result_nodevice:
		return L"no device";
	default:
		return L"success";
	}
}

const wchar_t *loglevel_string(int loglevel)
{
	switch (loglevel)
	{
	case LOGLEVEL_ERROR:
		return L"ERROR";
	case LOGLEVEL_WARNING:
		return L"WARN";
	case LOGLEVEL_INFO:
		return L"INFO";
	case LOGLEVEL_DEBUG:
		return L"DEBUG";
	default:
		return L"UNKNOWN";
	}
}

#ifndef _WIN32
int loglevel_to_sysloglevel(int loglevel)
{
	switch (loglevel)
	{
	case LOGLEVEL_ERROR:
		return LOG_ERR;
	case LOGLEVEL_WARNING:
		return LOG_WARNING;
	case LOGLEVEL_INFO:
		return LOG_INFO;
	case LOGLEVEL_DEBUG:
		return LOG_DEBUG;
	default:
		return LOG_INFO;
	}
}
#endif

char *widestr_to_str(const wchar_t *str)
{
	char *result;
	mbstate_t mbs;
	size_t len;
	memset(&mbs, 0, sizeof(mbs));
	len = wcsrtombs(NULL, &str, 0, &mbs);
	if (len == (size_t)(-1))
		return NULL;
	result = malloc(sizeof(char) * (len + 1));
	if (result && wcsrtombs(result, &str, len + 1, &mbs) != len)
	{
		free(result);
		return NULL;
	}
	return result;
}

void XIMC_CALLCONV my_logging_callback(int loglevel, const wchar_t *message, void *user_data)
{
	wchar_t wbuf[2048];
	char *abuf;
	(void)user_data;
	int used_loglevel = user_data ? *((int *)user_data) : LOGLEVEL_DEBUG;
	if (loglevel > used_loglevel)
		return;

	/* Print to console unicode chars */
	swprintf(wbuf, sizeof(wbuf) / sizeof(wbuf[0]) - 1, L"XIMC %ls: %ls", loglevel_string(loglevel), message);
	fwprintf(stderr, L"%ls\n", wbuf);

#ifdef _WIN32
	(void)abuf;
#else
	/* Print to syslog ANSI chars */
	abuf = widestr_to_str(wbuf);
	if (abuf)
	{
		syslog(loglevel_to_sysloglevel(loglevel), "%s", abuf);
		free(abuf);
	}
#endif
}

int main(int argc, char *argv[])
{
	device_t yStage, xStage;
	engine_settings_t engine_settings;
	engine_settings_calb_t engine_settings_calb;
	calibration_t calibration;
	device_information_t di;
	status_t state;
	result_t result;
	get_position_t getPosition;
	get_position_calb_t getPositionCalb;
	int names_count;
	char yStage_name[256];
	char xStage_name[256];
	int i;
	int device_specified = 0;
	const int probe_flags = ENUMERATE_PROBE | ENUMERATE_ALL_COM;
	const char *enumerate_hints = NULL;
	char ximc_version_str[32];

	device_enumeration_t devenum;

	/* Inherit system locale */
	setlocale(LC_ALL, "");
#ifdef _MSC_VER
	/* UTF-16 output on windows */
	_setmode(_fileno(stdout), _O_U16TEXT);
	_setmode(_fileno(stderr), _O_U16TEXT);
#endif

	int used_loglevel = getenv("XIMC_TESTAPP_VERBOSE") ? LOGLEVEL_DEBUG : LOGLEVEL_WARNING;
	set_logging_callback(my_logging_callback, &used_loglevel);
	ximc_version(ximc_version_str);

	wprintf(L"libximc version %hs\n", ximc_version_str);
	wprintf(L"I am %d bit\n", sizeof(int *) == 4 ? 32 : 64);

	//  Set bindy (network) keyfile. Must be called before any call to "enumerate_devices" or "open_device" if you
	//  wish to use network-attached controllers. Accepts both absolute and relative paths, relative paths are resolved
	//  relative to the process working directory. If you do not need network devices then "set_bindy_key" is optional.
	// set_bindy_key( "keyfile.sqlite" );

	devenum = enumerate_devices(probe_flags, enumerate_hints);
	if (!devenum)
	{
		wprintf(L"error enumerating devices\n");
		names_count = 0;
	}

	names_count = get_device_count(devenum);
	if (names_count == -1)
	{
		wprintf(L"error enumerating device\n");
		names_count = 0;
	}

	wprintf(L"\n");
	for (i = 0; i < names_count; ++i)
	{
		wprintf(L"device%d: %hs\n", i, get_device_name(devenum, i));
	}
	wprintf(L"\n");

	//----------------------------------------------------> X&Y Stage <----------------------------------------------------//
	home_settings_calb_t xHomeSettings, yHomeSettings;
	status_calb_t xStatus, yStatus;
	uint32_t visualizatonPeriod = 100;

	calibration.A = 0.015; // 1 step = 0.015 degree
	calibration.MicrostepMode = MICROSTEP_MODE_FULL;

	//----------------------------------------------------> X Stage Home Command<----------------------------------------------------//
	strcpy(xStage_name, get_device_name(devenum, 0));
	xStage = open_device(xStage_name);

	if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	wprintf(L"Current positon: %.3f°\n", xStatus.CurPosition);
	wprintf(L"Engine Current: %.2f A\n", xStatus.Ipwr / 100.0);
	wprintf(L"Power Voltage: %.2f V\n", xStatus.Upwr / 100.0);
	wprintf(L"Temperature: %.1f°C\n\n", xStatus.CurT / 10.0);

	if ((result = get_home_settings_calb(xStage, &xHomeSettings, &calibration)) != result_ok)
		wprintf(L"error getting home settings %ls\n\n", error_string(result));

	xHomeSettings.HomeDelta = -3.0; //Difference between home position and zero mark at the stage

	if ((result = set_home_settings_calb(xStage, &xHomeSettings, &calibration)) != result_ok)
		wprintf(L"error setting home settings %ls\n\n", error_string(result));

	wprintf(L"FastHome: %f\n", xHomeSettings.FastHome);
	wprintf(L"SlowHome: %f\n", xHomeSettings.SlowHome);
	wprintf(L"HomeDelta: %f\n", xHomeSettings.HomeDelta);
	wprintf(L"HomeFlags: %x\n\n", xHomeSettings.HomeFlags);

	if ((result = command_home(xStage)) != result_ok)
		wprintf(L"error command home  %ls\n\n", error_string(result));

	if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	while (xStatus.MvCmdSts == (MVCMD_HOME | MVCMD_RUNNING))		//Wait homming stop
	{
		get_status_calb(xStage, &xStatus, &calibration);
		msec_sleep(visualizatonPeriod);
	}

	if ((result = command_zero(xStage)) != result_ok)
		wprintf(L"error command zero  %ls\n\n", error_string(result));

	//----------------------------------------------------> Y Stage Home Command<----------------------------------------------------//
	strcpy( yStage_name, get_device_name( devenum, 1 ) );
	yStage = open_device( yStage_name );

	if ((result = get_status_calb(yStage, &yStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	wprintf(L"Current positon: %.3f°\n", yStatus.CurPosition);
	wprintf(L"Engine Current: %.2f A\n", yStatus.Ipwr / 100.0);
	wprintf(L"Power Voltage: %.2f V\n", yStatus.Upwr / 100.0);
	wprintf(L"Temperature: %.1f°C\n\n", yStatus.CurT / 10.0);

	if ((result = get_home_settings_calb(yStage, &yHomeSettings, &calibration)) != result_ok)
		wprintf(L"error getting home settings %ls\n\n", error_string(result));

	xHomeSettings.HomeDelta = 0.555; //Difference between home position and zero mark at the stage

	if ((result = set_home_settings_calb(yStage, &yHomeSettings, &calibration)) != result_ok)
		wprintf(L"error setting home settings %ls\n\n", error_string(result));

	wprintf(L"FastHome: %f\n", yHomeSettings.FastHome);
	wprintf(L"SlowHome: %f\n", yHomeSettings.SlowHome);
	wprintf(L"HomeDelta: %f\n", yHomeSettings.HomeDelta);
	wprintf(L"HomeFlags: %x\n\n", yHomeSettings.HomeFlags);

	if ((result = command_home(yStage)) != result_ok)
		wprintf(L"error command home  %ls\n\n", error_string(result));

	if ((result = get_status_calb(yStage, &yStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));
	wprintf(L"Command: %x\n", yStatus.MvCmdSts);

	while (yStatus.MvCmdSts == (MVCMD_HOME | MVCMD_RUNNING))		//Wait homming stop
	{
		get_status_calb(yStage, &yStatus, &calibration);
		msec_sleep(visualizatonPeriod);
	}

	if ((result = command_zero(yStage)) != result_ok)
		wprintf(L"error command zero  %ls\n\n", error_string(result));

	//----------------------------------------------------> Y Stage Take Starting Position<----------------------------------------------------//

	if ((result = command_move_calb(yStage, 60, &calibration)) != result_ok)
		wprintf(L"error command_movr %ls\n", error_string(result));

	if ((result = get_status_calb(yStage, &yStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	while (yStatus.MvCmdSts == (MVCMD_MOVE | MVCMD_RUNNING))
	{
		get_position_calb(yStage, &getPositionCalb, &calibration);
		wprintf(L"yPosition: %.3f°\n", getPositionCalb.Position);

		if ((result = get_status_calb(yStage, &yStatus, &calibration)) != result_ok)
			wprintf(L"error getting status %ls\n", error_string(result));

		msec_sleep(visualizatonPeriod);
	}

	//----------------------------------------------------> X Stage Scan Horizontal Row<----------------------------------------------------//

	if ((result = command_move_calb(xStage, -30, &calibration)) != result_ok)
		wprintf(L"error command_movr %ls\n", error_string(result));

	if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	while (xStatus.MvCmdSts == (MVCMD_MOVE | MVCMD_RUNNING))
	{
		get_position_calb(xStage, &getPositionCalb, &calibration);
		wprintf(L"xPosition: %.3f°\n", getPositionCalb.Position);

		if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
			wprintf(L"error getting status %ls\n", error_string(result));

		msec_sleep(visualizatonPeriod);
	}

	msec_sleep(2000);

	if ((result = command_move_calb(xStage, 30, &calibration)) != result_ok)
		wprintf(L"error command_movr %ls\n", error_string(result));

	if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	while (xStatus.MvCmdSts == (MVCMD_MOVE | MVCMD_RUNNING))
	{
		get_position_calb(xStage, &getPositionCalb, &calibration);
		wprintf(L"xPosition: %.3fº\n", getPositionCalb.Position);

		if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
			wprintf(L"error getting status %ls\n", error_string(result));

		msec_sleep(visualizatonPeriod);
	}

	msec_sleep(2000);

	if ((result = command_move_calb(xStage, -30, &calibration)) != result_ok)
		wprintf(L"error command_movr %ls\n", error_string(result));

	if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	while (xStatus.MvCmdSts == (MVCMD_MOVE | MVCMD_RUNNING))
	{
		get_position_calb(xStage, &getPositionCalb, &calibration);
		wprintf(L"xPosition: %.3fº\n", getPositionCalb.Position);

		if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
			wprintf(L"error getting status %ls\n", error_string(result));

		msec_sleep(visualizatonPeriod);
	}

	//----------------------------------------------------> Y Stage Starting Second Scan Row<----------------------------------------------------//

	if ((result = command_move_calb(yStage, 45, &calibration)) != result_ok)
		wprintf(L"error command_movr %ls\n", error_string(result));

	if ((result = get_status_calb(yStage, &yStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	while (yStatus.MvCmdSts == (MVCMD_MOVE | MVCMD_RUNNING))
	{
		get_position_calb(yStage, &getPositionCalb, &calibration);
		wprintf(L"yPosition: %.3f°\n", getPositionCalb.Position);

		if ((result = get_status_calb(yStage, &yStatus, &calibration)) != result_ok)
			wprintf(L"error getting status %ls\n", error_string(result));

		msec_sleep(visualizatonPeriod);
	}

	//----------------------------------------------------> X Stage Scan Horizontal Row<----------------------------------------------------//

	if ((result = command_move_calb(xStage, -30, &calibration)) != result_ok)
		wprintf(L"error command_movr %ls\n", error_string(result));

	if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	while (xStatus.MvCmdSts == (MVCMD_MOVE | MVCMD_RUNNING))
	{
		get_position_calb(xStage, &getPositionCalb, &calibration);
		wprintf(L"xPosition: %.3f°\n", getPositionCalb.Position);

		if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
			wprintf(L"error getting status %ls\n", error_string(result));

		msec_sleep(visualizatonPeriod);
	}

	msec_sleep(2000);

	if ((result = command_move_calb(xStage, 30, &calibration)) != result_ok)
		wprintf(L"error command_movr %ls\n", error_string(result));

	if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	while (xStatus.MvCmdSts == (MVCMD_MOVE | MVCMD_RUNNING))
	{
		get_position_calb(xStage, &getPositionCalb, &calibration);
		wprintf(L"xPosition: %.3fº\n", getPositionCalb.Position);

		if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
			wprintf(L"error getting status %ls\n", error_string(result));

		msec_sleep(visualizatonPeriod);
	}

	msec_sleep(2000);

	if ((result = command_move_calb(xStage, 0, &calibration)) != result_ok)
		wprintf(L"error command_movr %ls\n", error_string(result));

	if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	while (xStatus.MvCmdSts == (MVCMD_MOVE | MVCMD_RUNNING))
	{
		get_position_calb(xStage, &getPositionCalb, &calibration);
		wprintf(L"xPosition: %.3fº\n", getPositionCalb.Position);

		if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
			wprintf(L"error getting status %ls\n", error_string(result));

		msec_sleep(visualizatonPeriod);
	}


	if ((result = get_status_calb(xStage, &xStatus, &calibration)) != result_ok)
		wprintf(L"error getting status %ls\n", error_string(result));

	wprintf(L"\n");
	wprintf(L"Current positon: %.3f°\n", xStatus.CurPosition);
	wprintf(L"Engine Current: %.2f A\n", xStatus.Ipwr / 100.0);
	wprintf(L"Power Voltage: %.2f V\n", xStatus.Upwr / 100.0);
	wprintf(L"Temperature: %.1f°C\n\n", xStatus.CurT / 10.0);

	if ((result = close_device(&xStage)) != result_ok)
		wprintf(L"error closing device %ls\n", error_string(result));

	wprintf(L"Done\n");

	return 0;
}