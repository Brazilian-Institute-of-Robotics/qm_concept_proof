# Pan-tilt station

## This step by step will show you how to run the "testapp.c" file

First, execute this command to install library dependencies:

```sh
sudo apt-get install ruby1.9.1 debhelper vim sudo g++ mercurial git curl make cmake autotools-dev automake autoconf libtool default-jre-headless default-jdk openjdk-6-jdk dpkg-dev lintian texlive texlive-latex extra texlive-lang-cyrillic dh-autoreconf hardening-wrapper bison flex doxygen lsb-release pkg-config
```

Then, in folder deb/, install debian packages

```sh
sudo dpkg -i libximc7_2.9.14-1_amd64.deb
sudo dpkg -i libximc7-dev_2.9.14-1_amd64.deb
```

Finally, inside testapp folder, execute these commands

```sh
make
```

```sh
./testapp
```

## Functions used and their descriptions

**Function:**

```C
device_enumeration_t XIMC_API enumerate_devices (int enumerate flags, const char ∗ hints)
```

**Description:** Enumerate all devices that looks like valid.

---

**Function:**

```C
int XIMC_API get_device_count (device enumeration t device enumeration)
```

**Description:** Get device count.

---

**Function:**

```C
device_t XIMC_API open_device (const char* uri)
```

**Description:** Open a device with OS uri \a uri and return identifier of the device which can be used in calls.

---

**Function:**

```C
pchar XIMC_API get_device_name (device enumeration t device enumeration, int device index)
```

**Description:** Get device name from the device enumeration.

**Function:**

```C
result_t XIMC_API get_status_calb (device t id, status calb t ∗ status, const calibration t ∗
calibration)
```

**Description:** Calibrated device state. Useful structure that contains current controller status, including speed, position and boolean flags.

---

**Function:**

```C
result_t XIMC_API get_home_settings_calb (device_t id, home_settings_calb_t* home_settings_calb, const calibration_t* calibration)
```

**Description:** Read home settings. This function fill structure with settings of calibrating position.

---

**Function:**

```C
result_t XIMC_API set_home_settings_calb (device_t id, const home_settings_calb_t* home_settings_calb, const calibration_t* calibration)
```

**Description:** Set home settings. This function send structure with calibrating position settings to controller's memory.

---

**Function:**

```C
result_t XIMC_API command_home (device_t id);
```

**Description:** The positive direction is to the right. A value of zero reverses the direction of the direction of the flag, the set speed. Restriction imposed by the trailer, act the same, except that the limit switch contact does not stop. Limit the maximum speed, acceleration and deceleration function.
1) Moves the motor according to the speed FastHome, uFastHome and flag HOME_DIR_FAST until limit switch, if the flag is set HOME_STOP_ENDS, until the signal from the input synchronization if the flag HOME_STOP_SYNC (as accurately as possible is important to catch the moment of operation limit switch) or until the signal is received from the speed sensor, if the flag HOME_STOP_REV_SN
2) Then moves according to the speed SlowHome, uSlowHome and flag HOME_DIR_SLOW until
signal from the clock input, if the flag HOME_MV_SEC. If the flag HOME_MV_SEC reset
skip this paragraph.
3) Then move the motor according to the speed FastHome, uFastHome and flag HOME_DIR_SLOW a distance
HomeDelta, uHomeDelta.
description of flags and variable see in description for commands GHOM/SHOM

---

**Function:**

```C
result_t XIMC_API command_zero (device_t id)
```

**Description:** Sets the current position and the position in which the traffic moves by the move command and movr zero for all cases, except for movement to the target position. In the latter case, set the zero current position and the target position counted so that the absolute position of the destination is the same. That is, if we were at 400 and moved to 500, then the command Zero makes the current position of 0, and the position of the destination - 100. Does not change the mode of movement that is if the motion is carried, it continues, and if the engine is in the "hold", the type of retention remains.

---

**Function:**

```C
result_t XIMC_API command_move_calb (device_t id, float Position, const calibration_t* calibration)
```

**Description:** Upon receiving the command "move" the engine starts to move with pre-set parameters (speed acceleration, retention), to the point specified to the Position.

---

**Function:**

```C
result_t XIMC_API get_position (device_t id, get_position_t* the_get_position)
```

**Description:** Reads the value position in steps and micro for stepper motor and encoder steps all engines.

---

**Function:**

```C
result_t XIMC_API get_move_settings_calb (device_t id, move_settings_calb_t* move_settings_calb, const calibration_t* calibration)
```

**Description:** Read command setup movement (speed, acceleration, threshold and etc).

---

**Function:**

```C
result_t XIMC_API set_move_settings_calb (device_t id, const move_settings_calb_t* move_settings_calb, const calibration_t* calibration)
```

**Description:** Set command setup movement (speed, acceleration, threshold and etc).

---

**Function:**

```C
result_t XIMC_API close_device (device_t* id)
```

**Description:** Close specified device.

---

### For more information about functions used, see [API Document](https://github.com/Brazilian-Institute-of-Robotics/qm_concept_proof/blob/feature/initialTests/pan_tilt/libximc7-en.pdf)
