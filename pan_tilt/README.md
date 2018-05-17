# Pan-tilt station

### This step by step will show you how to run the "testapp.c" file

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

### Functions used and their descriptions

**Function:**

```C
device_enumeration_t XIMC_API enumerate_devices (int enumerate flags, const char ∗ hints)
```

**Description:** Enumerate all devices that looks like valid

---

**Function:**

```C
int XIMC_API get_device_count (device enumeration t device enumeration)
```

**Description:** Get device count

---

**Function:**

```C
pchar XIMC_API get_device_name (device enumeration t device enumeration, int device index)
```

**Description:** Get device name from the device enumeration
