# FTP Cliente

### This step by step will show you how to configure and run the "mainFtpGet.cpp" and "mainFtpPut.cpp" file

### Configuration 

First, execute this command to install library dependencies:

```sh
sudo apt-get install ncftp
```

Inside the code, write correct information about connection FTP Server

```sh
std::string username = "username";
std::string password = "password";
std::string urlFtp = " ipadress ";
```
Within the main method, type the folder to be sent or downloaded from the FTP server

```sh
executeShellScript("directory");
```

The directory must be inside the "~/Documents/" directory

### Make and Run

First, in the source codes folder

```sh
g++ mainFtpGet.cpp -o mainFtpGet
or
g++ mainFtpPut.cpp -o mainFtpPut
```

Finally, execute this command

```sh
./mainFtpGet
or
./mainFtpPut
```