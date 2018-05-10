#include <stdio.h>
#include <stdlib.h>
#include <string>

std::string username = "ftpuser";
std::string password = "geo011";
std::string urlFtp = " 192.168.0.1 ";

void executeShellScript(const std::string& directory){

std::string initialConnection = "ncftpget -R -v -u" 
+ username + " -p " + password + urlFtp + " ~/Documents/ /" + directory;

system(initialConnection.c_str());
}

int main() {

executeShellScript("teste");

return 0;
}