#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <cstring>
#include <netdb.h>
#include <string>
#include <sstream>
#include <vector>
#include <unistd.h>
//#include <stdio.h>
//#include <sys/socket.h>
//#include <exception>
//#include <ctype.h>


using namespace std;

class MotorController {
public:
    MotorController();
    ~MotorController();
    bool initialize(bool debugParam = false); //#debug figure out IP addr stuff for galil
    bool isInitialized(); // return initialized
    bool Stop(); //emergancy stop
    bool Stop(int motorNum); // emergancy stop a single motor
    float readPos(int motorNum); // returns the current motor angle in radians
    float readPosErr(int motorNum); // returns the error in  
    bool setMaxVelocity(int motorNum, float angularVelocity);
    bool setAccel(int motorNum, float angularAccelaration);
    bool setAccelAll(std::vector<int> acclVal);
    bool setDecel(int motorNum, float angularDecelaration);
    float encToAng(int motorNum, long enc);
    long angToEnc(int motorNum, float angle);
    bool positionControl(int motorNum,float angle);
    bool MotorsOFF();
    string command(string Command);
    int degToEnc(int motorNum, float deg);
    bool definePosition(int motorNum, float angle);
    vector<double> readState();


private:
	
    int sID;
    struct addrinfo *host_info_list;
    bool initialized;
    bool connected;
    bool debug;
    string ipAddr;
    double enc2Radian[9];
    long rad2Enc[9];
    static string motorLookup[9];\
    bool setPID(int motorNum, int P, int I, int D);
    bool isValidMotor(int motorNum);
    int commandGalil(char* Command, char* Response, int ResponseSize);
};
#endif
