
//
// *********     Sync Read and Sync Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is designed for using two Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below "#define"d variables yourself.
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000 [1M])
//

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

//#include "KMR_dxl/KMR_dxl.hpp"
#include "KMR_dxl.hpp"

// TEMP
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std;

int main()
{
    vector<int> ids = {1,2};
    const char* portname = "/dev/ttyUSB0";
    int baudrate = 1000000;

    KMR::dxl::BaseRobot robot(ids, portname, baudrate);

    KMR::dxl::ControlMode mode = KMR::dxl::POSITION;
    robot.setControlModes(mode);

    cout << "Field = " << KMR::dxl::ControlTableItem::ACCELERATION_LIMIT << endl;
}
