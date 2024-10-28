/**
 *****************************************************************************
 * @file            ex1_position.cpp
 * @brief           Example for position control
 * @details         This example is designed for 2 motors to showcase the effect of setting
 *                  the min and max positions.
 *                  Those motors can be any model, in protocol 2.
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

#include "../include/KMR_dxl.hpp"

#define BAUDRATE        1000000
#define PORTNAME        "/dev/ttyUSB0"
#define GOAL_CURRENT1   0.2
#define GOAL_CURRENT2   -0.2
#define MAX_CTR         2000
#define CTRL_PERIOD_US  5000

using namespace std;


// --------------------------------------------------------------------------- //
// Id(s) of motor(s)
vector<int> ids = {1,2};  // EDIT HERE FOR YOUR MOTOR(s)
// --------------------------------------------------------------------------- //

int nbrMotors = ids.size();
KMR::dxl::BaseRobot robot(ids, PORTNAME, BAUDRATE);

int main()
{
    cout << endl << endl << " ---------- CURRENT CONTROL ---------" << endl;
    robot.disableMotors();
    KMR::dxl::ControlMode mode = KMR::dxl::ControlMode::CURRENT;
    robot.setControlModes(mode);
    sleep(1);
    robot.enableMotors();

    // Set max currents
    vector<float> maxCurrents = {0.3, 0.2};  
    robot.setMaxCurrent(maxCurrents, ids);
    usleep(50*1000);
    robot.enableMotors();        

    float current = GOAL_CURRENT1; // A
    int ctr = 0;

    vector<float> goalCurrents(nbrMotors, current);
    vector<float> fbckCurrents(nbrMotors, 0);

    while (ctr < MAX_CTR) {
        // Get feedback
        timespec start = time_s();
        robot.getCurrents(fbckCurrents);

        cout << "Currents: "; 
        for (int i=0; i<nbrMotors; i++) {
            cout << fbckCurrents[i] << " A";
            if (i != (nbrMotors-1))
                cout << ", ";
        }
        cout << endl;

        // Send new goal currents
        if (ctr > MAX_CTR/2)
            current = GOAL_CURRENT2;
        for (int i=0; i<nbrMotors; i++)
            goalCurrents[i] = current;
        robot.setCurrents(goalCurrents);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    current = 0;
    for (int i=0; i<nbrMotors; i++)
        goalCurrents[i] = current;
    robot.setCurrents(goalCurrents);    

    sleep(1);

    robot.disableMotors();    

    // Reset the limits
    for (int i=0; i<nbrMotors; i++)
        maxCurrents[i] = 6.5;
    robot.setMaxCurrent(maxCurrents, ids);
    usleep(50*1000);

    cout << "Example finished, exiting" << endl;
}
