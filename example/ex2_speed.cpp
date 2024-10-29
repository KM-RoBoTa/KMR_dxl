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
#define GOAL_SPEED1      2*M_PI/2
#define GOAL_SPEED2      -2*M_PI/3
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
    cout << endl << endl << " ---------- SPEED CONTROL ---------" << endl;
    robot.disableMotors();

    // Set control mode
    KMR::dxl::ControlMode mode = KMR::dxl::ControlMode::SPEED;
    robot.setControlModes(mode);
    sleep(1);

    // Set max speeds
    // Max limit of second motor < goal speed => second motor will not move
    vector<float> maxSpeeds = {2*M_PI, +2*M_PI/3};  
    robot.setMaxSpeed(maxSpeeds);
    usleep(50*1000);
    robot.enableMotors();        

    vector<float> goalSpeeds(nbrMotors, 0);
    vector<float> fbckSpeeds(nbrMotors, 0);

    float speed = GOAL_SPEED1;
    bool forward = 1;
    int ctr = 0;

    while (ctr < MAX_CTR) {
        // Get feedback
        timespec start = time_s();
        robot.getSpeeds(fbckSpeeds);

        cout << "Speeds: "; 
        for (int i=0; i<nbrMotors; i++) {
            cout << fbckSpeeds[i] << " rad/s";
            if (i != (nbrMotors-1))
                cout << ", ";
        }
        cout << endl;

        // Send new goal speeds
        if (ctr > MAX_CTR/2)
            speed = GOAL_SPEED2;
        for (int i=0; i<nbrMotors; i++)
            goalSpeeds[i] = speed;
        robot.setSpeeds(goalSpeeds);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    speed = 0;
    for (int i=0; i<nbrMotors; i++)
        goalSpeeds[i] = speed;
    robot.setSpeeds(goalSpeeds);    

    sleep(1);

    robot.disableMotors();

    // Reset the limits
    for (int i=0; i<nbrMotors; i++)
        maxSpeeds[i] = +2*M_PI;
    robot.setMaxSpeed(maxSpeeds);
    usleep(50*1000);

    cout << "Example finished, exiting" << endl;
}
