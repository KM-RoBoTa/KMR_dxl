/**
 ********************************************************************************************
 * @file    ex6_hybrid.cpp
 * @brief   Example for PWM control
 * @details This example is designed for 2 motors to showcase the effect of setting a max.
 *          PWM lower than the goal PWM. \n
 *          Those motors can be any model, in protocol 2 (IDs 1 and 2 by default). \n \n
 * 
 *          At first, the motors are commanded with a positive PWM. Since both motors' limits
 *          are set higher than this command, both motors execute it, resulting in a positive
 *          rotation. \n 
 *          During the second half, the motors are commanded with a negative PWM with a 
 *          higher absolute value than in the previous phase. However, since the second motor's
 *          limit is lower than the command, the second motor ignores the new command and thus 
 *          continues applying the command from the first part.
 ********************************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 ********************************************************************************************
 */


#ifdef 0

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
#define INCREMENT       0.02
#define MAX_CTR         2000
#define CTRL_PERIOD_US  5000

using namespace std;



// --------------------------------------------------------------------------- //
// Id(s) of motor(s)
vector<int> ids = {1,2};  // EDIT HERE FOR YOUR MOTOR(s)
// --------------------------------------------------------------------------- //

int nbrMotors = ids.size();
KMR::dxl::MotorHandler robot(ids, PORTNAME, BAUDRATE);

int main()
{
    cout << endl << endl << " ---------- HYBRID CONTROL ---------" << endl;
    robot.disableMotors();
    KMR::dxl::ControlMode mode = KMR::dxl::ControlMode::HYBRID;
    robot.setControlModes(mode);
    sleep(1);

    // Set max currents
    /*vector<float> maxPWMs = {100, 40};  // %
    robot.setMaxPWM(maxPWMs);
    usleep(50*1000);
    robot.enableMotors();        */

    vector<float> goalPositions(nbrMotors, 0);
    vector<float> goalCurrents(nbrMotors, 1);
    vector<float> fbckPositions(nbrMotors, 0);
    vector<float> fbckCurrents(nbrMotors, 0);

    float angle = 0;
    bool forward = 1;
    int ctr = 0;

    while (ctr < MAX_CTR) {
        // Get feedback
        timespec start = time_s();
        robot.getHybrid(fbckPositions, fbckCurrents);

        cout << endl;
        for (int i=0; i<nbrMotors; i++) {
            cout << "Motor " << ids[i] << ": position = " << fbckPositions[i] <<
            ", currents = " << fbckCurrents[i] << " A" << endl;
        }

        // Send new goal positions
        for (int i=0; i<nbrMotors; i++)
            goalPositions[i] = angle;
        robot.setHybrid(goalPositions, goalCurrents);

        // Update the goal angle for next loop
        if (forward) {
            angle += INCREMENT;

            if (angle > M_PI) {
                angle = M_PI;
                forward = false;
            }
        }
        else {
            angle -= INCREMENT;

            if (angle < -M_PI) {
                angle = -M_PI;
                forward = true;
            }
        }

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    robot.disableMotors();    
    cout << "Example finished, exiting" << endl;
}

#endif