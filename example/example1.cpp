
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
#include <cmath>

//#include "KMR_dxl/KMR_dxl.hpp"
#include "../include/KMR_dxl.hpp"

// TEMP
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std;

// Global variables for easiness
vector<int> ids = {1,2};
//vector<int> ids = {1};
const char* portname = "/dev/ttyUSB0";
int baudrate = 1000000;
int nbrMotors = ids.size();
KMR::dxl::BaseRobot robot(ids, portname, baudrate);

// Functions
void positionControlDemo();
void speedControlDemo();
void currentControlDemo();
void pwmControlDemo();

int main()
{
    robot.disableMotors();
    robot.setReturnDelayTime(0);
    sleep(1);

    cout << "Motor turned on." << endl << "Rebooting in 3 sec" << endl;
    sleep(1);
    cout << "Rebooting in 2 sec" << endl;
    sleep(1);
    cout << "Rebooting in 1 sec" << endl;
    sleep(1);
    cout << "Rebooting..." << endl;

    robot.reboot(1);
    sleep(3);

    //positionControlDemo();
    //sleep(5);
    //speedControlDemo();
    //sleep(5);
    //currentControlDemo();
    //sleep(5);
    //pwmControlDemo();
}


void positionControlDemo()
{
    cout << endl << endl << " ---------- POSITION CONTROL ---------" << endl;
    robot.disableMotors();
    KMR::dxl::ControlMode mode = KMR::dxl::POSITION;
    robot.setControlModes(mode);
    sleep(1);
    vector<float> minPositions = {-M_PI, -2*M_PI/3};
    vector<float> maxPositions = {+M_PI, +M_PI/2};
    robot.setMinPosition(minPositions, ids);
    sleep(1);
    robot.setMaxPosition(maxPositions, ids);
    sleep(1);
    robot.enableMotors();    

    vector<float> goalPositions(nbrMotors, 0);
    vector<float> fbckPositions(nbrMotors, 0);

    float angle = 0;
    bool forward = 1;
    const float increment = 0.02;
    int ctr = 0, maxCtr = 1000;

    robot.setPositions(goalPositions);
    sleep(1);

    while (ctr < maxCtr) {
        // Get feedback
        timespec start = time_s();
        robot.getPositions(fbckPositions);

        cout << "Positions: "; 
        for (int i=0; i<nbrMotors; i++)
            cout << fbckPositions[0] << " rad, ";
        cout << endl;

        // Send new goal positions
        for (int i=0; i<nbrMotors; i++)
            goalPositions[i] = angle;

        robot.setPositions(goalPositions);


        // Update the goal angle for next loop
        if (forward) {
            angle += increment;

            if (angle > M_PI) {
                angle = M_PI;
                forward = false;
            }
        }
        else {
            angle -= increment;

            if (angle < -M_PI) {
                angle = -M_PI;
                forward = true;
            }
        }

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        //cout << "Elapsed = " << elapsed << " us " << endl;

        double toSleep_us = 5*1000-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    robot.disableMotors();
}


void speedControlDemo()
{
    cout << endl << endl << " ---------- SPEED CONTROL ---------" << endl;
    robot.disableMotors();
    KMR::dxl::ControlMode mode = KMR::dxl::SPEED;
    robot.setControlModes(mode);
    sleep(1);
    robot.enableMotors();

    vector<float> goalSpeeds(nbrMotors, 0);
    vector<float> fbckSpeeds(nbrMotors, 0);

    float speed = -2*M_PI/5;
    bool forward = 1;
    float runtime = 5;
    int ctr = 0, maxCtr = runtime/(5/1000.0);

    while (ctr < maxCtr) {
        // Get feedback
        timespec start = time_s();
        robot.getSpeeds(fbckSpeeds);

        cout << "Speeds: "; 
        for (int i=0; i<nbrMotors; i++)
            cout << fbckSpeeds[i] << " rad/s, ";
        cout << endl;

        // Send new goal speeds
        for (int i=0; i<nbrMotors; i++)
            goalSpeeds[i] = speed;

        robot.setSpeeds(goalSpeeds);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        //cout << "Elapsed = " << elapsed << " us " << endl;

        double toSleep_us = 5*1000-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    ctr = 0;
    speed = -2*M_PI/5;
    while (ctr < maxCtr) {
        // Get feedback
        timespec start = time_s();
        robot.getSpeeds(fbckSpeeds);

        cout << "Speeds: "; 
        for (int i=0; i<nbrMotors; i++)
            cout << fbckSpeeds[i] << " rad/s, ";
        cout << endl;

        // Send new goal speeds
        for (int i=0; i<nbrMotors; i++)
            goalSpeeds[i] = speed;

        robot.setSpeeds(goalSpeeds);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        //cout << "Elapsed = " << elapsed << " us " << endl;

        double toSleep_us = 5*1000-elapsed;
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
}

void currentControlDemo()
{
    cout << endl << endl << " ---------- CURRENT CONTROL ---------" << endl;
    robot.disableMotors();
    KMR::dxl::ControlMode mode = KMR::dxl::CURRENT;
    robot.setControlModes(mode);
    sleep(1);
    robot.enableMotors();

    float current = -0.2; // A
    float runtime = 5; // s
    int ctr = 0, maxCtr = runtime/(5/1000.0);

    vector<float> goalCurrents(nbrMotors, current);
    vector<float> fbckCurrents(nbrMotors, 0);

    while (ctr < maxCtr) {
        // Get feedback
        timespec start = time_s();
        robot.getCurrents(fbckCurrents);

        cout << "Currents: "; 
        for (int i=0; i<nbrMotors; i++)
            cout << fbckCurrents[i] << " A,  ";
        cout << endl;

        // Send new goal speeds
        robot.setCurrents(goalCurrents);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        //cout << "Elapsed = " << elapsed << " us " << endl;

        double toSleep_us = 5*1000-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    ctr = 0;
    current = 0.2;
    // Send new goal speeds
    for (int i=0; i<nbrMotors; i++)
        goalCurrents[i] = current;

    while (ctr < maxCtr) {
        // Get feedback
        timespec start = time_s();
        robot.getCurrents(fbckCurrents);

        cout << "Currents: "; 
        for (int i=0; i<nbrMotors; i++)
            cout << fbckCurrents[i] << " A,  ";
        cout << endl;

        // Send new goal speeds
        robot.setCurrents(goalCurrents);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        //cout << "Elapsed = " << elapsed << " us " << endl;

        double toSleep_us = 5*1000-elapsed;
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
}


void pwmControlDemo()
{
    cout << endl << endl << " ---------- PWM CONTROL ---------" << endl;
    robot.disableMotors();
    KMR::dxl::ControlMode mode = KMR::dxl::PWM;
    robot.setControlModes(mode);
    sleep(1);
    robot.enableMotors();

    float pwm = 33; // %
    float runtime = 5; // s
    int ctr = 0, maxCtr = runtime/(5/1000.0);

    vector<float> goalPWNs(nbrMotors, pwm);
    vector<float> fbckPWMs(nbrMotors, 0);

    while (ctr < maxCtr) {
        // Get feedback
        timespec start = time_s();
        robot.getPWMs(fbckPWMs);

        cout << "PWMs: "; 
        for (int i=0; i<nbrMotors; i++)
            cout << fbckPWMs[i] << " %,  ";
        cout << endl;

        // Send new goal speeds
        robot.setPWMs(goalPWNs);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        //cout << "Elapsed = " << elapsed << " us " << endl;

        double toSleep_us = 5*1000-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    ctr = 0;
    pwm = -75;
    // Send new goal speeds
    for (int i=0; i<nbrMotors; i++)
        goalPWNs[i] = pwm;

    while (ctr < maxCtr) {
        // Get feedback
        timespec start = time_s();
        robot.getPWMs(fbckPWMs);

        cout << "PWMs: "; 
        for (int i=0; i<nbrMotors; i++)
            cout << fbckPWMs[i] << " %,  ";
        cout << endl;

        // Send new goal speeds
        robot.setPWMs(goalPWNs);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        //cout << "Elapsed = " << elapsed << " us " << endl;

        double toSleep_us = 5*1000-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    pwm = 0;
    for (int i=0; i<nbrMotors; i++)
        goalPWNs[i] = pwm;
    robot.setCurrents(goalPWNs);    

    sleep(1);

    robot.disableMotors();    
}