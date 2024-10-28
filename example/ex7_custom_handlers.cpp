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
#define INCREMENT       0.02
#define MAX_CTR         2000
#define CTRL_PERIOD_US  5000

using namespace std;


// --------------------------------------------------------------------------- //
// Id(s) of motor(s)
vector<int> ids = {1,2};  // EDIT HERE FOR YOUR MOTOR(s)
// --------------------------------------------------------------------------- //

int nbrMotors = ids.size();
KMR::dxl::BaseRobot robot(ids, PORTNAME, BAUDRATE);

// Get custom handlers
vector<KMR::dxl::ControlTableItem> wFields = {KMR::dxl::ControlTableItem::GOAL_VELOCITY,
                                              KMR::dxl::ControlTableItem::LED};
KMR::dxl::Writer* writer = robot.getNewWriter(wFields, ids);

vector<KMR::dxl::ControlTableItem> rFields = {KMR::dxl::ControlTableItem::PRESENT_POSITION,
                                              KMR::dxl::ControlTableItem::PRESENT_TEMPERATURE};
KMR::dxl::Reader* reader = robot.getNewReader(rFields, ids);

void writeCommands(vector<float> goalSpeeds, vector<int> leds);
bool getFeedbacks(vector<float>& fbckPositions, vector<float>& fbckTemperatures);


int main()
{
    cout << endl << endl << " ---------- CUSTOM HANDLERS ---------" << endl;
    robot.disableMotors();

    // Set the control mode
    KMR::dxl::ControlMode mode = KMR::dxl::ControlMode::SPEED;
    robot.setControlModes(mode);
    sleep(1);

    // Set min/max positions
    vector<float> minPositions = {-M_PI, -2*M_PI/3};
    vector<float> maxPositions = {+M_PI, +M_PI/2};
    robot.setMinPosition(minPositions, ids);
    usleep(50*1000);
    robot.setMaxPosition(maxPositions, ids);
    usleep(50*1000);
    robot.enableMotors();    
    
    // Required variables
    vector<float> goalSpeeds(nbrMotors, 0);
    vector<int> leds(nbrMotors, 0);
    vector<float> fbckPositions(nbrMotors, 0);
    vector<float> fbckTemperatures(nbrMotors, 0);

    float angle = 0;
    int ledOn = 0;
    bool forward = 1;
    int ctr = 0;

    writeCommands(goalSpeeds, leds);
    getFeedbacks(fbckPositions, fbckTemperatures);

    exit(1);

    robot.setPositions(goalSpeeds);
    sleep(1);

    // Main loop
    while (ctr < MAX_CTR) {
        // Get feedback
        timespec start = time_s();
        robot.getPositions(fbckPositions);

        cout << "Positions: "; 
        for (int i=0; i<nbrMotors; i++) {
            cout << fbckPositions[i] << " rad";
            if (i != (nbrMotors-1))
                cout << ", ";
        }
        cout << endl;

        // Send new goal positions
        for (int i=0; i<nbrMotors; i++)
            goalSpeeds[i] = angle;
        robot.setPositions(goalSpeeds);

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

    // Reset the limits
    for (int i=0; i<nbrMotors; i++) {
        minPositions[i] = -M_PI;
        maxPositions[i] = +M_PI;
    }
    robot.setMinPosition(minPositions, ids);
    usleep(50*1000);
    robot.setMaxPosition(maxPositions, ids);
    usleep(50*1000);

    // Cleanup of the heap
    robot.deleteWriter(writer);
    robot.deleteReader(reader);

    cout << "Example finished, exiting" << endl;


}


void writeCommands(vector<float> goalSpeeds, vector<int> leds)
{
    writer->addDataToWrite(goalSpeeds, KMR::dxl::ControlTableItem::GOAL_VELOCITY);
    writer->addDataToWrite(leds, KMR::dxl::ControlTableItem::LED);
    writer->syncWrite();
}

bool getFeedbacks(vector<float>& fbckPositions, vector<float>& fbckTemperatures)
{
    bool readSuccess = reader->syncRead();
    if (readSuccess) {
        fbckPositions = reader->getReadingResults(KMR::dxl::ControlTableItem::PRESENT_POSITION);
        fbckTemperatures = reader->getReadingResults(KMR::dxl::ControlTableItem::PRESENT_TEMPERATURE);
        return true;
    }
    else
        return false;
}