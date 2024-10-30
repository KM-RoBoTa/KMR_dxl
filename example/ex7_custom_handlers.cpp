/**
 ********************************************************************************************
 * @file    ex7_custom_handlers.cpp
 * @brief   Example for custom handlers
 * @details Concept of the example:
 *          - Speed control. Motors rotate positively in the first half of the program, then 
 *          negatively
 *          - When going forward, we want to set the LED on
 *          => custom Writer handler that writes into GOAL_VELOCITY and LED fields
 * 
 *          - At each step, we get the speed, position and temperature feedbacks
 *          => custom Reader handler that reads from those 3 fields
 ********************************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 ********************************************************************************************
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
#define GOAL_SPEED      2*M_PI/2
#define MAX_CTR         2000
#define CTRL_PERIOD_US  5000

using namespace std;


// --------------------------------------------------------------------------- //
// Id(s) of motor(s)
vector<int> ids = {1,2};  // EDIT HERE FOR YOUR MOTOR(s)
// --------------------------------------------------------------------------- //

int nbrMotors = ids.size();
KMR::dxl::MotorHandler robot(ids, PORTNAME, BAUDRATE);

// Get custom handlers
vector<KMR::dxl::ControlTableItem> wFields = {KMR::dxl::ControlTableItem::GOAL_VELOCITY,
                                              KMR::dxl::ControlTableItem::LED};
KMR::dxl::Writer* writer = robot.getNewWriter(wFields, ids);

vector<KMR::dxl::ControlTableItem> rFields = {KMR::dxl::ControlTableItem::PRESENT_POSITION,
                                              KMR::dxl::ControlTableItem::PRESENT_TEMPERATURE,
                                              KMR::dxl::ControlTableItem::PRESENT_VELOCITY};
KMR::dxl::Reader* reader = robot.getNewReader(rFields, ids);

// Create functions to write/read with the custom handlers
void writeCommands(vector<float> goalSpeeds, vector<int> leds);
bool getFeedbacks(vector<float>& fbckPositions, vector<float>& fbckTemperatures, vector<float>& fbckSpeeds);


int main()
{
    cout << endl << endl << " ---------- CUSTOM HANDLERS ---------" << endl;
    robot.disableMotors();

    // Set the control mode
    KMR::dxl::ControlMode mode = KMR::dxl::ControlMode::SPEED;
    robot.setControlModes(mode);
    sleep(1);
    robot.enableMotors();    
    
    // Required variables
    vector<float> goalSpeeds(nbrMotors, 0);
    vector<int> leds(nbrMotors, 0);
    vector<float> fbckPositions(nbrMotors, 0);
    vector<float> fbckTemperatures(nbrMotors, 0);
    vector<float> fbckSpeeds(nbrMotors, 0);

    float speed = 0;
    int ledOn = 0;
    bool forward = 1;
    int ctr = 0;

    // Main loop
    while (ctr < MAX_CTR) {
        // Get feedback and print it
        timespec start = time_s();
        getFeedbacks(fbckPositions, fbckTemperatures, fbckSpeeds);

        cout << endl;
        for (int i=0; i<nbrMotors; i++) {
            cout << "Motor " << ids[i] << ": speed = " << fbckSpeeds[i] << " rad/s, position = "
            << fbckPositions[i] << " rad, temperature = " << fbckTemperatures[i] << " °C" << endl;
        }

        if (ctr > MAX_CTR/2)
            forward = 0;

        // Update the goal angle for next loop
        if (forward) {
            ledOn = 1;
            speed = GOAL_SPEED;
        }
        else {
            ledOn = 0;
            speed = -GOAL_SPEED;
        }

        // Send new goal positions
        for (int i=0; i<nbrMotors; i++) {
            goalSpeeds[i] = speed;
            leds[i] = ledOn;
        }

        writeCommands(goalSpeeds, leds);

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

bool getFeedbacks(vector<float>& fbckPositions, vector<float>& fbckTemperatures, vector<float>& fbckSpeeds)
{
    bool readSuccess = reader->syncRead();
    if (readSuccess) {
        fbckPositions = reader->getReadingResults(KMR::dxl::ControlTableItem::PRESENT_POSITION);
        fbckTemperatures = reader->getReadingResults(KMR::dxl::ControlTableItem::PRESENT_TEMPERATURE);
        fbckSpeeds = reader->getReadingResults(KMR::dxl::ControlTableItem::PRESENT_VELOCITY);       
        return true;
    }
    else
        return false;
}