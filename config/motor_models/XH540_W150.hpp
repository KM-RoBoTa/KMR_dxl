/**
 ******************************************************************************
 * @file            XH540_W150.hpp
 * @brief           Header file containing control tables of different motors
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 03/2024
 * @authors kamilo.melo@km-robota.com, 03/2024
 ******************************************************************************
 */

#ifndef KMR_DXL_XH540_W150_HPP
#define KMR_DXL_XH540_W150_HPP

#include "../KMR_dxl_structures.hpp"

namespace KMR::dxl
{

struct XH540_W150_P2 : public KMR::dxl::ControlTable {
    XH540_W150_P2() : ControlTable()
    {
        modelNumber.addr = 0;
        modelNumber.length = 2;
        modelNumber.unit = 1;
        modelInfo.addr = 2;
        modelInfo.length = 4;
        modelInfo.unit = 1;
        firmwareVersion.addr = 6;
        firmwareVersion.length = 1;
        firmwareVersion.unit = 1;
        id.addr = 7;
        id.length = 1;
        id.unit = 1;
        baudrate.addr = 8;
        baudrate.length = 1;
        baudrate.unit = 1;
        returnDelayTime.addr = 9;
        returnDelayTime.length = 1;
        returnDelayTime.unit = 0.000002;
        driveMode.addr = 10;
        driveMode.length = 1;
        driveMode.unit = 1;
        operatingMode.addr = 11;
        operatingMode.length = 1;
        operatingMode.unit = 1;
        secondaryId.addr = 12;
        secondaryId.length = 1;
        secondaryId.unit = 1;
        protocolVersion.addr = 13;
        protocolVersion.length = 1;
        protocolVersion.unit = 1;
        homingOffset.addr = 20;
        homingOffset.length = 4;
        homingOffset.unit = 0.001536;
        movingThreshold.addr = 24;
        movingThreshold.length = 4;
        movingThreshold.unit = 0.0240;
        temperatureLimit.addr = 31;
        temperatureLimit.length = 1;
        temperatureLimit.unit = 1;
        maxVoltageLimit.addr = 32;
        maxVoltageLimit.length = 2;
        maxVoltageLimit.unit = 0.1;
        minVoltageLimit.addr = 34;
        minVoltageLimit.length = 2;
        minVoltageLimit.unit = 0.1;
        PWM_limit.addr = 36;
        PWM_limit.length = 2;
        PWM_limit.unit = 0.113;
        currentLimit.addr = 38;
        currentLimit.length = 2;
        currentLimit.unit = 0.00269;
        accelerationLimit.addr = 40;
        accelerationLimit.length = 4;
        accelerationLimit.unit = 0.3745;
        velocityLimit.addr = 44;
        velocityLimit.length = 4;
        velocityLimit.unit = 0.0240;
        maxPositionLimit.addr = 48;
        maxPositionLimit.length = 4;
        maxPositionLimit.unit = 0.001536;
        minPositionLimit.addr = 52;
        minPositionLimit.length = 4;
        minPositionLimit.unit = 0.001536;        
        shutdown.addr = 63;
        shutdown.length = 1;
        shutdown.unit = 1;

        torqueEnable.addr = 64;
        torqueEnable.length = 1;
        torqueEnable.unit = 1;
        LED.addr = 65;
        LED.length = 1;
        LED.unit = 1;
        statusReturnLevel.addr = 68;
        statusReturnLevel.length = 1;
        statusReturnLevel.unit = 1;
        registered.addr = 69;
        registered.length = 1;
        registered.unit = 1;
        hardwareErrorStatus.addr = 70;
        hardwareErrorStatus.length = 1;
        hardwareErrorStatus.unit = 1;
        velocity_I_gain.addr = 76;
        velocity_I_gain.length = 2;
        velocity_I_gain.unit = 1;
        velocity_P_gain.addr = 78;
        velocity_P_gain.length = 2;
        velocity_P_gain.unit = 1;
        position_D_gain.addr = 80;
        position_D_gain.length = 2;
        position_D_gain.unit = 1;
        position_I_gain.addr = 82;
        position_I_gain.length = 2;
        position_I_gain.unit = 1;
        position_P_gain.addr = 84;
        position_P_gain.length = 2;
        position_P_gain.unit = 1;
        feedforward_2_gain.addr = 88;
        feedforward_2_gain.length = 2;
        feedforward_2_gain.unit = 1;
        feedforward_1_gain.addr = 90;
        feedforward_1_gain.length = 2;
        feedforward_1_gain.unit = 1;
        busWatchdog.addr = 98;
        busWatchdog.length = 1;
        busWatchdog.unit = 1;
        goalPWM.addr = 100;
        goalPWM.length = 2;
        goalPWM.unit = 0.113;
        goalCurrent.addr = 102;
        goalCurrent.length = 2;
        goalCurrent.unit = 0.00269;
        goalVelocity.addr = 104;
        goalVelocity.length = 4;
        goalVelocity.unit = 0.0240;
        profileAcceleration.addr = 108;
        profileAcceleration.length = 4;
        profileAcceleration.unit = 214.577;
        profileVelocity.addr = 112;
        profileVelocity.length = 4;
        profileVelocity.unit = 0.0240;
        goalPosition.addr = 116;
        goalPosition.length = 4;
        goalPosition.unit = 0.001536;
        realtimeTick.addr = 120;
        realtimeTick.length = 2;
        realtimeTick.unit = 0.001;
        moving.addr = 122;
        moving.length = 1;
        moving.unit = 1;
        movingStatus.addr = 123;
        movingStatus.length = 1;
        movingStatus.unit = 1;
        presentPWM.addr = 124;
        presentPWM.length = 2;
        presentPWM.unit = 0.113;
        presentCurrent.addr = 126;
        presentCurrent.length = 2;
        presentCurrent.unit = 0.00336;
        presentVelocity.addr = 128;
        presentVelocity.length = 4;
        presentVelocity.unit = 0.0240;
        presentPosition.addr = 132;
        presentPosition.length = 4;
        presentPosition.unit = 0.001536;
        velocityTrajectory.addr = 136;    
        velocityTrajectory.length = 4;
        velocityTrajectory.unit = 1;
        positionTrajectory.addr = 140;
        positionTrajectory.length = 4;
        positionTrajectory.unit = 1;
        presentVoltage.addr = 144;
        presentVoltage.length = 2;
        presentVoltage.unit = 0.1;
        presentTemperature.addr = 146;
        presentTemperature.length = 1;
        presentTemperature.unit = 1; 

        indirectAddress1.addr = 168;
        indirectAddress1.length = 2;
        indirectAddress1.unit = 1;
        indirectData1.addr = 224;
        indirectData1.length = 1;
        indirectData1.unit = 1;
        indirectAddress2.addr = 578;
        indirectAddress2.length = 2;
        indirectAddress2.unit = 1;
        indirectData2.addr = 634;
        indirectData2.length = 1;
        indirectData2.unit = 1;
    }
};

}

#endif