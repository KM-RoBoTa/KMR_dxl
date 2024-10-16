/**
 ******************************************************************************
 * @file            KMR_dxl_structures.hpp
 * @brief           Header file containing useful structures definitions
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com,10/2024
 * @authors kamilo.melo@km-robota.com, 10/2024
 ******************************************************************************
 */


#ifndef KMR_DXL_STRUCTURES_HPP
#define KMR_DXL_STRUCTURES_HPP

namespace KMR_dxl
{
#define UNDEF -1

struct Field {
    float unit;
    int length;
    int addr;
};

struct ControlTable {
    Field modelNumber;
    Field modelInfo;
    Field firmwareVersion;
    Field protocolVersion;
    Field id;
    Field secondaryId;
    Field baudrate;
    Field driveMode;
    Field controlMode;
    Field operatingMode;
    Field CW_angleLimit;
    Field CCW_angleLimit;
    Field temperatureLimit;
    Field minVoltageLimit;
    Field maxVoltageLimit;
    Field PWM_limit;
    Field currentLimit;
    Field velocityLimit;
    Field maxPositionLimit;
    Field minPositionLimit;
    Field accelerationLimit;
    Field maxTorque;
    Field homingOffset;
    Field movingThreshold;
    Field multiturnOffset;
    Field resolutionDivider;
    Field externalPortMode1;
    Field externalPortMode2;
    Field externalPortMode3;
    Field externalPortMode4;
    Field statusReturnLevel;
    Field returnDelayTime;
    Field alarmLed;
    Field shutdown;

    Field torqueEnable;
    Field LED;
    Field LED_red;
    Field LED_green;
    Field LED_blue;
    Field registeredInstruction;
    Field hardwareErrorStatus;
    Field velocity_P_gain;
    Field velocity_I_gain;
    Field position_P_gain;
    Field position_I_gain;
    Field position_D_gain;
    Field feedforward_1_gain;
    Field feedforward_2_gain;
    Field P_gain;
    Field I_gain;
    Field D_gain;
    Field CW_complianceMargin;
    Field CCW_complianceMargin;
    Field CW_complianceSlope;
    Field CCW_complianceSlope;
    Field goalPWM;
    Field goalTorque;
    Field goalCurrent;
    Field goalPosition;
    Field goalVelocity;
    Field goalAcceleration;
    Field movingSpeed;
    Field presentPWM;
    Field presentLoad;
    Field presentSpeed;
    Field presentCurrent;
    Field presentPosition;
    Field presentVelocity;
    Field presentVoltage;
    Field presentTemperature;
    Field torqueLimit;
    Field registered;
    Field moving;
    Field lock;
    Field punch;
    Field current;
    Field sensedCurrent;
    Field realtimeTick;
    Field torqueCtrlModeEnabled;
    Field busWatchdog;
    Field profileAcceleration;
    Field profileVelocity;
    Field movingStatus;
    Field velocityTrajectory;
    Field positionTrajectory;
    Field externalPortData1;
    Field externalPortData2;
    Field externalPortData3;
    Field externalPortData4;
    Field indirectAddress1;
    Field indirectData1;
    Field indirectAddress2;
    Field indirectData2;

    ControlTable()
    {
        modelNumber.length = UNDEF;
        modelInfo.length = UNDEF;
        firmwareVersion.length = UNDEF;
        protocolVersion.length = UNDEF;
        id.length = UNDEF;
        secondaryId.length = UNDEF;
        baudrate.length = UNDEF;
        driveMode.length = UNDEF;
        controlMode.length = UNDEF;
        operatingMode.length = UNDEF;
        CW_angleLimit.length = UNDEF;
        CCW_angleLimit.length = UNDEF;
        temperatureLimit.length = UNDEF;
        minVoltageLimit.length = UNDEF;
        maxVoltageLimit.length = UNDEF;
        PWM_limit.length = UNDEF;
        currentLimit.length = UNDEF;
        velocityLimit.length = UNDEF;
        maxPositionLimit.length = UNDEF;
        minPositionLimit.length = UNDEF;
        accelerationLimit.length = UNDEF;
        maxTorque.length = UNDEF;
        homingOffset.length = UNDEF;
        movingThreshold.length = UNDEF;
        multiturnOffset.length = UNDEF;
        resolutionDivider.length = UNDEF;
        externalPortMode1.length = UNDEF;
        externalPortMode2.length = UNDEF;
        externalPortMode3.length = UNDEF;
        externalPortMode4.length = UNDEF;
        statusReturnLevel.length = UNDEF;
        returnDelayTime.length = UNDEF;
        alarmLed.length = UNDEF;
        shutdown.length = UNDEF;

        torqueEnable.length = UNDEF;
        LED.length = UNDEF;
        LED_red.length = UNDEF;
        LED_green.length = UNDEF;
        LED_blue.length = UNDEF;
        registeredInstruction.length = UNDEF;
        hardwareErrorStatus.length = UNDEF;
        velocity_P_gain.length = UNDEF;
        velocity_I_gain.length = UNDEF;
        position_P_gain.length = UNDEF;
        position_I_gain.length = UNDEF;
        position_D_gain.length = UNDEF;
        feedforward_1_gain.length = UNDEF;
        feedforward_2_gain.length = UNDEF;
        P_gain.length = UNDEF;
        I_gain.length = UNDEF;
        D_gain.length = UNDEF;
        CW_complianceMargin.length = UNDEF;
        CCW_complianceMargin.length = UNDEF;
        CW_complianceSlope.length = UNDEF;
        CCW_complianceSlope.length = UNDEF;
        goalPWM.length = UNDEF;
        goalTorque.length = UNDEF;
        goalCurrent.length = UNDEF;
        goalPosition.length = UNDEF;
        goalVelocity.length = UNDEF;
        goalAcceleration.length = UNDEF;
        movingSpeed.length = UNDEF;
        presentPWM.length = UNDEF;
        presentLoad.length = UNDEF;
        presentSpeed.length = UNDEF;
        presentCurrent.length = UNDEF;
        presentPosition.length = UNDEF;
        presentVelocity.length = UNDEF;
        presentVoltage.length = UNDEF;
        presentTemperature.length = UNDEF;
        torqueLimit.length = UNDEF;
        registered.length = UNDEF;
        moving.length = UNDEF;
        lock.length = UNDEF;
        punch.length = UNDEF;
        current.length = UNDEF;
        sensedCurrent.length = UNDEF;
        realtimeTick.length = UNDEF;
        torqueCtrlModeEnabled.length = UNDEF;
        busWatchdog.length = UNDEF;
        profileAcceleration.length = UNDEF;
        profileVelocity.length = UNDEF;
        movingStatus.length = UNDEF;
        velocityTrajectory.length = UNDEF;
        positionTrajectory.length = UNDEF;
        externalPortData1.length = UNDEF;
        externalPortData2.length = UNDEF;
        externalPortData3.length = UNDEF;
        externalPortData4.length = UNDEF;
    }
};

}

#endif