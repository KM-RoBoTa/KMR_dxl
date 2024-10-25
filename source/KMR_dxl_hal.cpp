/**
 ******************************************************************************
 * @file            KMR_dxl_hal.cpp
 * @brief           Defines the Hal class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT  
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include "KMR_dxl_hal.hpp"
#include <iostream>
#include <cstdint>

using namespace std;

namespace KMR::dxl

#define POS_OFFSET_DEFAULT 3.14159265358979323846264338327950288 // M_PI
{
    
/**
 * @brief       Constructor for Hal
 */
Hal::Hal()
{
    MX_28 = new MX_28_P2();
    MX_64 = new MX_64_P2();
    MX_106 = new MX_106_P2();
    XH540_W150 = new XH540_W150_P2();
    XH540_W270 = new XH540_W270_P2();
    XM430_W350 = new XM430_W350_P2();
    XM540_W150 = new XM540_W150_P2();
    XM540_W270 = new XM540_W270_P2();
    XW430_T200 = new XW430_T200_P2();
    XW540_T140 = new XW540_T140_P2();
    XW540_T260 = new XW540_T260_P2();
}

/**
 * @brief       Destructor for Hal
 */
Hal::~Hal()
{
    delete MX_28;
    delete MX_64;
    delete MX_106;
    delete XH540_W150;
    delete XH540_W270;
    delete XM430_W350;
    delete XM540_W150;
    delete XM540_W270;
    delete XW430_T200;
    delete XW540_T140;
    delete XW540_T260;    
}


/**
 * @brief       Initialize the Hal - called by BaseRobot in its cstr
 * @param[in]   ids List of IDs of all the motors in the robot
 * @param[in]   nbrMotors Number of motors in the robot
 * @param[in]   models List of model numbers of each motor, gotten during the motors ping
 */
void Hal::init(vector<int> ids, int nbrMotors, vector<int> models)
{
    m_nbrMotors = nbrMotors;
    m_ids = ids;
    m_models =  models;

    for (int i=0; i<m_nbrMotors; i++) {
        Motor motor(m_ids[i], m_models[i]);
        m_motorsList.push_back(motor);
    }
}

/**
 * @brief       Get the control table corresponding to the input motor model number
 * @param[in]   modelNumber Query motor model number
 * @return      Control table corresponding to the query motor
 */
ControlTable Hal::getControlTable(int modelNumber)
{
    ControlTable motor;

    switch (modelNumber) {
    case MODEL_NBR_MX_28: 
        motor = *MX_28; break;
    case MODEL_NBR_MX_64:
        motor = *MX_64; break;
    case MODEL_NBR_MX_106:
        motor = *MX_106; break;
    case MODEL_NBR_XH540_W150:
        motor = *XH540_W150; break;
    case MODEL_NBR_XH540_W270:
        motor = *XH540_W270; break;
    case MODEL_NBR_XM430_W350:
        motor = *XM430_W350; break;
    case MODEL_NBR_XM540_W150:
        motor = *XM540_W150; break;
    case MODEL_NBR_XM540_W270:
        motor = *XM540_W270; break;
    case MODEL_NBR_XW430_T200:
        motor = *XW430_T200; break;
    case MODEL_NBR_XW540_T140:
        motor = *XW540_T140; break;
    case MODEL_NBR_XW540_T260:
        motor = *XW540_T260; break;

    default:
        cout << "Error: this model is unknown! Exiting" << endl;
        exit(1);
    }

    return motor;
}

/**
 * @brief       Get a specific control field corresponding to the input motor model number
 * @param[in]   modelNumber Query motor model number
 * @param[in]   item Query field in the control table (ex: GOAL_POSITION)
 * @return      Control field corresponding to the query
 */
Field Hal::getControlFieldFromModel(int modelNumber, ControlTableItem item)
{
    ControlTable motor = getControlTable(modelNumber);
    Field field = getControlField(motor, item);

    return field;
}


/**
 * @brief       Extract a specific control field from the input control table
 * @param[in]   motor Control table of the query motor, previously gotten with getControlTable()
 * @param[in]   item Query field in the control table (ex: GOAL_POSITION)
 * @return      Control field corresponding to the query
 */
Field Hal::getControlField(ControlTable motor, ControlTableItem item)
{
    Field field;

    switch (item) {
    case ControlTableItem::MODEL_NBR:               field = motor.modelNumber;              break;
    case ControlTableItem::MODEL_INFO:              field = motor.modelInfo;                break;
    case ControlTableItem::FIRMWARE:                field = motor.firmwareVersion;          break;
    case ControlTableItem::ID:                      field = motor.id;                       break;
    case ControlTableItem::BAUDRATE:                field = motor.baudrate;                 break;
    case ControlTableItem::RETURN_DELAY:            field = motor.returnDelayTime;          break;
    case ControlTableItem::DRIVE_MODE:              field = motor.driveMode;                break;
    case ControlTableItem::OPERATING_MODE:          field = motor.operatingMode;            break;
    case ControlTableItem::SHADOW_ID:               field = motor.secondaryId;              break;
    case ControlTableItem::PROTOCOL:                field = motor.protocolVersion;          break;
    case ControlTableItem::HOMING_OFFSET:           field = motor.homingOffset;             break;
    case ControlTableItem::MOVING_THRESHOLD:        field = motor.movingThreshold;          break;
    case ControlTableItem::TEMPERATURE_LIMIT:       field = motor.temperatureLimit;         break;
    case ControlTableItem::MAX_VOLTAGE_LIMIT:       field = motor.maxVoltageLimit;          break;
    case ControlTableItem::MIN_VOLTAGE_LIMIT:       field = motor.minVoltageLimit;          break;
    case ControlTableItem::PWM_LIMIT:               field = motor.PWM_limit;                break;
    case ControlTableItem::CURRENT_LIMIT:           field = motor.currentLimit;             break;
    case ControlTableItem::ACCELERATION_LIMIT:      field = motor.accelerationLimit;        break;
    case ControlTableItem::VELOCITY_LIMIT:          field = motor.velocityLimit;            break;
    case ControlTableItem::MAX_POSITION_LIMIT:      field = motor.maxPositionLimit;         break;
    case ControlTableItem::MIN_POSITION_LIMIT:      field = motor.minPositionLimit;         break;
    case ControlTableItem::SHUTDOWN:                field = motor.shutdown;                 break;


    case ControlTableItem::TORQUE_ENABLE:           field = motor.torqueEnable;             break; 
    case ControlTableItem::LED:                     field = motor.LED;                      break;
    case ControlTableItem::STATUS_RETURN:           field = motor.statusReturnLevel;        break;
    case ControlTableItem::REGISTERED:              field = motor.registered;               break;
    case ControlTableItem::HARDWARE_ERROR:          field = motor.hardwareErrorStatus;      break;
    case ControlTableItem::VELOCITY_I_GAIN:         field = motor.velocity_I_gain;          break;
    case ControlTableItem::VELOCITY_P_GAIN:         field = motor.velocity_P_gain;          break;
    case ControlTableItem::POSITION_D_GAIN:         field = motor.position_D_gain;          break;
    case ControlTableItem::POSITION_P_GAIN:         field = motor.position_P_gain;          break;
    case ControlTableItem::POSITION_I_GAIN:         field = motor.position_I_gain;          break;
    case ControlTableItem::FF_2ND_GAIN:             field = motor.feedforward_2_gain;       break;
    case ControlTableItem::FF_1ST_GAIN:             field = motor.feedforward_1_gain;       break;
    case ControlTableItem::BUS_WATCHDOG:            field = motor.busWatchdog;              break;
    case ControlTableItem::GOAL_PWM:                field = motor.goalPWM;                  break;
    case ControlTableItem::GOAL_CURRENT:            field = motor.goalCurrent;              break;
    case ControlTableItem::GOAL_VELOCITY:           field = motor.goalVelocity;             break;
    case ControlTableItem::PROFILE_ACCELERATION:    field = motor.profileAcceleration;      break;
    case ControlTableItem::PROFILE_VELOCITY:        field = motor.profileVelocity;          break;
    case ControlTableItem::GOAL_POSITION:           field = motor.goalPosition;             break;
    case ControlTableItem::REALTIME_TICK:           field = motor.realtimeTick;             break;
    case ControlTableItem::MOVING:                  field = motor.moving;                   break;
    case ControlTableItem::MOVING_STATUS:           field = motor.movingStatus;             break;
    case ControlTableItem::PRESENT_PWM:             field = motor.presentPWM;               break;
    case ControlTableItem::PRESENT_CURRENT:         field = motor.presentCurrent;           break;
    case ControlTableItem::PRESENT_VELOCITY:        field = motor.presentVelocity;          break;
    case ControlTableItem::PRESENT_POSITION:        field = motor.presentPosition;          break;
    case ControlTableItem::VELOCITY_TRAJECTORY:     field = motor.velocityTrajectory;       break;
    case ControlTableItem::POSITION_TRAJECTORY:     field = motor.positionTrajectory;       break;
    case ControlTableItem::PRESENT_VOLTAGE:         field = motor.presentVoltage;           break;
    case ControlTableItem::PRESENT_TEMPERATURE:     field = motor.presentTemperature;       break;

    case ControlTableItem::INDIR_ADD_1:             field = motor.indirectAddress1;         break;
    case ControlTableItem::INDIR_DATA_1:            field = motor.indirectData1;            break;
    case ControlTableItem::INDIR_ADD_2:             field = motor.indirectAddress2;         break;
    case ControlTableItem::INDIR_DATA_2:            field = motor.indirectData2;            break;

    default:
        cout << "Error: this field is unknown! Exiting" << endl;
        exit(1);
    }    

    if (field.length == UNDEF) {
        cout << "Error: this field does not exist for this motor or protocol!" << endl;
        exit(1);
    }

    return field;
}

/**
 * @brief       Get our custom position offset, so that the 0 angle is in the center
 * @param[in]   modelNumber Query motor model number
 * @return      Position offset [rad]
 */
float Hal::getPositionOffset(int modelNumber)
{
    float offset = 0;

    // Insert any special case here (eg AX-12A in protocol 1)
    offset = POS_OFFSET_DEFAULT;
            
    return offset;
}




/*****************************************************************************
 *                     Query functions from outside
 ****************************************************************************/

/**
 * @brief       Get a motor's info structure from motor ID
 * @param[in]   id ID of the query motor
 * @retval      The Motor structure of the query motor
 */
Motor Hal::getMotorFromID(int id)
{
    int motor_idx = getIndex(m_ids, id);
    Motor motor = m_motorsList[motor_idx];

    return motor;
}


/*****************************************************************************
 *                             Misc. functions
 ****************************************************************************/

/**
 * @brief       Update the offsets to access empty indirect addresses for a given motor
 * @param[in]   id ID of the query motor
 * @param[in]   data_length Byte length of the data that was just assigned an indirect address
 * @param[in]   field_name Type of field that was just assigned an indirect address (address or data)
 */
void Hal::addMotorOffsetFromID(int id, uint8_t data_length, std::string field_name)
{
    int motor_idx = getIndex(m_ids, id);

    if (field_name == "indir_address_offset")
        m_motorsList[motor_idx].indir_address_offset += data_length;
    else if (field_name == "indir_data_offset")
        m_motorsList[motor_idx].indir_data_offset += data_length;
    else{
        cout << "Cannot change that motor field!" << endl;
        exit(1);
    }

}

/**
 * @brief       Update a motor's "to reset" status in multiturn mode
 * @param[in]   id ID of the query motor
 * @param[in]   status Boolean: 1 if need to reset, 0 if not
 */
void Hal::updateResetStatus(int id, int status)
{
    int idx = getIndex(m_ids, id);
    m_motorsList[idx].toReset = status;
}


/**
 * @brief       For each motor, save all operating modes control values. \n 
 *              Needed for resetting in multiturn mode
 */
/*void Hal::saveControlValuesToMotors()
{
    int idx, id, model;

    for (int i=0; i<m_tot_nbr_motors; i++){
        id = m_all_IDs[i];
        int idx = getIndex(m_ids, id);
        model = m_motors_list[idx].model;

        m_motors_list[idx].control_modes.current_based_position_control = 
                            m_controlModesPerModel[model].current_based_position_control;
        m_motors_list[idx].control_modes.position_control = m_controlModesPerModel[model].position_control;
        m_motors_list[idx].control_modes.current_control = m_controlModesPerModel[model].current_control;
        m_motors_list[idx].control_modes.multiturn_control = m_controlModesPerModel[model].multiturn_control;
        m_motors_list[idx].control_modes.PWM_control = m_controlModesPerModel[model].PWM_control;
        m_motors_list[idx].control_modes.velocity_control= m_controlModesPerModel[model].velocity_control;
    }
}*/
       

}
