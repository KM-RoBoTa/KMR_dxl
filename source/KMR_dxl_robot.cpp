/**
 ******************************************************************************
 * @file            KMR_dxl_robot.cpp
 * @brief           Defines the BaseRobot class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include <cstdint>
#include <iostream>


#include <unistd.h>  // Provides sleep function for linux
#include "KMR_dxl_robot.hpp"

#define PROTOCOL_VERSION            2.0
#define ENABLE                      1
#define DISABLE                     0


using namespace std;

namespace KMR::dxl
{


/**
 * @brief       Constructor for BaseRobot
 * @param[in]   all_ids List of IDs of all the motors in the robot
 * @param[in]   port_name Name of the port handling the communication with motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   hal Previously initialized Hal object
 */
BaseRobot::BaseRobot(vector<int> all_ids, const char *port_name, int baudrate, Hal hal)
{
    m_hal = hal;
    m_all_IDs = all_ids;

    // Connect U2D2
    init_comm(port_name, baudrate, PROTOCOL_VERSION);

    // 2 integrated handlers: motor enabling and mode setter
    m_motor_enabler = new Writer(vector<Fields>{TRQ_ENABLE}, m_all_IDs, portHandler_, packetHandler_, m_hal, 0);
    m_controlMode_setter = new Writer(vector<Fields>{OP_MODE}, m_all_IDs, portHandler_, packetHandler_, m_hal, 0);

    // Ping each motor to validate the communication is working
    check_comm();

}


/**
 * @brief Destructor
 */
BaseRobot::~BaseRobot()
{
    delete m_motor_enabler;
    delete m_controlMode_setter;
}


/**
 * @brief       Initialize the serial communication
 * @param[in]   port_name Name of the port handling communication with motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   protocol_version Protocol version, for the communication (U2D2)
 */
void BaseRobot::init_comm(const char *port_name, int baudrate, float protocol_version)
{
    portHandler_ = dynamixel::PortHandler::getPortHandler(port_name);
    if (!portHandler_->openPort()) {
        cout<< "Failed to open the motors port!" <<endl;
        exit(1);
    }
    else
        cout<< "Succeeded to open the motors port!" <<endl;

    if (!portHandler_->setBaudRate(baudrate)) {
        cout<< "Failed to set baudrate!" <<endl;
        return ;
    }
    else
        cout<< "Succeeded to change the baudrate!" <<endl;

    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
}

/**
 * @brief       Ping each motor to validate the communication is working
 */
void BaseRobot::check_comm()
{
    bool result = false;
    uint16_t model_number = 0;
    uint8_t dxl_error = 0;
    int motor_idx;
    int id = 0;

    cout << "Pinging motors...." << endl;

    for (int i=0; i<m_all_IDs.size(); i++) {
        id = m_all_IDs[i];
        result = packetHandler_->ping(portHandler_, id, &model_number, &dxl_error);
        if (result != COMM_SUCCESS) {
            cout << "Failed to ping, check config file and motor ID: " << id << endl;
            cout << "Check also the power source and the cabling ;) " << endl;
            cout << packetHandler_->getTxRxResult(result) << endl;
            exit(1);
        }
        else {
            cout << "id: " << id << ", model number : " << model_number << endl;
            motor_idx = m_hal.getMotorsListIndexFromID(id);
            m_hal.m_motors_list[motor_idx].scanned_model = model_number; 
        }
    }
}

/*
******************************************************************************
 *                         Enable/disable motors
 ****************************************************************************/

/**
 * @brief       Enable all the motors
 */
void BaseRobot::enableMotors()
{
    m_motor_enabler->addDataToWrite(vector<int>{ENABLE}, TRQ_ENABLE, m_all_IDs);
    m_motor_enabler->syncWrite(m_all_IDs);
}

/**
 * @brief       Enable motors specified by IDs
 * @param[in]   ids List of motor ids to be enabled
 */
void BaseRobot::enableMotors(vector<int> ids)
{
    m_motor_enabler->addDataToWrite(vector<int>{ENABLE}, TRQ_ENABLE, ids);
    m_motor_enabler->syncWrite(ids);    
}

/**
 * @brief       Disable all the motors
 */
void BaseRobot::disableMotors()
{
    m_motor_enabler->addDataToWrite(vector<int>{DISABLE}, TRQ_ENABLE, m_all_IDs);
    m_motor_enabler->syncWrite(m_all_IDs);
}

/**
 * @brief       Disable motors specified by IDs
 * @param[in]   ids List of motor ids to be disabled
 */
void BaseRobot::disableMotors(vector<int> ids)
{
    m_motor_enabler->addDataToWrite(vector<int>{DISABLE}, TRQ_ENABLE, ids);
    m_motor_enabler->syncWrite(ids);    
}


/*
******************************************************************************
 *                Reset necessary motors in multiturn mode
 ****************************************************************************/
/**
 * @brief       Set single motor to multiturn mode. Used for multiturn reset
 * @param[in]   id Motor id to get set to multiturn mode
 * @param[in]   motor Query motor 
 */
void BaseRobot::setMultiturnControl_singleMotor(int id, Motor motor)
{
    m_controlMode_setter->addDataToWrite(vector<int>{motor.control_modes.multiturn_control}, OP_MODE, vector<int>{id});
    m_controlMode_setter->syncWrite(vector<int>{id});
}

/**
 * @brief       Set single motor to position control mode. Used for multiturn reset
 * @param[in]   id Motor id to get set to control position mode
 * @param[in]   motor Query motor 
 */
void BaseRobot::setPositionControl_singleMotor(int id, Motor motor)
{
    int pos_control = motor.control_modes.position_control;
    m_controlMode_setter->addDataToWrite(vector<int>{motor.control_modes.position_control}, OP_MODE, vector<int>{id});
    m_controlMode_setter->syncWrite(vector<int>{id});
}


/**
 * @brief       Reset multiturn motors flagged as needing a reset.
 * @note        Make sure the motors had enough time to execute the goal position command before 
 *              calling this function. Failure to do so results in undefined behavior.
 */
void BaseRobot::resetMultiturnMotors()
{
    Motor motor;
    int id;

    for(int i=0; i<m_all_IDs.size(); i++) {
        id = m_all_IDs[i];
        motor = m_hal.getMotorFromID(id);
        if (motor.toReset) {
            disableMotors(vector<int>{id});
            setPositionControl_singleMotor(id, motor);
            setMultiturnControl_singleMotor(id, motor);    
            enableMotors(vector<int>{id});

            m_hal.updateResetStatus(id, 0);
        }
    }
}


/*
******************************************************************************
 *                               EEPROM init writing
 ****************************************************************************/

/**
 * @brief       Set the control modes of motors
 * @param[in]   controlModes Control modes to be set to motors
 */
void BaseRobot::setControlModes(vector<int> controlModes)
{
    m_controlMode_setter->addDataToWrite(controlModes, OP_MODE, m_all_IDs);
    m_controlMode_setter->syncWrite(m_all_IDs);
}


/**
 * @brief       Set the minimum voltage of motors
 * @param[in]   minVoltages Min. allowed voltages in motors
 */                                 
void BaseRobot::setMinVoltage(vector<float> minVoltages)
{
    m_EEPROM_writer = new Writer(vector<Fields> {MIN_VOLT_LIMIT}, 
                                            m_all_IDs, portHandler_, packetHandler_, m_hal, 0);

    m_EEPROM_writer->addDataToWrite(minVoltages, KMR::dxl::MIN_VOLT_LIMIT, m_all_IDs);
    m_EEPROM_writer->syncWrite(m_all_IDs);

    delete m_EEPROM_writer;
}

/**
 * @brief       Set the maximum voltage of motors
 * @param[in]   minVoltages Max. allowed voltages in motors
 */                                 
void BaseRobot::setMaxVoltage(vector<float> maxVoltages)
{
    m_EEPROM_writer = new Writer(vector<Fields> {MAX_VOLT_LIMIT}, 
                                            m_all_IDs, portHandler_, packetHandler_, m_hal, 0);

    m_EEPROM_writer->addDataToWrite(maxVoltages, MAX_VOLT_LIMIT, m_all_IDs);
    m_EEPROM_writer->syncWrite(m_all_IDs);

    delete m_EEPROM_writer;
}

/**
 * @brief       Set the minimum position of motors
 * @param[in]   minPositions Min. positions for motors (lower saturation) 
 */                                 
void BaseRobot::setMinPosition(vector<float> minPositions)
{
    m_EEPROM_writer = new Writer(vector<Fields> {MIN_POS_LIMIT}, 
                                            m_all_IDs, portHandler_, packetHandler_, m_hal, 0);

    m_EEPROM_writer->addDataToWrite(minPositions, MIN_POS_LIMIT, m_all_IDs);
    m_EEPROM_writer->syncWrite(m_all_IDs);

    delete m_EEPROM_writer;
}

/**
 * @brief       Set the maximum position of motors
 * @param[in]   maxPositions Max. positions for motors (upper saturation) 
 */                                 
void BaseRobot::setMaxPosition(vector<float> maxPositions)
{
    m_EEPROM_writer = new Writer(vector<Fields> {MAX_POS_LIMIT}, 
                                            m_all_IDs, portHandler_, packetHandler_, m_hal, 0);

    m_EEPROM_writer->addDataToWrite(maxPositions, MAX_POS_LIMIT, m_all_IDs);
    m_EEPROM_writer->syncWrite(m_all_IDs);

    delete m_EEPROM_writer;
}

/**
 * @brief   Set the return delay to all motors
 */
void BaseRobot::setAllDelay(int val)
{
    m_EEPROM_writer = new Writer(vector<Fields> {RETURN_DELAY}, 
                                            m_all_IDs, portHandler_, packetHandler_, m_hal, 0);

    m_EEPROM_writer->addDataToWrite(vector<int>{val}, RETURN_DELAY, m_all_IDs);
    m_EEPROM_writer->syncWrite(m_all_IDs);

    delete m_EEPROM_writer;
}


}
