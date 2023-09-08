/**
 * KM-Robota library
 ******************************************************************************
 * @file            kmr_dxl_robot.cpp
 * @brief           Defines the BaseRobot class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT  !!!!!
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023  !!!
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ******************************************************************************
 */

#include <cstdint>
#include <iostream>

#include "KMR_dxl_robot.hpp"

#define PROTOCOL_VERSION            2.0
#define ENABLE                      1
#define DISABLE                     0


using namespace std;

namespace KMR::dxl
{


/**
 * @brief       Constructor for LibRobot
 * @param[in]   num_motors Number of motors in the robot
 * @param[in]   all_ids List of IDs of all the motors in the robot
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   port_name Name of the port handling communication with motors
 */
BaseRobot::BaseRobot(vector<int> all_ids, const char *port_name, int baudrate, Hal hal)
{
    m_hal = hal;
    m_all_IDs = all_ids;

    // Connect U2D2
    init_comm(port_name, baudrate, PROTOCOL_VERSION);

    // Writing handler, taking care of enabling/disabling motors
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
    cout << "The Robot object is being deleted" << endl;
}


/**
 * @brief       Initialize the serial communication for a LibRobot instance
 * @param[in]   port_name Name of the port handling communication with motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   protocol_version Protocol version, for the communication (U2D2)
 * @retval      void
 */
void BaseRobot::init_comm(const char *port_name, int baudrate, float protocol_version)
{
    portHandler_ = dynamixel::PortHandler::getPortHandler(port_name);
    if (!portHandler_->openPort()) {
        cout<< "Failed to open the motors port!" <<endl;
        exit(1);
    }
    else
        cout<< "Succeed to open the motors port" <<endl;

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
 * @retval      void
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
 * @retval      void
 */
void BaseRobot::enableMotors()
{
    m_motor_enabler->addDataToWrite(vector<int>{ENABLE}, TRQ_ENABLE, m_all_IDs);
    m_motor_enabler->syncWrite(m_all_IDs);
}

/**
 * @brief       Enable motors specified by IDs
 * @param[in]   ids List of motor ids to be enabled
 * @retval      void
 */
void BaseRobot::enableMotors(vector<int> ids)
{
    m_motor_enabler->addDataToWrite(vector<int>{ENABLE}, TRQ_ENABLE, ids);
    m_motor_enabler->syncWrite(ids);    
}

/**
 * @brief       Disable all the motors
 * @retval      void
 */
void BaseRobot::disableMotors()
{
    m_motor_enabler->addDataToWrite(vector<int>{DISABLE}, TRQ_ENABLE, m_all_IDs);
    m_motor_enabler->syncWrite(m_all_IDs);
}

/**
 * @brief       Disable motors specified by IDs
 * @param[in]   ids List of motor ids to be disabled
 * @retval      void
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

void BaseRobot::setMultiturnControl_singleMotor(int id, Motor motor)
{
    disableMotors(vector<int>{id});
    m_controlMode_setter->addDataToWrite(vector<int>{motor.control_modes.multiturn_control}, OP_MODE, vector<int>{id});
    m_controlMode_setter->syncWrite(vector<int>{id});
    enableMotors(vector<int>{id});
}

void BaseRobot::setPositionControl_singleMotor(int id, Motor motor)
{
    disableMotors(vector<int>{id});
    int pos_control = motor.control_modes.position_control;
    m_controlMode_setter->addDataToWrite(vector<int>{motor.control_modes.position_control}, OP_MODE, vector<int>{id});
    m_controlMode_setter->syncWrite(vector<int>{id});
    enableMotors(vector<int>{id});
}


void BaseRobot::resetMultiturnMotors()
{
    Motor motor;
    int id;

    for(int i=0; i<m_all_IDs.size(); i++) {
        id = m_all_IDs[i];
        motor = m_hal.getMotorFromID(id);
        if (motor.toReset) {
            setPositionControl_singleMotor(id, motor);
            setMultiturnControl_singleMotor(id, motor);

            m_hal.updateResetStatus(id, 0);

            cout << "Motor is over limit and reset" << endl;
        }
    }
}


}
