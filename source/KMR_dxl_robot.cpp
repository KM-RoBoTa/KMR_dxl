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

using namespace std;

namespace KMR::dxl
{

#define PROTOCOL_VERSION            2.0
#define ENABLE                      1
#define DISABLE                     0

// Multiturn

/**
 * @brief       Constructor for BaseRobot
 * @param[in]   all_ids List of IDs of all the motors in the robot
 * @param[in]   port_name Name of the port handling the communication with motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   hal Previously initialized Hal object
 */
BaseRobot::BaseRobot(vector<int> ids, const char *port_name, int baudrate)
{
    m_hal = new Hal();
    m_ids = ids;
    m_nbrMotors = ids.size();
    m_models = vector<int>(m_nbrMotors);

    // Connect U2D2
    init_comm(port_name, baudrate, PROTOCOL_VERSION);

    // Ping each motor to validate the communication is working
    check_comm();

    // Initialize Hal
    m_hal->init(m_ids, m_nbrMotors, m_models);

    // 2 integrated handlers: motor enabling and mode setter
    m_controlModeWriter = getNewWriter(vector<ControlTableItem>{OPERATING_MODE}, m_ids);
    m_motorEnableWriter = getNewWriter(vector<ControlTableItem>{TORQUE_ENABLE}, m_ids);

    // Integrated base command handlers
    m_positionWriter = getNewWriter(vector<ControlTableItem>{GOAL_POSITION}, m_ids);
    m_speedWriter = getNewWriter(vector<ControlTableItem>{GOAL_VELOCITY}, m_ids);
    m_currentWriter = getNewWriter(vector<ControlTableItem>{GOAL_CURRENT}, m_ids);
    m_PWMWriter = getNewWriter(vector<ControlTableItem>{GOAL_PWM}, m_ids);
    m_positionReader = getNewReader(vector<ControlTableItem>{PRESENT_POSITION}, m_ids);
    m_speedReader = getNewReader(vector<ControlTableItem>{PRESENT_VELOCITY}, m_ids);
    m_currentReader = getNewReader(vector<ControlTableItem>{PRESENT_CURRENT}, m_ids);
    m_PWMReader = getNewReader(vector<ControlTableItem>{PRESENT_PWM}, m_ids);

}


/**
 * @brief Destructor
 */
BaseRobot::~BaseRobot()
{
    // Delete handlers created on heap
    deleteWriter(m_controlModeWriter);
    deleteWriter(m_motorEnableWriter);

    deleteWriter(m_positionWriter); 
    deleteWriter(m_speedWriter);
    deleteWriter(m_currentWriter);
    deleteWriter(m_PWMWriter);
    deleteReader(m_positionReader);
    deleteReader(m_speedReader);
    deleteReader(m_currentReader);
    deleteReader(m_PWMReader);

    m_controlModeWriter = nullptr;
    m_motorEnableWriter = nullptr;

    m_positionWriter = nullptr;
    m_speedWriter = nullptr;
    m_currentWriter = nullptr;
    m_PWMWriter = nullptr;
    m_positionReader = nullptr;
    m_speedReader = nullptr;
    m_currentReader = nullptr;
    m_PWMReader = nullptr;

    // Delete the Hal
    delete m_hal;
    m_hal = nullptr;

    // Close port
    portHandler_->closePort();
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
        cout << "Failed to open the motors port!" <<endl;
        exit(1);
    }
    else
        cout<< "Succeeded to open the motors port!" <<endl;

    if (!portHandler_->setBaudRate(baudrate)) {
        cout<< "Failed to set baudrate!" <<endl;
        return ;
    }
    else
        cout << "Succeeded to change the baudrate!" <<endl;

    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
}

/**
 * @brief   Ping each motor to validate the communication is working
 * @note    Also populates the models vector, required to initialize Hal
 */
void BaseRobot::check_comm()
{
    cout << "Pinging motors...." << endl;

    for (int i=0; i<m_nbrMotors; i++) {
        int id = m_ids[i];
        uint16_t model_number = 0;
        uint8_t dxl_error = 0;

        bool result = packetHandler_->ping(portHandler_, id, &model_number, &dxl_error);
        if (result != COMM_SUCCESS) {
            cout << "Failed to ping, check config file and motor ID: " << id << endl;
            cout << "Check also the power source and the cabling ;) " << endl;
            cout << packetHandler_->getTxRxResult(result) << endl;
            exit(1);
        }
        else {
            cout << "id: " << id << ", model number : " << model_number << endl;
            m_models[i] = (int)model_number;
        }
    }
}

/*
******************************************************************************
 *                         Easy handlers creation
 ****************************************************************************/

// Create a new Writer handler (!!! on heap)
Writer* BaseRobot::getNewWriter(vector<ControlTableItem> fields, vector<int> ids)
{
    // Get the list of models corresponding to the ids
    vector<int> models(ids.size());
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        if (idx < 0) {
            cout << "Error! Unknown ID during Writer creation. Exiting" << endl;
            exit(1);
        }
        models[i] = m_models[i];
    }

    Writer* writer = new Writer(fields, ids, models, portHandler_, packetHandler_, m_hal, 0);
    return writer;
}

// Create a new Reader handler (!!! on heap)
Reader* BaseRobot::getNewReader(vector<ControlTableItem> fields, vector<int> ids)
{
    // Get the list of models corresponding to the ids
    vector<int> models(ids.size());
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        if (idx < 0) {
            cout << "Error! Unknown ID during Reader creation. Exiting" << endl;
            exit(1);
        }
        models[i] = m_models[i];
    }

    Reader* reader = new Reader(fields, ids, models, portHandler_, packetHandler_, m_hal, 0);
    return reader;
}

void BaseRobot::deleteWriter(Writer* writer)
{
    delete writer;
    writer = nullptr; // Security in case of double freeing
}


void BaseRobot::deleteReader(Reader* reader)
{
    delete reader;
    reader = nullptr; // Security in case of double freeing
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
    m_motorEnableWriter->addDataToWrite(vector<int>{ENABLE});
    m_motorEnableWriter->syncWrite();
}

/**
 * @brief       Disable all the motors
 */
void BaseRobot::disableMotors()
{
    m_motorEnableWriter->addDataToWrite(vector<int>{DISABLE});
    m_motorEnableWriter->syncWrite();
}


/*
*****************************************************************************
*                      Multiturn mode functions
****************************************************************************/

/**
 * @brief       Reset multiturn motors flagged as needing a reset.
 * @note        Make sure the motors had enough time to execute the goal position command before 
 *              calling this function. Failure to do so results in undefined behavior.
 */
void BaseRobot::resetMultiturnMotors()
{
    bool needSleep = 0;
    for(int i=0; i<m_nbrMotors; i++) {
        int id = m_ids[i];
        Motor motor = m_hal->getMotorFromID(id);

        if (motor.toReset) {
            needSleep = 1;
            reboot(id);
            m_hal->updateResetStatus(id, 0);
        }
    }

    if (needSleep) {
        usleep(100*1000);  // Wait for the reboot to finish
        enableMotors();
        usleep(5*1000); // Allow the enable
    }
}

void BaseRobot::reboot(int id)
{
    packetHandler_->reboot(portHandler_, id);
}

void BaseRobot::reboot()
{
    for (int i=0; i<m_nbrMotors; i++)
        packetHandler_->reboot(portHandler_, m_ids[i]);
}

//******************************************************************************
// *                               EEPROM init writing
// ****************************************************************************/

/**
 * @brief       Set the control modes of motors
 * @param[in]   controlModes Control modes to be set to motors
 */
void BaseRobot::setControlModes(vector<ControlMode> controlModes)
{
    if (controlModes.size() != m_nbrMotors) {
        cout << "Error! Not all motors have their control modes assigned. Exiting" << endl; 
        cout << endl;
    }

    vector<int> controlModes_int(m_nbrMotors);
    for (int i=0; i<m_nbrMotors; i++) {
        switch (controlModes[i])
        {
        case CURRENT:
            controlModes_int[i] = CTRL_CURRENT;
            break;
        case SPEED:
            controlModes_int[i] = CTRL_SPEED;
            break;
        case POSITION:
            controlModes_int[i] = CTRL_POSITION;
            break;
        case MULTITURN:
            controlModes_int[i] = CTRL_MULTITURN;
            m_hal->setMultiturnMode(m_ids[i]);
            break;
        case HYBRID:
            controlModes_int[i] = CTRL_HYBRID;
            break;
        case PWM:
            controlModes_int[i] = CTRL_PWM;
            break;
        
        default:
            cout << "Error! Trying to assign an unknown control mode. Exiting" << endl;
            exit(1);
            break;
        }
    }

    m_controlModeWriter->addDataToWrite(controlModes_int);
    m_controlModeWriter->syncWrite();
}

/**
 * @brief       Set the control modes of motors
 * @param[in]   controlModes Control modes to be set to motors
 */
void BaseRobot::setControlModes(ControlMode controlMode)
{
    vector<ControlMode> controlModes(m_nbrMotors, controlMode);
    setControlModes(controlModes);
}



/**
 * @brief       Set the minimum voltage of motors
 * @param[in]   minVoltages Min. allowed voltages in motors
 */                                 
void BaseRobot::setMinVoltage(vector<float> minVoltages)
{
    Writer writer(vector<ControlTableItem>{MIN_VOLTAGE_LIMIT}, m_ids, m_models, portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(minVoltages);
    writer.syncWrite();
}

/**
 * @brief       Set the minimum voltage of motors
 * @param[in]   minVoltages Min. allowed voltages in motors
 */                                 
void BaseRobot::setMinVoltage(float minVoltage)
{
    vector<float> minVoltages(m_nbrMotors, minVoltage);
    setMinVoltage(minVoltages);
}


/**
 * @brief       Set the minimum voltage of motors
 * @param[in]   minVoltages Min. allowed voltages in motors
 */                                 
void BaseRobot::setMaxVoltage(vector<float> maxVoltages)
{
    Writer writer(vector<ControlTableItem>{MAX_VOLTAGE_LIMIT}, m_ids, m_models, portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(maxVoltages);
    writer.syncWrite();
}

/**
 * @brief       Set the minimum voltage of motors
 * @param[in]   minVoltages Min. allowed voltages in motors
 */                                 
void BaseRobot::setMaxVoltage(float maxVoltage)
{
    vector<float> maxVoltages(m_nbrMotors, maxVoltage);
    setMaxVoltage(maxVoltages);
}

/**
 * @brief       Set the minimum position of motors
 * @param[in]   minPositions Min. positions for motors (lower saturation) 
 */                                 
void BaseRobot::setMinPosition(vector<float> minPositions, vector<int> ids)
{
    // Get the list of models corresponding to the ids
    vector<int> models(ids.size());
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        if (idx < 0) {
            cout << "Error! Unknown ID during Writer creation. Exiting" << endl;
            exit(1);
        }
        models[i] = m_models[i];
    }
    
    Writer writer(vector<ControlTableItem>{MIN_POSITION_LIMIT}, ids, models, portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(minPositions);
    writer.syncWrite();
}

/**
 * @brief       Set the maximum position of motors
 * @param[in]   maxPositions Max. positions for motors (upper saturation) 
 */                                 
void BaseRobot::setMaxPosition(vector<float> maxPositions, vector<int> ids)
{
    // Get the list of models corresponding to the ids
    vector<int> models(ids.size());
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        if (idx < 0) {
            cout << "Error! Unknown ID during Writer creation. Exiting" << endl;
            exit(1);
        }
        models[i] = m_models[i];
    }

    Writer writer(vector<ControlTableItem>{MAX_POSITION_LIMIT}, ids, models, portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(maxPositions);
    writer.syncWrite();
}

/**
 * @brief   Set the return delay to all motors
 */
void BaseRobot::setReturnDelayTime(float val)
{
    Writer writer(vector<ControlTableItem>{RETURN_DELAY}, m_ids, m_models, portHandler_, packetHandler_, m_hal, 0);

    vector<float> vals(m_nbrMotors, val);
    writer.addDataToWrite(vals);
    writer.syncWrite();
}


void BaseRobot::setMaxSpeed(vector<float> maxSpeeds, vector<int> ids)
{
    // Get the list of models corresponding to the ids
    vector<int> models(ids.size());
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        if (idx < 0) {
            cout << "Error! Unknown ID during Writer creation. Exiting" << endl;
            exit(1);
        }
        models[i] = m_models[i];
    }

    Writer writer(vector<ControlTableItem>{VELOCITY_LIMIT}, ids, models, portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(maxSpeeds);
    writer.syncWrite();
}


void BaseRobot::setMaxCurrent(vector<float> maxCurrents, vector<int> ids)
{
    // Get the list of models corresponding to the ids
    vector<int> models(ids.size());
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        if (idx < 0) {
            cout << "Error! Unknown ID during Writer creation. Exiting" << endl;
            exit(1);
        }
        models[i] = m_models[i];
    }

    Writer writer(vector<ControlTableItem>{CURRENT_LIMIT}, ids, models, portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(maxCurrents);
    writer.syncWrite();    
}

void BaseRobot::setMaxPWM(vector<float> maxPWMs, vector<int> ids)
{
    // Get the list of models corresponding to the ids
    vector<int> models(ids.size());
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        if (idx < 0) {
            cout << "Error! Unknown ID during Writer creation. Exiting" << endl;
            exit(1);
        }
        models[i] = m_models[i];
    }

    Writer writer(vector<ControlTableItem>{PWM_LIMIT}, ids, models, portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(maxPWMs);
    writer.syncWrite();   
}



/******************************************************************************
/ *                            Base controls
/ ****************************************************************************/

void BaseRobot::setPositions(vector<float> positions)
{
    m_positionWriter->addDataToWrite(positions);
    m_positionWriter->syncWrite();
}

bool BaseRobot::getPositions(vector<float>& positions)
{
    bool readSuccess = m_positionReader->syncRead();
    if (readSuccess) {
        positions = m_positionReader->getReadingResults();
        return true;
    }
    else
        return false;
}

void BaseRobot::setSpeeds(vector<float> speeds)
{
    m_speedWriter->addDataToWrite(speeds);
    m_speedWriter->syncWrite();
}

bool BaseRobot::getSpeeds(vector<float>& speeds)
{
    bool readSuccess = m_speedReader->syncRead();
    if (readSuccess) {
        speeds = m_speedReader->getReadingResults();
        return true;
    }
    else
        return false;
}

void BaseRobot::setCurrents(vector<float> currents)
{
    m_currentWriter->addDataToWrite(currents);
    m_currentWriter->syncWrite();
}

bool BaseRobot::getCurrents(vector<float>& currents)
{
    bool readSuccess = m_currentReader->syncRead();
    if (readSuccess) {
        currents = m_currentReader->getReadingResults();
        return true;
    }
    else 
        return false;
}

void BaseRobot::setPWMs(vector<float> pwms)
{
    m_PWMWriter->addDataToWrite(pwms);
    m_PWMWriter->syncWrite();
}

bool BaseRobot::getPWMs(vector<float>& pwms)
{
    bool readSuccess = m_PWMReader->syncRead();
    if (readSuccess) {
        pwms = m_PWMReader->getReadingResults();
        return true;
    }
    else 
        return false;
}

void BaseRobot::setHybrid(vector<float> positions, vector<float> currents)
{
    m_positionWriter->addDataToWrite(positions);
    m_currentWriter->addDataToWrite(currents);

    m_positionWriter->syncWrite();
    m_currentWriter->syncWrite();
}

bool BaseRobot::getHybrid(vector<float>& positions, vector<float>& currents)
{
    bool readPositionSuccess = m_positionReader->syncRead();
    bool readCurrentSuccess = m_currentReader->syncRead();

    if (readPositionSuccess && readCurrentSuccess) {
        positions = m_positionReader->getReadingResults();
        currents = m_currentReader->getReadingResults();
        return true;
    }
    else 
        return false;
}

}
