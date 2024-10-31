/**
 *****************************************************************************
 * @file            KMR_dxl_writer.cpp
 * @brief           Define the Writer class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#include <algorithm>
#include <cstdint>
#include <cmath>

#include "KMR_dxl_writer.hpp"

using namespace std;

namespace KMR::dxl
{

/**
 * @brief       Constructor for a Writer handler
 * @param[in]   list_fields List of fields to be handled by the writer
 * @param[in]   ids Motors to be handled by the writer
 * @param[in]   models Models of the motors to be handled by the writer
 * @param[in]   portHandler Object handling port communication
 * @param[in]   packetHandler Object handling packets
 * @param[in]   hal Hal object for interface with hardware
 * @param[in]   forceIndirect 1 to force the Handler as indirect when only 1 field
 */
Writer::Writer(vector<ControlTableItem> list_fields, vector<int> ids, vector<int> models,
                dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                Hal* hal, bool forceIndirect)
: Handler(list_fields, ids, models, packetHandler, portHandler, hal, forceIndirect)
{
    m_groupSyncWriter = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_,
                                                    m_data_address, m_data_byte_size);

    // Create the table to save parametrized data (to be read or sent)
    m_dataParam = new uint8_t *[m_nbrMotors];
    for (int i=0; i<m_ids.size(); i++)
        m_dataParam[i] = new uint8_t[m_data_byte_size];
}

/**
 * @brief Destructor
 */
Writer::~Writer()
{
    delete m_groupSyncWriter;

    for (int i=0; i<m_ids.size(); i++)
        delete[] m_dataParam[i];
    delete[] m_dataParam;

    m_groupSyncWriter = nullptr;
    m_dataParam = nullptr;
}


/*
 *****************************************************************************
 *                             Data writing
 ****************************************************************************/

/**
 * @brief       Save a parametrized data into the general table. Used by addDataToWrite()
 * @param[in]   data Parametrized data to be sent to motor
 * @param[in]   motor_idx Index of the motor
 * @param[in]   field_idx Index of the field (type of data)
 * @param[in]   field_length Byte size of the data
 */
void Writer::populateDataParam(int32_t data, int motor_idx, int field_idx, int field_length)
{
    if (field_length == 4) {
        m_dataParam[motor_idx][field_idx+0] = DXL_LOBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][field_idx+1] = DXL_HIBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][field_idx+2] = DXL_LOBYTE(DXL_HIWORD(data));
        m_dataParam[motor_idx][field_idx+3] = DXL_HIBYTE(DXL_HIWORD(data));
    }
    else if (field_length == 2) {
        m_dataParam[motor_idx][field_idx+0] = DXL_LOBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][field_idx+1] = DXL_HIBYTE(DXL_LOWORD(data));
    }
    else if (field_length == 1) {
        m_dataParam[motor_idx][field_idx+0] = DXL_LOBYTE(DXL_LOWORD(data));
    }
    else
        cout << "Wrong number of parameters to populate the parametrized matrix!" << endl;
}


/**
 * @brief   Send the previously prepared data with addDataToWrite() to motors
 */
void Writer::syncWrite()
{
    clearParam();

    for(int i=0; i<m_nbrMotors; i++) {
        bool dxl_addparam_result = addParam((uint8_t) m_ids[i], m_dataParam[i]);

        if (dxl_addparam_result != true) {
            cout << "Adding parameters failed for ID = " << m_ids[i] << endl;
        }
    }

    // Send the packet
    int dxl_comm_result = m_groupSyncWriter->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
}


/**
 * @brief   Clear the parameters list
 */
void Writer::clearParam()
{
    m_groupSyncWriter->clearParam();
}


/**
 * @brief       Add data to be written to a motor
 * @param[in]   id ID of the motor
 * @param[in]   data Parametrized data to be sent to the motor
 * @retval      bool: true if data-to-send added to the list successfully
 */
bool Writer::addParam(uint8_t id, uint8_t* data)
{
    bool dxl_addparam_result = m_groupSyncWriter->addParam(id, data);
    return dxl_addparam_result;
}

/*
 *****************************************************************************
 *                            Multiturn functions
 ****************************************************************************/

/** 
 * @brief       Set the internal reset request flag in multiturn mode if necessary
 * @param[in]   id ID of the motor
 * @param[in]   angle Current input angle sent to the motor [rad]
 */
void Writer::multiturnUpdate(int id, float angle)
{
    Motor motor = m_hal->getMotorFromID(id);

    if (motor.multiturn && multiturnOverLimit(angle)) {
        m_hal->updateResetStatus(id, 1);
    }
}

/**
 * @brief       Check if the goal position will place the motor over a full turn
 * @param[in]   angle Current goal position sent to a motor [rad]
 * @retval      1 if over a full turn, 0 otherwise
 */
bool Writer::multiturnOverLimit(float angle)
{
    if (angle > 2*M_PI || angle < -2*M_PI)
        return true;
    else
        return false;
}

}