/**
 *****************************************************************************
 * @file            KMR_dxl_reader.cpp
 * @brief           Define the Reader class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#include <algorithm>
#include <cstdint>

#include "KMR_dxl_reader.hpp"

using namespace std;

namespace KMR::dxl
{

const int BITS_PER_BYTE = 8;

/**
 * @brief       Constructor for a Reader handler
 * @param[in]   list_fields List of fields to be handled by the reader
 * @param[in]   ids Motors to be handled by the reader
 * @param[in]   models Models of the motors to be handled by the reader
 * @param[in]   portHandler Object handling port communication
 * @param[in]   packetHandler Object handling packets
 * @param[in]   hal Hal object for interface with hardware
 * @param[in]   forceIndirect 1 to force the Handler as indirect when only 1 field
 */
Reader::Reader(vector<ControlTableItem> list_fields, vector<int> ids, vector<int> models,
                dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                Hal* hal, bool forceIndirect)
: Handler(list_fields, ids, models, packetHandler, portHandler, hal, forceIndirect)
{
    m_groupSyncReader = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, m_data_address, m_data_byte_size);

    // Create the table to save read data
    m_dataFromMotor = vector<vector<float>>(m_fields.size());
    vector<float> data(m_nbrMotors, 0);
    for (int i=0; i<m_fields.size(); i++)
        m_dataFromMotor[i] = data;
}


/**
 * @brief Destructor
 */
Reader::~Reader()
{
    delete m_groupSyncReader;
    m_groupSyncReader = nullptr;
}

/*
 *****************************************************************************
 *                             Data reading
 ****************************************************************************/

/**
 * @brief   Read the handled fields of all handled motors
 * @retval  true if read successfully, 0 otherwise
 */
bool Reader::syncRead()
{
    clearParam();    

    // Add the input motors to the reading list
    for (int i=0; i<m_nbrMotors; i++){
        bool dxl_addparam_result = addParam(m_ids[i]);
        if (dxl_addparam_result != true) {
            cout << "Adding parameters failed for ID = " << m_ids[i] << endl;
            return false;
        }
    }

    // Read the motors' sensors
    int dxl_comm_result = m_groupSyncReader->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
        return false;
    }

    if (dataAvailable())
        populateOutputMatrix();

    return true;
}


/** 
 * @brief       Extract the previously read feedbacks with syncRead() from the internal storage
 * @note        If the Reader handles only one field, you can use the overload function that does
 *              not take the field argument
 * @param[in]   field Field we want the feedbacks from (eg present position)
 * @return      Vector of feedbacks for the query field, in SI units
 */
vector<float> Reader::getReadingResults(ControlTableItem field)
{
    int idx = getIndex(m_fields, field);
    vector<float> results;

    if (idx >= 0)
        results = m_dataFromMotor[idx];
    else {
        cout << "Error! This Reader does not handle the asked field. Exiting" << endl;
        exit(1);
    }

    return results;
}

/** 
 * @brief       Extract the previously read feedbacks with syncRead() from the internal storage
 * @note        If the Reader handles more than one field, use the overload function that takes 
 *              the query field as an argument
 * @return      Vector of feedbacks, in SI units
 */
vector<float> Reader::getReadingResults()
{
    if (m_isIndirectHandler) {
        cout << "[Reader handler] Error! This Handler is indirect (at least 2 fields). "
        "Use the getReadingResults(ControlTableItem field) overload instead" << endl;
        exit(1);
    }

    int idx = 0;  // Direct handler (unique field), so index necessarily equal to 0
    vector<float> results = m_dataFromMotor[idx];

    return results;
}






/**
 * @brief   Clear the parameters list: no motors added
 */
void Reader::clearParam()
{
    m_groupSyncReader->clearParam();
}

/**
 * @brief       Add a motor to the list of motors who will read
 * @param[in]   id ID of the motor
 * @retval      true if motor added successfully
 */
bool Reader::addParam(uint8_t id)
{
    bool dxl_addparam_result = m_groupSyncReader->addParam(id);
    return dxl_addparam_result;
}


/**
 * @brief   Check if read data from motors is available
 * @retval  1 if data available, 0 otherwise
 */
bool Reader::dataAvailable()
{
    // Check if groupsyncread data of Dynamixel is available
    int field_idx = 0, field_length = 0;
    uint8_t offset = 0;

    for (int i=0; i<m_nbrMotors; i++){
        for (int j=0; j<m_fields.size(); j++){
            getFieldPosition(m_fields[j], field_idx, field_length);

            bool dxl_getdata_result = m_groupSyncReader->isAvailable(m_ids[i], m_data_address + offset, field_length);

            if (dxl_getdata_result != true) {
                fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed \n", m_ids[i]);
                return false;
            }
            offset += field_length;
        }

        offset = 0;
    }

    return true;
}


/**
 * @brief       The reading being successful, save the read data into the output matrix
 */
void Reader::populateOutputMatrix()
{
    uint8_t offset = 0;

    for (int i=0; i<m_nbrMotors; i++){
        for (int j=0; j<m_fields.size(); j++){
            ControlTableItem field = m_fields[j];
            int id = m_ids[i];

            int field_idx, field_length;
            getFieldPosition(field, field_idx, field_length);
            uint32_t paramData = m_groupSyncReader->getData(id, m_data_address + offset, field_length);

            // Apply 2's complement if the received parameter represents a negative value
            int sign = 0;
            uint32_t absValue = 0;
            if (canBeNegative(field)) {

                int shift = field_length * BITS_PER_BYTE - 1;

                switch (field_length)
                {
                case 1:
                {
                    uint8_t param8 = (uint8_t) paramData;
                    if ( (param8>>shift) == 0) {
                        sign = 1;
                        absValue = paramData;
                    }
                    else {
                        sign = -1;
                        uint8_t absValue8 = ~(param8-1);
                        absValue = (uint32_t) absValue8;
                    }
                    break;
                }
                case 2:
                {
                    uint16_t param16 = (uint16_t) paramData;
                    if ( (param16>>shift) == 0) {
                        sign = 1;
                        absValue = paramData;
                    }
                    else {
                        sign = -1;
                        uint16_t absValue16 =  ~(param16-1);
                        absValue = (uint32_t) absValue16;
                    }
                    break;
                }
                case 4:
                {
                    uint32_t param32 = (uint32_t) paramData;
                    if ( (param32>>shift) == 0) {
                        sign = 1;
                        absValue = paramData;
                    }
                    else {
                        sign = -1;
                        uint32_t absValue32 = ~(param32-1);
                        absValue = (uint32_t) absValue32;
                    }
                    break;    
                }
                default:
                    cout << "Error! Field length not supported" << endl;
                    exit(1);
                    break;
                }
            }
            else {
                sign = 1;
                absValue = paramData;
            }

            // Get SI value
            float data =  (float)absValue * (float)sign * m_units[j][i] - m_offsets[j][i];

            // Save the converted value into the output matrix
            m_dataFromMotor[j][i] = data;

            // Offset for the data address
            offset += field_length;
        }

        offset = 0;
    }
}


/** 
 * @brief   Check if the field can have negative feedback values
 * @param   field Query field
 * @return  1 if feedback values can be negative, 0 otherwise
 */
bool Reader::canBeNegative(ControlTableItem field)
{
    using enum ControlTableItem;

    switch (field)
    {
    case MODEL_NBR:             return false;   break;
    case MODEL_INFO:            return false;   break;
    case FIRMWARE:              return false;   break;
    case ID:                    return false;   break;
    case BAUDRATE:              return false;   break;
    case RETURN_DELAY:          return false;   break;
    case DRIVE_MODE:            return false;   break;
    case OPERATING_MODE:        return false;   break;
    case SHADOW_ID:             return false;   break;
    case PROTOCOL:              return false;   break;
    case HOMING_OFFSET:         return true;    break;
    case MOVING_THRESHOLD:      return false;   break;
    case TEMPERATURE_LIMIT:     return false;   break;
    case MAX_VOLTAGE_LIMIT:     return false;   break;
    case MIN_VOLTAGE_LIMIT:     return false;   break;
    case PWM_LIMIT:             return false;   break;
    case CURRENT_LIMIT:         return false;   break;
    case ACCELERATION_LIMIT:    return false;   break;
    case VELOCITY_LIMIT:        return false;   break;
    case MAX_POSITION_LIMIT:    return false;   break;
    case MIN_POSITION_LIMIT:    return false;   break;
    case SHUTDOWN:              return false;   break;

    case TORQUE_ENABLE:         return false;   break;
    case LED:                   return false;   break;
    case STATUS_RETURN:         return false;   break;
    case REGISTERED:            return false;   break;
    case HARDWARE_ERROR:        return false;   break;
    case VELOCITY_I_GAIN:       return false;   break;
    case VELOCITY_P_GAIN:       return false;   break;
    case POSITION_D_GAIN:       return false;   break;
    case POSITION_I_GAIN:       return false;   break;
    case POSITION_P_GAIN:       return false;   break;
    case FF_2ND_GAIN:           return false;   break;
    case FF_1ST_GAIN:           return false;   break;
    case BUS_WATCHDOG:          return true;    break;
    case GOAL_PWM:              return true;    break;
    case GOAL_CURRENT:          return true;    break;
    case GOAL_VELOCITY:         return true;    break;
    case PROFILE_ACCELERATION:  return false;   break;
    case PROFILE_VELOCITY:      return false;   break;
    case GOAL_POSITION:         return true;    break;
    case REALTIME_TICK:         return false;   break;
    case MOVING:                return false;   break;
    case MOVING_STATUS:         return false;   break;
    case PRESENT_PWM:           return true;    break;
    case PRESENT_CURRENT:       return true;    break;
    case PRESENT_VELOCITY:      return true;    break;
    case PRESENT_POSITION:      return true;    break;
    case VELOCITY_TRAJECTORY:   return false;   break;
    case POSITION_TRAJECTORY:   return false;   break;
    case PRESENT_VOLTAGE:       return false;   break;
    case PRESENT_TEMPERATURE:   return false;   break;

    default:
        cout << "Error! Unknown field being tested for being able to have negative feedback" << endl;
        exit(1);
        break;
    }
}


}