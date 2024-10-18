/**
 ******************************************************************************
 * @file            KMR_dxl_reader.cpp
 * @brief           Defines the Reader class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT 
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include "KMR_dxl_reader.hpp"
#include <algorithm>
#include <cstdint>


using namespace std;

namespace KMR::dxl
{

/**
 * @brief       Constructor for a Reader handler
 * @param[in]   list_fields List of fields to be handled by the reader
 * @param[in]   ids Motors to be handled by the reader
 * @param[in]   portHandler Object handling port communication
 * @param[in]   packetHandler Object handling packets
 * @param[in]   hal Previouly initialized Hal object
 * @param[in]   forceIndirect Boolean: 1 to force the reader to be indirect address
 *              (has no effect if at least 2 fields)
 */
Reader::Reader(vector<ControlTableItem> list_fields, vector<int> ids, vector<int> models,
                dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                Hal* hal, bool forceIndirect)
: Handler(list_fields, ids, models, packetHandler, portHandler, hal, forceIndirect)
{
    m_groupSyncReader = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, m_data_address, m_data_byte_size);

    // Create the table to save read data
    m_dataFromMotor = new float *[m_ids.size()]; 
    for (int i=0; i<m_ids.size(); i++)
        m_dataFromMotor[i] = new float[m_fields.size()];
}


/**
 * @brief Destructor
 */
Reader::~Reader()
{
    delete m_groupSyncReader;

    for (int i=0; i<m_ids.size(); i++)
        delete[] m_dataFromMotor[i];
    delete[] m_dataFromMotor;
}

/*
 *****************************************************************************
 *                             Data reading
 ****************************************************************************/

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
 * @retval      bool: true if motor added successfully
 */
bool Reader::addParam(uint8_t id)
{
    bool dxl_addparam_result = m_groupSyncReader->addParam(id);
    return dxl_addparam_result;
}

/**
 * @brief       Read the handled fields of input motors
 * @param[in]   ids List of motors whose fields will be read 
 */
void Reader::syncRead(vector<int> ids)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = 0;

    clearParam();    

    // Add the input motors to the reading list
    for (int i=0; i<ids.size(); i++){
        dxl_addparam_result = addParam(ids[i]);
        if (dxl_addparam_result != true) {
            cout << "Adding parameters failed for ID = " << ids[i] << endl;
        }
    }

    // Read the motors' sensors
    dxl_comm_result = m_groupSyncReader->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
    }

    checkReadSuccessful(ids);
    populateOutputMatrix(ids);
}


/**
 * @brief       Check if read data from motors is available
 * @param[in]   ids List of motors whose fields have just been read
 */
void Reader::checkReadSuccessful(vector<int> ids)
{
    // Check if groupsyncread data of Dyanamixel is available
    bool dxl_getdata_result = false;
    ControlTableItem field;
    int field_idx = 0;
    int field_length = 0;
    uint8_t offset = 0;

    for (int i=0; i<ids.size(); i++){
        for (int j=0; j<m_fields.size(); j++){
            field = m_fields[j];
            getFieldPosition(field, field_idx, field_length);

            dxl_getdata_result = m_groupSyncReader->isAvailable(ids[i], m_data_address + offset, field_length);

            if (dxl_getdata_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed \n", ids[i]);
            }

            offset += field_length;
        }

        offset = 0;
    }
}


/**
 * @brief       The reading being successful, save the read data into the output matrix
 * @param[in]   ids List of motors whose fields have been successfully read
 */
void Reader::populateOutputMatrix(vector<int> ids)
{
    uint8_t offset = 0;

    for (int i=0; i<ids.size(); i++){
        for (int j=0; j<m_fields.size(); j++){
            ControlTableItem field = m_fields[j];
            int id = ids[i];

            int field_idx, field_length;
            getFieldPosition(field, field_idx, field_length);
            int32_t paramData = m_groupSyncReader->getData(id, m_data_address + offset, field_length);

            float data =  (float) paramData * m_units[j][i] - m_offsets[j][i];

            // Save the converted value into the output matrix
            for (int row=0; row<ids.size(); row++){
                for(int col=0; col<m_fields.size(); col++){
                    if (ids[row] == id && m_fields[col] == field) {
                        m_dataFromMotor[row][col] = data;
                        goto loop_break;
                    }
                }
            }

            loop_break:

            // Offset for the data address
            offset += field_length;
        }

        offset = 0;
    }
}


// TO DELETE?
/**
 * @brief       Convert position into angle based on motor model 
 * @param[in]   position Position to be converted
 * @param[in]   id ID of the motor
 * @param[in]   units Conversion units between the position and the angle
 * @return      Angle position [rad] of the query motor
 */
float Reader::position2Angle(int32_t position, int model, float units)
{
    float offset = m_hal->getPositionOffset(model);
    float angle = (float) position * units - offset;

    return angle;
}

}