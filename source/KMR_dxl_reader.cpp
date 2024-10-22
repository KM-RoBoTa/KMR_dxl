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
    m_dataFromMotor = vector<vector<float>>(m_fields.size());
    vector<float> data(m_nbrMotors, 0);
    for (int i=0; i<m_nbrMotors; i++)
        m_dataFromMotor[i] = data;

}


/**
 * @brief Destructor
 */
Reader::~Reader()
{
    delete m_groupSyncReader;
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
void Reader::syncRead()
{
    clearParam();    

    // Add the input motors to the reading list
    for (int i=0; i<m_ids.size(); i++){
        bool dxl_addparam_result = addParam(m_ids[i]);
        if (dxl_addparam_result != true) {
            cout << "Adding parameters failed for ID = " << m_ids[i] << endl;
        }
    }

    // Read the motors' sensors
    int dxl_comm_result = m_groupSyncReader->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
    }

    checkReadSuccessful();
    populateOutputMatrix();
}


/**
 * @brief       Check if read data from motors is available
 * @param[in]   ids List of motors whose fields have just been read
 */
void Reader::checkReadSuccessful()
{
    // Check if groupsyncread data of Dyanamixel is available
    int field_idx = 0, field_length = 0;
    uint8_t offset = 0;

    for (int i=0; i<m_nbrMotors; i++){
        for (int j=0; j<m_fields.size(); j++){
            getFieldPosition(m_fields[j], field_idx, field_length);

            bool dxl_getdata_result = m_groupSyncReader->isAvailable(m_ids[i], m_data_address + offset, field_length);

            if (dxl_getdata_result != true)
                fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed \n", m_ids[i]);

            offset += field_length;
        }

        offset = 0;
    }
}


/**
 * @brief       The reading being successful, save the read data into the output matrix
 * @param[in]   ids List of motors whose fields have been successfully read
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
            int32_t paramData = m_groupSyncReader->getData(id, m_data_address + offset, field_length);

            float data =  (float) paramData * m_units[j][i] - m_offsets[j][i];

            // Save the converted value into the output matrix
            m_dataFromMotor[j][i] = data;

            // Offset for the data address
            offset += field_length;
        }

        offset = 0;
    }
}


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

}