/**
 ******************************************************************************
 * @file            KMR_dxl_handler.cpp
 * @brief           Defines the Handler class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT 
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include "KMR_dxl_handler.hpp"
#include <algorithm>
#include <cstdint>

#define INDIR_OFFSET                2
#define PARAM_OFFSET                1
#define POS_DATA_SIZE               4


using namespace std;

namespace KMR::dxl
{


/*
 *****************************************************************************
 *                                Initializations
 ****************************************************************************/

Handler::Handler(vector<ControlTableItem> list_fields, vector<int> ids, vector<int> models,
                 dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler,
                 Hal* hal, bool forceIndirect)
{
    m_ids = ids;
    m_models = models;
    m_nbrMotors = ids.size();
    packetHandler_ = packetHandler;
    portHandler_ = portHandler;
    m_hal = hal;
    m_fields = list_fields;   

    getDataByteSize();

    if (list_fields.size() == 1 && !forceIndirect) {
        m_isIndirectHandler = false;
        checkMotorCompatibility(list_fields[0]);
    }

    else {
        m_isIndirectHandler = true;
        checkMotorCompatibility(INDIR_DATA_1);
        setIndirectAddresses();
    }

    getConversionVariables();
}


/**
 * @brief       Check if the motors are compatible for a given field: same address for data storing. \n 
 *              For indirect handling, also find the first common address for every motor
 * @param[in]   field Control field that the handler is taking care of
 */
void Handler::checkMotorCompatibility(ControlTableItem field)
{
    uint8_t address = -1;
    uint8_t address_prev = -1;
    int id = -1, id_prev = -1;
    uint8_t biggest_data_offset = 0;
  
    for(int i=1; i<m_nbrMotors; i++){
        id = m_ids[i];
        id_prev = m_ids[i-1];

        address = m_hal->getControlFieldFromModel(m_models[i], field).addr;
        address_prev = m_hal->getControlFieldFromModel(m_models[i-1], field).addr;

        if(address != address_prev){
            cout << "Motors " << id << " and " << id_prev << " have incompatible addresses!" << endl;
            exit(1);
        }

        if (m_isIndirectHandler) {
            if (m_hal->getMotorFromID(id).indir_data_offset > biggest_data_offset)
                biggest_data_offset = m_hal->getMotorFromID(id).indir_data_offset;
        }
    }

    if (m_ids.size() == 1)
        address = m_hal->getControlFieldFromModel(m_models[0], field).addr;

    m_data_address = address + biggest_data_offset;

    // Now that we have the guarantee that all indirect data begin at the same address and we found the first 
    // available common address for all motors for indirect handling, update their offset (already assigned memory)
    if (m_isIndirectHandler) {
        for (int i=0; i<m_ids.size();i++)
            m_hal->addMotorOffsetFromID(m_ids[i], (uint8_t) (biggest_data_offset + m_data_byte_size), "indir_data_offset");
    }

}

/**
 * @brief       Set the indirect addresses: link the direct field address to an indirect address
 */
void Handler::setIndirectAddresses()
{
    uint8_t param_address_offset = 0;

    for (int k=0; k<m_nbrMotors; k++){
        int id = m_ids[k];
        uint8_t indir_address_start = m_hal->getControlFieldFromModel(m_models[k], INDIR_ADD_1).addr;

        for (int i=0; i<m_fields.size(); i++){
            Field field = m_hal->getControlFieldFromModel(m_models[i], m_fields.at(i));

            for (int j=0; j<field.length; j++) {
                uint8_t dxl_error = 0;  
                int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, 
                                                                indir_address_start + m_hal->getMotorFromID(id).indir_address_offset,
                                                                field.addr + param_address_offset, 
                                                                &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                    cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

                m_hal->addMotorOffsetFromID(id, INDIR_OFFSET, "indir_address_offset");
                param_address_offset += PARAM_OFFSET;
            }

            param_address_offset = 0;
        }
    }
}


/**
 * @brief       Calculate and store the byte length of data read/written by the handler. \n 
 *              Also check if the motors are field-compatible (same data lengths required for a given field)
 */
void Handler::getDataByteSize()
{
    ControlTableItem field;
    uint8_t length = 0, length_prev = 0;

    m_field_indices = vector<int> (m_fields.size());
    m_field_lengths = vector<int> (m_fields.size());

    for (int i=0; i<m_fields.size(); i++){
        field = m_fields[i];
        
        for (int j=1; j<m_nbrMotors; j++){
            length = m_hal->getControlFieldFromModel(m_models[j], field).length;
            length_prev = m_hal->getControlFieldFromModel(m_models[j], field).length;  

            if(length != length_prev){
                cout << "Motors " << m_ids[j] << " and " << m_ids[j-1] << " have incompatible field lengths!" << endl;
                exit(1);
            }
        }

        if (m_ids.size() == 1)
            length = m_hal->getControlFieldFromModel(m_models[0], field).length;  

        m_field_lengths[i] = length;
        m_field_indices[i] = m_data_byte_size;
        m_data_byte_size += length;
    }
}


void Handler::getConversionVariables()
{
    int nbrFields = m_fields.size();
    m_units = vector<vector<float>>(nbrFields);
    m_offsets = vector<vector<float>>(nbrFields);

    for (int i=0; i<nbrFields; i++) {
        vector<float> units(m_nbrMotors);
        vector<float> offsets(m_nbrMotors);

        for (int j=0; j<m_nbrMotors; j++) {
            float unit = m_hal->getControlFieldFromModel(m_models[j], m_fields[i]).unit;
            float offset = 0;

            if (m_fields[i] == ControlTableItem::GOAL_POSITION       ||
                m_fields[i] == ControlTableItem::PRESENT_POSITION    ||
                m_fields[i] == ControlTableItem::MIN_POSITION_LIMIT  || 
                m_fields[i] == ControlTableItem::MAX_POSITION_LIMIT  ||
                m_fields[i] == ControlTableItem::HOMING_OFFSET        )
                offset = m_hal->getPositionOffset(m_models[j]);

            units[j] = unit;
            offsets[j] = offset;
        }

        m_units[i] = units;
        m_offsets[i] = offsets;
    }
} 




/*
 *****************************************************************************
 *                        Security checking functions
 ****************************************************************************/

/**
 * @brief       Check if query motors are handled by this specific handler
 * @param[in]   ids List of query motors
 */
void Handler::checkIDvalidity(vector<int> ids)
{
    for(int i=0; i<ids.size(); i++){
        if ( find(m_ids.begin(), m_ids.end(), ids[i]) == m_ids.end() ) {
            cout << "Error: motor " << ids[i] << " is not handled by this handler!" << endl;  
            exit(1);
        }
    }
}

/**
 * @brief       Check if query field is handled by this specific handler
 * @param[in]   field Query control field
 */
void Handler::checkFieldValidity(ControlTableItem field)
{
    if ( find(m_fields.begin(), m_fields.end(), field) == m_fields.end() ) {
        cout << "Error: field " << field << " is not handled by this handler!" << endl;  
        exit(1);
    }
}

/**
 * @brief       Get the index for reading/writing a certain field in a m_data_byte_size array
 * @param[in]   field Query control field
 * @retval      Index of the field to be written/read
 */
void Handler::getFieldPosition(ControlTableItem field, int& field_idx, int& field_length)
{
    for (int i=0; i<m_fields.size(); i++){
        if (m_fields[i] == field){
            field_idx = m_field_indices[i];
            field_length = m_field_lengths[i];
        }
    }
}

/**
 * @brief       Get the index of a motor in the list of handled motors
 * @param[in]   id Query motor
 * @retval      Index of the motor in the list of handled motors
 */
int Handler::getMotorIndexFromID(int id)
{
    int idx = 0;

    while(m_ids[idx] != id)
        idx++;

    return idx; 
}

}
