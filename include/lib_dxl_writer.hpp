/**
 * KM-Robota library
 ******************************************************************************
 * @file            lib_dxl_writer.hpp
 * @brief           Header for the lib_dxl_writer.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ******************************************************************************
 */

#ifndef LIB_DXL_WRITER_HPP
#define LIB_DXL_WRITER_HPP

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "lib_hal.hpp"
#include <cstdint>
#include "lib_dxl_handler.hpp"

namespace KMR::dxl
{

class LibDxlWriter: public LibDxlHandler {
    private:
        dynamixel::GroupSyncWrite* m_groupSyncWriter;
        uint8_t** m_dataParam; 

        int angle2Position(float angle, int id);
        void bindParameter(int lower_bound, int upper_bound, int& param);
        void populateDataParam(int32_t data, int motor_idx, int field_idx, int field_length);
        void clearParam();
        bool addParam(uint8_t id, uint8_t* data);

    public:
        LibDxlWriter(std::vector<Fields> list_fields, std::vector<int> ids, dynamixel::PortHandler *portHandler,
                            dynamixel::PacketHandler *packetHandler, LibHal hal, bool forceIndirect);
        ~LibDxlWriter();
        template <typename T>
        void addDataToWrite(std::vector<T> data, Fields field, std::vector<int> ids);
        void syncWrite(std::vector<int> ids);
};

// Templates need to be defined in hpp

/**
 * @brief       Update the current to-be-sent-to-motors data table with fresh values
 * @param[in]   data Fresh data to be sent to motors (eg, new goal positions). \n 
 *              NB: If only one value is input, it will be sent to all motors
 * @param[in]   field Control field to receive the data
 * @param[in]   ids List of motors that will receive the data
 * @retval      Void
 */
template <typename T>
void LibDxlWriter::addDataToWrite(std::vector<T> data, Fields field, std::vector<int> ids)
{
    int field_length;
    int field_idx;
    checkIDvalidity(ids);
    checkFieldValidity(field);

    getFieldPosition(field, field_idx, field_length);
    T current_data;
    int param_data;
    float units;
    int id;
    int motor_idx = 0;

    for(int i=0; i<ids.size(); i++){
        id = ids[i];
        units = m_hal.getControlParametersFromID(id, field).unit;
        motor_idx = getMotorIndexFromID(id);

        if(data.size() == 1)
            current_data = data[0];
        else    
            current_data = data[i];

        // Transform data into its parametrized form and write it into the parametrized data matrix
        if (field != GOAL_POS && field != PRESENT_POS &&
            field != MIN_POS_LIMIT && field != MAX_POS_LIMIT &&
            field != HOMING_OFFSET) {
            param_data = current_data / units;        
        }
        else
            param_data = angle2Position(current_data, id);

        // debug
/*         std::cout << std::endl;
        std::cout << "Going to populate" << std::endl;    
        std::cout << "Field: " << field << "Current data: " << current_data << std::endl;
        std::cout << "Parametrized data: " << (int) param_data << std::endl;
        std::cout << "Motor idx: " << motor_idx << std::endl;
        std::cout << "Field idx: " << field_idx << std::endl; */


        populateDataParam(param_data, motor_idx, field_idx, field_length);

    }

    // debug
/*     std::cout << std::endl;
    std::cout << "TABLE AFTER AN ADDITION" << std::endl;
    for(int i=0; i<m_ids.size();i++){
        for(int j=0; j<m_data_byte_size; j++){
            std::cout << " " << (int) m_dataParam[i][j];
        }
        std::cout << std::endl;
    } */
}

}
#endif