/**
 ******************************************************************************
 * @file            KMR_dxl_writer.hpp
 * @brief           Header for the KMR_dxl_writer.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#ifndef KMR_DXL_WRITER_HPP
#define KMR_DXL_WRITER_HPP

#include <cstdint>
#include "KMR_dxl_handler.hpp"

namespace KMR::dxl
{

/**
 * @brief       Custom Writer class that contains a dynamixel::GroupSyncWrite object
 * @details 	This custom Writer class simplifies greatly the creation of dynamixel writing handlers. \n 
 * 				It takes care automatically of address assignment, even for indirect address handling. 
 */
class Writer : public Handler
{
public:
    Writer(std::vector<ControlTableItem> list_fields, std::vector<int> ids, std::vector<int> models,
            dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
            Hal* hal, bool forceIndirect);
    ~Writer();
    template <typename T> void addDataToWrite(std::vector<T> data, ControlTableItem field);
    template <typename T> void addDataToWrite(std::vector<T> data);
    void syncWrite();

private:
    dynamixel::GroupSyncWrite *m_groupSyncWriter = nullptr;
    uint8_t **m_dataParam = nullptr; // Table containing all parametrized data to be sent next step

    int angle2Position(float angle, int id, float units); // TO DELETE LATER: multiturn stuff
    void populateDataParam(int32_t data, int motor_idx, int field_idx, int field_length);
    void clearParam();
    bool addParam(uint8_t id, uint8_t *data);

    void multiturnUpdate(int id, float angle);
    bool multiturnOverLimit(float angle);
};

// Templates need to be defined in hpp

/**
 * @brief       Add data to the list to be sent later with syncWrite
 * @param[in]   data Data to be sent to motors (eg, new goal positions), in SI units. \n
 *              NB: If only one value is input, it will be sent to all input motors
 * @param[in]   field Control field to receive the data
 * @param[in]   ids List of motors that will receive the data
 */
template <typename T>
void Writer::addDataToWrite(std::vector<T> data, ControlTableItem field)
{
    int field_length;
    int field_idx;

    checkFieldValidity(field);
    getFieldPosition(field, field_idx, field_length);

    for (int i=0; i<m_nbrMotors; i++)
    {
        int id = m_ids[i];

        T current_data;
        if (data.size() == 1)
            current_data = data[0];
        else
            current_data = data[i];

        // Transform data into its parametrized form and write it into the parametrized data matrix
        T data = current_data + m_offsets[field_idx][i];  // Go to the same reference as Dynamixel's SDK

        int32_t parameter = 0;
        int32_t absParam = (int32_t) abs((float)data/m_units[field_idx][i]);

        if (data >= 0)
            parameter = absParam;
        else
            parameter = (~absParam) + 1;  // 2's complement for negative values

        populateDataParam(parameter, i, field_idx, field_length);

        if (field == GOAL_POSITION)
            multiturnUpdate(id, (float)current_data);
    }
}


template <typename T>
void Writer::addDataToWrite(std::vector<T> data)
{
    if (m_isIndirectHandler) {
        std::cout << "[Writer handler] Error! This Handler is indirect (at least 2 fields). "
        "Use the addDataToWrite(vector<T> data, ControlTableItem field) overload instead" << std::endl;
        exit(1);
    }

    ControlTableItem field = m_fields[0];
    addDataToWrite(data, field);
}

}
#endif