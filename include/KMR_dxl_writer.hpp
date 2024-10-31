/**
 *****************************************************************************
 * @file            KMR_dxl_writer.hpp
 * @brief           Declare the Writer class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#ifndef KMR_DXL_WRITER_HPP
#define KMR_DXL_WRITER_HPP

#include <cstdint>
#include "KMR_dxl_handler.hpp"

namespace KMR::dxl
{

/**
 * @brief   Custom Writer class that handles any writing (sending) to motors
 * @details This custom Writer class simplifies greatly the creation of dynamixel writing handlers. \n 
 *          Each Writer object contains a dynamixel::GroupSyncWrite object that enables synchronized
 *          writing to all motors. In addition, it takes care automatically of address assignment,
 *          even for indirect addresses, based on the fields and motor models handled by said object
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
    uint8_t **m_dataParam = nullptr; // Table containing all parametrized data to be sent

    void populateDataParam(int32_t data, int motor_idx, int field_idx, int field_length);
    void clearParam();
    bool addParam(uint8_t id, uint8_t *data);

    void multiturnUpdate(int id, float angle);
    bool multiturnOverLimit(float angle);
};


// Templates need to be defined in hpp

/**
 * @brief       Add data to the list to be sent later with syncWrite()
 * @note        If the Writer object handles only one field, you can use the overload
 *              function that does not need the field argument
 * @param[in]   data Data to be sent to all motors handled by this object (eg, new goal positions),
 *              in SI units. NB: If vector of size 1, its value will be sent to all motors
 * @param[in]   field Control field of the data (eg goal position)
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
        int idx = getIndex(m_fields, field);
        T data = current_data + m_offsets[idx][i];  // Go to the same reference as Dynamixel's SDK

        int32_t parameter = 0;
        int32_t absParam = (int32_t) abs((float)data/m_units[idx][i]);

        if (data >= 0)
            parameter = absParam;
        else
            parameter = (~absParam) + 1;  // 2's complement for negative values

        populateDataParam(parameter, i, field_idx, field_length); 

        if (field == ControlTableItem::GOAL_POSITION)
            multiturnUpdate(id, (float)current_data);
    }
}

/**
 * @brief       Add data to the list to be sent later with syncWrite()
 * @note        If the Writer object handles more than a single field, you need to use the 
 *              overload of this function
 * @param[in]   data Data to be sent to all motors handled by this object (eg, new goal positions),
 *              in SI units. NB: If vector of size 1, its value will be sent to all motors
 */
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