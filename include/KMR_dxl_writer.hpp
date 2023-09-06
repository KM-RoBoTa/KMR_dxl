/**
 * KM-Robota library
 ******************************************************************************
 * @file            writer.hpp
 * @brief           Header for the writer.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ******************************************************************************
 */

#ifndef KMR_DXL_WRITER_HPP
#define KMR_DXL_WRITER_HPP

#include <cstdint>
#include "KMR_dxl_handler.hpp"

namespace KMR::dxl
{

class Writer : public Handler
{
private:
    dynamixel::GroupSyncWrite *m_groupSyncWriter;
    uint8_t **m_dataParam;

    int angle2Position(float angle, int id);
    void bindParameter(int lower_bound, int upper_bound, int &param);
    void populateDataParam(int32_t data, int motor_idx, int field_idx, int field_length);
    void clearParam();
    bool addParam(uint8_t id, uint8_t *data);

public:
    Writer(std::vector<Fields> list_fields, std::vector<int> ids, dynamixel::PortHandler *portHandler,
            dynamixel::PacketHandler *packetHandler, Hal hal, bool forceIndirect);
    ~Writer();
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
void Writer::addDataToWrite(std::vector<T> data, Fields field, std::vector<int> ids)
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

    for (int i = 0; i < ids.size(); i++)
    {
        id = ids[i];
        units = m_hal.getControlParametersFromID(id, field).unit;
        motor_idx = getMotorIndexFromID(id);

        if (data.size() == 1)
            current_data = data[0];
        else
            current_data = data[i];

        // Transform data into its parametrized form and write it into the parametrized data matrix
        if (field != GOAL_POS && field != PRESENT_POS &&
            field != MIN_POS_LIMIT && field != MAX_POS_LIMIT &&
            field != HOMING_OFFSET)
        {
            param_data = current_data / units;
        }
        else
            param_data = angle2Position(current_data, id);

        populateDataParam(param_data, motor_idx, field_idx, field_length);
    }

}

}
#endif