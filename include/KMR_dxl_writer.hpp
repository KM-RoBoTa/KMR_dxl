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
    template <typename T>
    void addDataToWrite(std::vector<T> data, ControlTableItem field);
    void syncWrite(std::vector<int> ids);

private:
    dynamixel::GroupSyncWrite *m_groupSyncWriter = nullptr;
    uint8_t **m_dataParam = nullptr; // Table containing all parametrized data to be sent next step

    int angle2Position(float angle, int id, float units);
    void bindParameter(int lower_bound, int upper_bound, int &param);
    void populateDataParam(int32_t data, int motor_idx, int field_idx, int field_length);
    void clearParam();
    bool addParam(uint8_t id, uint8_t *data);
    bool multiturnOverLimit(int position);
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
        int param_data = (int) ( (current_data + m_offsets[field_idx][i]) / m_units[field_idx][i] );

        // TODO : RESET MULTITURN STATUS (check old angle 2 pos function)

        populateDataParam(param_data, i, field_idx, field_length);

        // Find a way to link it with writing?
    }
}

}
#endif