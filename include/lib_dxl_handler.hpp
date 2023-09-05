/**
 * KM-Robota library
 ******************************************************************************
 * @file            lib_dxl_handler.hpp
 * @brief           Header for the lib_dxl_handler.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ******************************************************************************
 */

#ifndef LIB_DXL_HANDLER_HPP
#define LIB_DXL_HANDLER_HPP

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "lib_hal.hpp"
#include <cstdint>

namespace KMR::dxl
{

class LibDxlHandler{
    public:        
        std::vector<int> m_ids;
        bool m_isIndirectHandler;
        std::vector<Fields> m_list_fields;
        std::vector<int> m_field_indices;
        std::vector<int> m_field_lengths;

    protected:
        dynamixel::PacketHandler* packetHandler_;
        dynamixel::PortHandler *portHandler_;
        LibHal m_hal;
        uint8_t m_data_address = -1;
        uint8_t m_data_byte_size = 0;

        void checkMotorCompatibility(Fields field);
        void setIndirectAddresses();
        void getDataByteSize();
        void checkIDvalidity(std::vector<int> ids);
        void checkFieldValidity(Fields field);
        void getFieldPosition(Fields field, int& field_idx, int& field_length);
        int getMotorIndexFromID(int id);

        // Methods that need to be implemented in child classes
        virtual void clearParam() = 0;  // Pure Virtual Function
};

}
#endif