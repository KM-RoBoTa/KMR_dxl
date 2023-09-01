/**
 * KM-Robota library
 ******************************************************************************
 * @file            lib_dxl_reader.hpp
 * @brief           Header for the lib_dxl_reader.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ******************************************************************************
 */


#ifndef LIB_DXL_READER_HPP
#define LIB_DXL_READER_HPP

#include "lib_dxl_handler.hpp"


class LibDxlReader: public LibDxlHandler {
    protected:
        dynamixel::GroupSyncRead* m_groupSyncReader;

        void clearParam();
        bool addParam(uint8_t id);
        void checkReadSuccessful(std::vector<int> ids);
        void populateOutputMatrix(std::vector<int> ids);
        float position2Angle(int32_t position, int id, float units);

    public:
        float** m_dataFromMotor;
        int* motorIndices_dataFromMotor;
        int* fieldIndices_dataFromMotor;

        LibDxlReader(std::vector<Fields> list_fields, std::vector<int> ids, dynamixel::PortHandler *portHandler,
                            dynamixel::PacketHandler *packetHandler, LibHal hal, bool forceIndirect);
        ~LibDxlReader();
        void syncRead(std::vector<int> ids);
};



#endif