/**
 * KM-Robota library
 ******************************************************************************
 * @file            lib_robot.hpp
 * @brief           Header for the lib_robot.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ******************************************************************************
 */

#ifndef LIB_ROBOT_HPP
#define LIB_ROBOT_HPP

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "lib_controller.hpp"
#include "lib_hal.hpp"
#include "lib_dxl_writer.hpp"
#include "lib_dxl_reader.hpp"

class LibRobot {
    protected:
        dynamixel::PortHandler   *portHandler_;
        dynamixel::PacketHandler *packetHandler_;

        LibDxlWriter *m_motor_enabler;

        void init_comm(const char *port_name, int baudrate, float protocol_version);
        void check_comm();

        
    public:
        int *scanned_motor_models;
        LibHal m_hal;  // to put private asap @todo
        std::vector<int> m_all_IDs;

        LibRobot(std::vector<int> all_ids, const char *port_name, int baudrate, LibHal hal);
        ~LibRobot();
       
        void enableMotors();
        void enableMotors(std::vector<int> ids);
        void disableMotors();
        void disableMotors(std::vector<int> ids);
};


#endif
