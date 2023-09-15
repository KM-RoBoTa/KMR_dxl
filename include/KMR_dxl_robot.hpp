/**
 * KM-Robota library
 ******************************************************************************
 * @file            kmr_dxl_robot.hpp
 * @brief           Header for the kmr_dxl_robot.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#ifndef KMR_DXL_ROBOT_HPP
#define KMR_DXL_ROBOT_HPP

#include "KMR_dxl_writer.hpp"
#include "KMR_dxl_reader.hpp"

namespace KMR::dxl
{

class BaseRobot {
    protected:
        dynamixel::PortHandler   *portHandler_;
        dynamixel::PacketHandler *packetHandler_;

        Writer *m_motor_enabler;
        Writer *m_controlMode_setter;


        void init_comm(const char *port_name, int baudrate, float protocol_version);
        void check_comm();
        void setMultiturnControl_singleMotor(int id, Motor motor);
        void setPositionControl_singleMotor(int id, Motor motor);

        
    public:
        int *scanned_motor_models;
        Hal m_hal;  // to put private asap @todo
        std::vector<int> m_all_IDs;

        BaseRobot(std::vector<int> all_ids, const char *port_name, int baudrate, Hal hal);
        ~BaseRobot();
       
        void enableMotors();
        void enableMotors(std::vector<int> ids);
        void disableMotors();
        void disableMotors(std::vector<int> ids);
        void resetMultiturnMotors();
};

}
#endif
