/**
 *****************************************************************************
 * @file            KMR_dxl_hal.hpp
 * @brief           Declare the Hal class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#ifndef KMR_DXL_HAL_HPP
#define KMR_DXL_HAL_HPP

#include <string>
#include <iostream>
#include <cstdint>
#include <vector>

#include "../config/KMR_dxl_motor_models.hpp"
#include "KMR_dxl_utils.hpp"

namespace KMR::dxl
{

/**
 * @brief       Hardware abstraction layer for Dynamixel motors
 * @details     The lowest-level element in the library. The Hal class serves as
 *              an abstraction layer, providing high-level functions to get the Dynamixel control
 *              table addresses and byte sizes
 */
class Hal {
public:
    Hal();
    ~Hal();
    void init(std::vector<int> ids, int nbrMotors, std::vector<int> models);

    // Handlers initializations helpers

    void addMotorOffsetFromID(int id, uint8_t data_length, std::string field_name);

    // Get hardware information

    float getPositionOffset(int modelNumber);
    Field getControlFieldFromModel(int modelNumber, ControlTableItem item);
    Motor getMotorFromID(int id);
    
    // Multiturn functionalities
    
    void setMultiturnMode(int id);
    void updateResetStatus(int id, int status);


private:
    ControlTable* MX_28 = nullptr;
    ControlTable* MX_64 = nullptr;
    ControlTable* MX_106 = nullptr;
    ControlTable* XH540_W150 = nullptr;
    ControlTable* XH540_W270 = nullptr;
    ControlTable* XM430_W350 = nullptr;
    ControlTable* XM540_W150 = nullptr;
    ControlTable* XM540_W270 = nullptr;
    ControlTable* XW430_T200 = nullptr;
    ControlTable* XW540_T140 = nullptr;
    ControlTable* XW540_T260 = nullptr;

    int m_nbrMotors = -1;
    std::vector<int> m_ids;             // IDs of all motors
    std::vector<int> m_models;          // Models of all motors
	std::vector<Motor> m_motorsList;    // Parameters per specific motor

    // Get hardware information, in private scope
    Field getControlField(ControlTable motor, ControlTableItem item);
    ControlTable getControlTable(int modelNumber);
};

}


#endif

