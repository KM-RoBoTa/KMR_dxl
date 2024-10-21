/**
 *****************************************************************************
 * @file            KMR_dxl_hal.hpp
 * @brief           Header for KMR_dxl_hal.cpp file
 *****************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 *****************************************************************************
 */

#ifndef KMR_DXL_HAL_HPP
#define KMR_DXL_HAL_HPP

#include <string>
#include <iostream>
#include <cstdint>
#include <vector>

#include "KMR_dxl_motor_models.hpp"
#include "KMR_dxl_utils.hpp"

namespace KMR::dxl
{

/**
 * @brief       Hardware abstraction layer for Dynamixel motors
 * @details     The lowest-level element in the library. The Hal class serves primarily as
 *              an abstraction layer, providing high-level functions to get the Dynamixel control
 *              table addresses by creating a control table. \n 
 *              It also parses the project's motors configuration file.
 */
class Hal {
public:
    Hal();
    ~Hal();
    void init(std::vector<int> ids, int nbrMotors, std::vector<int> models);

    // Keep public
    Field getControlFieldFromModel(int modelNumber, ControlTableItem item);
    float getPositionOffset(int modelNumber);



    // TO edit? USED IN HANDLER
    Motor getMotorFromID(int id);
    void addMotorOffsetFromID(int id, uint8_t data_length, std::string field_name);




    // TO DELETE?
    /*int m_tot_nbr_motors;   // Number of motors used in the robot. TO DELETE
    Motor* m_motors_list = nullptr;     // List containing all motors' info 
    std::vector<int> m_all_IDs;  // All motor IDs in the robot
    Motor_data_field** m_control_table = nullptr;   // Table containing all Dynamixel control values for every model
    Control_modes* m_controlModesPerModel = nullptr; // List of control modes values for each Dxl model


    void get_ID_list_from_motors_list();
    Motor_data_field getControlParametersFromID(int id, Fields field); 

    Motor getMotorFromID(int id);
    void addMotorOffsetFromID(int id, uint8_t data, std::string field_name);
    void updateResetStatus(int id, int status);*/

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
    std::vector<int> m_ids;
    std::vector<int> m_models;
	std::vector<Motor> m_motorsList;     // List containing all motors' info

    Field getControlField(ControlTable motor, ControlTableItem item);
    ControlTable getControlTable(int modelNumber);
 

    // TO DELETE?
    /*Motor_models getModelFromID(int id);
    void saveControlValuesToMotors();*/
};

}


#endif

