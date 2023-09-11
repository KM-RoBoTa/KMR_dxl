/**
 * KM-Robota library
 ******************************************************************************
 * @file            kmr_dxl_writer.cpp
 * @brief           Defines the Writer class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT  !!!!!
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023  !!!
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ******************************************************************************
 */

#include "KMR_dxl_writer.hpp"
#include <algorithm>
#include <cstdint>

#define MULTITURN_MAX   6144
#define MULTITURN_MIN   -2048

using std::cout;
using std::endl;
using std::vector;

using namespace std;

namespace KMR::dxl
{

/**
 * @brief       Constructor for LibDxlWriter
 */
Writer::Writer(vector<Fields> list_fields, vector<int> ids, dynamixel::PortHandler *portHandler,
                            dynamixel::PacketHandler *packetHandler, Hal hal, bool forceIndirect)
{
    portHandler_ = portHandler;
    packetHandler_ = packetHandler;
    m_hal = hal;
    m_ids = ids;
    m_list_fields = list_fields;

    getDataByteSize();

    if (list_fields.size() == 1 && !forceIndirect) {
        m_isIndirectHandler = false;
        checkMotorCompatibility(list_fields[0]);
    }

    else {
        m_isIndirectHandler = true;
        checkMotorCompatibility(INDIR_DATA_1);
        setIndirectAddresses();
    }

    m_groupSyncWriter = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, m_data_address, m_data_byte_size);

    // Create the table to save parametrized data (to be read or sent)
    m_dataParam = new uint8_t *[m_ids.size()];
    for (int i=0; i<m_ids.size(); i++)
        m_dataParam[i] = new uint8_t[m_data_byte_size];

    cout << "DXL HANDLER created!" << endl;

}

/**
 * @brief Destructor
 */
Writer::~Writer()
{
    cout << "The Dxl Writer object is being deleted" << endl;
}


/*
 *****************************************************************************
 *                             Data writing
 ****************************************************************************/
void Writer::clearParam()
{
    m_groupSyncWriter->clearParam();
}

bool Writer::addParam(uint8_t id, uint8_t* data)
{
    bool dxl_addparam_result = m_groupSyncWriter->addParam(id, data);
    return dxl_addparam_result;
}

void Writer::syncWrite(vector<int> ids)
{
    bool dxl_addparam_result;
    int dxl_comm_result = COMM_TX_FAIL;   
    int id, motor_idx;

    clearParam();

    for(int i=0; i<ids.size(); i++) {
        id = ids[i];
        motor_idx = getMotorIndexFromID(id);

        dxl_addparam_result = addParam((uint8_t) id, m_dataParam[motor_idx]);

        if (dxl_addparam_result != true) {
            cout << "Adding parameters failed for ID = " << id << endl;
            exit(1);
        }
    }

    // Send the packet
    dxl_comm_result = m_groupSyncWriter->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
    //else
        //cout << "Sync write executed correctly" << endl;

}

                                                                                                                                                                                     
/**
 * @brief       Convert angle input into position data based on motor model provided
 * @param[in]   angle Angle to be converted, in rad or degrees
 * @param[in]   model Motor model, as encoded on Dynamixel motor's control table
 * @param[in]   isRadians Boolean flag for the angle's units, true if radians (radians by default)
 * @return      Position value corresponding to the joint angle (0 degrees or rad -> mid-position)
 */
int Writer::angle2Position(float angle, int id)
{
/*     cout << endl;
    cout << "Angle at start: " << angle << endl; */


	int position = 2048;
    int motor_idx = m_hal.getMotorsListIndexFromID(id);
    int model = m_hal.m_motors_list[motor_idx].scanned_model;
    float units = m_hal.getControlParametersFromID(id, GOAL_POS).unit;
    Motor motor = m_hal.getMotorFromID(id);

    if (model == 1030 || model == 1000 || model == 311){
    	int Model_max_position = 4095;
        int Model_min_position = 0;
		position = angle/units + Model_max_position/2 + 0.5;

        if (!motor.multiturn)
            bindParameter(Model_min_position, Model_max_position, position);
        else {
            if (multiturnOverLimit(position))
                m_hal.updateResetStatus(id, 1);
        }
    }
    else {
        cout << "This model is unknown, cannot calculate position from angle!" << endl;
        return (1);
    }

    return position;
}

void Writer::bindParameter(int lower_bound, int upper_bound, int& param)
{
    if (param > upper_bound)
        param = upper_bound;
    else if (param < lower_bound)
        param = lower_bound;
}


/**
 * @brief       Transform a target position value into the shape required by Dynamixel motors.
 * @param[in]   data Target value to the motors
 * @param[out]  param_array Transformed data into the required shape, which will be sent to motors
 * @retval      void
 */
void Writer::populateDataParam(int32_t data, int motor_idx, int field_idx, int field_length)
{
    if (field_length == 4) {
        m_dataParam[motor_idx][field_idx+0] = DXL_LOBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][field_idx+1] = DXL_HIBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][field_idx+2] = DXL_LOBYTE(DXL_HIWORD(data));
        m_dataParam[motor_idx][field_idx+3] = DXL_HIBYTE(DXL_HIWORD(data));
    }
    else if (field_length == 2) {
        m_dataParam[motor_idx][field_idx+0] = DXL_LOBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][field_idx+1] = DXL_HIBYTE(DXL_LOWORD(data));
    }
    else if (field_length == 1) {
        m_dataParam[motor_idx][field_idx+0] = DXL_LOBYTE(DXL_LOWORD(data));
    }
    else
        cout<< "Wrong number of parameters to populate the parametrized matrix!" <<endl;
}


bool Writer::multiturnOverLimit(int position)
{
    if (position > MULTITURN_MAX || position < MULTITURN_MIN)
        return true;
    else
        return false;
}



}