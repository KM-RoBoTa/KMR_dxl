/**
 *****************************************************************************
 * @file            KMR_dxl_handler.hpp
 * @brief           Declare the Handler class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#ifndef KMR_DXL_HANDLER_HPP
#define KMR_DXL_HANDLER_HPP

#include <cstdint>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "KMR_dxl_hal.hpp"
#include "KMR_dxl_utils.hpp"


namespace KMR::dxl
{

/**
 * @brief       Abstract parent class, to be specialized as a Reader or Writer
 * @details		This class is not usable by itself, it is a non-specialized skeleton inherited
 * 				by the child classes Reader and Writer. \n
 * 				It contains functionalities to check the viability of sync readers/writers 
 * 				that will be defined in child classes (motor compatibility). \n 
 * 				It also takes care of setting indirect addresses should the Handler be indirect
 */
class Handler
{
protected:
	int m_nbrMotors;							// Number of handled motors
	std::vector<int> m_ids;						// IDs of motors handled by this Handler
	std::vector<int> m_models;					// Models of the handled motors

	std::vector<ControlTableItem> m_fields;		// All Fields handled by this specific handler
	bool m_isIndirectHandler;					// 1 if the handler is indirect, 0 otherwise

	dynamixel::PacketHandler *packetHandler_;	// Handler for communication packets
	dynamixel::PortHandler *portHandler_;		// Handler for the serial port
	Hal* m_hal;									// Hal object for interface with hardware

	uint8_t m_data_address = -1;				// Address where the data is written/read
	uint8_t m_data_byte_size = 0;				// Total data byte size handled by the handler	

	// SI-to-parameters conversion variables
	std::vector<std::vector<float>> m_units;	// Units to convert from SI to parameter
	std::vector<std::vector<float>> m_offsets;	// SI offsets for custom references
	
	Handler(std::vector<ControlTableItem> list_fields, std::vector<int> ids, std::vector<int> models,
			dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler,
			Hal* hal, bool forceIndirect);

	void checkFieldValidity(ControlTableItem field);
	void getFieldPosition(ControlTableItem field, int &field_idx, int &field_length);

	// Methods that need to be implemented in child classes
	virtual void clearParam() = 0; // Pure virtual function


private:	
	std::vector<int> m_field_indices;	// Byte index of the starts of each field 
	std::vector<int> m_field_lengths;	// Byte size list of fields in the Handler

	// Initialization functions on constructor call
	
	void getDataByteSize();
	void checkMotorCompatibility(ControlTableItem field);
	void getConversionVariables();
	void setIndirectAddresses();
};

} 

#endif