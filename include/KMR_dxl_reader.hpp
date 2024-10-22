/**
 ******************************************************************************
 * @file            KMR_dxl_reader.hpp
 * @brief           Header for the KMR_dxl_reader.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#ifndef KMR_DXL_READER_HPP
#define KMR_DXL_READER_HPP

#include "KMR_dxl_handler.hpp"

namespace KMR::dxl
{

/**
 * @brief       Custom Reader class that contains a dynamixel::GroupSyncRead object
 * @details 	This custom Reader class simplifies greatly the creation of dynamixel reading handlers. \n 
 * 				It takes care automatically of address assignment, even for indirect address handling. 
 */
class Reader : public Handler
{
public:
	Reader(std::vector<ControlTableItem> list_fields, std::vector<int> ids, std::vector<int> models,
                dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                Hal* hal, bool forceIndirect);
	~Reader();
	void syncRead();
	std::vector<float> getReadingResults(ControlTableItem field);
	std::vector<float> getReadingResults();

protected:
	dynamixel::GroupSyncRead *m_groupSyncReader = nullptr;
	std::vector<std::vector<float>> m_dataFromMotor;

	void clearParam();
	bool addParam(uint8_t id);
	void checkReadSuccessful();
	void populateOutputMatrix();
};

} // namespace KMR::dxl

#endif