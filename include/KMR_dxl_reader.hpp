/**
 *****************************************************************************
 * @file            KMR_dxl_reader.hpp
 * @brief           Declare the Reader class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#ifndef KMR_DXL_READER_HPP
#define KMR_DXL_READER_HPP

#include "KMR_dxl_handler.hpp"

namespace KMR::dxl
{

/**
 * @brief   Custom Reader class that handles any reading (receiving feedback) from motors
 * @details This custom Reader class simplifies greatly the creation of dynamixel reading handlers. \n 
 *          Each Reader object contains a dynamixel::GroupSyncRead object that enables synchronized
 *          reading from all motors. In addition, it takes care automatically of address assignment,
 *          even for indirect addresses, based on the fields and motor models handled by said object
 */      
class Reader : public Handler
{
public:
	Reader(std::vector<ControlTableItem> list_fields, std::vector<int> ids, std::vector<int> models,
                dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                Hal* hal, bool forceIndirect);
	~Reader();
	bool syncRead();
	std::vector<float> getReadingResults(ControlTableItem field);
	std::vector<float> getReadingResults();

protected:
	dynamixel::GroupSyncRead *m_groupSyncReader = nullptr;
	std::vector<std::vector<float>> m_dataFromMotor;

	void clearParam();
	bool addParam(uint8_t id);
	bool dataAvailable();
	void populateOutputMatrix();

private:
	bool canBeNegative(ControlTableItem field);
};

} // namespace KMR::dxl

#endif