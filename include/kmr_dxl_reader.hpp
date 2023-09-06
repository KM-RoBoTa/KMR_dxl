/**
 * KM-Robota library
 ******************************************************************************
 * @file            reader.hpp
 * @brief           Header for the reader.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ******************************************************************************
 */

#ifndef KMR_DXL_READER_HPP
#define KMR_DXL_READER_HPP

#include "kmr_dxl_handler.hpp"

namespace KMR::dxl
{

class Reader : public Handler
{
protected:
	dynamixel::GroupSyncRead *m_groupSyncReader;

	void clearParam();
	bool addParam(uint8_t id);
	void checkReadSuccessful(std::vector<int> ids);
	void populateOutputMatrix(std::vector<int> ids);
	float position2Angle(int32_t position, int id, float units);

public:
	float **m_dataFromMotor;
	int *motorIndices_dataFromMotor;
	int *fieldIndices_dataFromMotor;

	Reader(std::vector<Fields> list_fields, std::vector<int> ids,
			dynamixel::PortHandler *portHandler,
			dynamixel::PacketHandler *packetHandler, Hal hal, bool forceIndirect);
	~Reader();
	void syncRead(std::vector<int> ids);
};

} // namespace KMR::dxl

#endif