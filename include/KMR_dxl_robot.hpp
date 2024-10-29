/**
 ******************************************************************************
 * @file            KMR_dxl_robot.hpp
 * @brief           Header for the KMR_dxl_robot.cpp file.
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

/**
 * @brief   Class that defines a base robot, to be inherited by a robot class in the project
 * @details This class contains base necessities for handling a robot with dynamixel motors. \n 
 *          It provides functions to enable/disable motors, as well as to reset motors in multiturn. \n 
 *          The user needs to create handlers they need (Writers and Readers) for their specific
 *          application, as well as their respective reading/writing functions
 */
class BaseRobot {
public:
    std::vector<int> m_ids;     // List of IDs in the robot
    int m_nbrMotors;
    std::vector<int> m_models; // List of models of the motors

    BaseRobot(std::vector<int> ids, const char *port_name, int baudrate);
    ~BaseRobot();

    // Easy handlers creations
    Writer* getNewWriter(std::vector<ControlTableItem> fields, std::vector<int> ids);
    Reader* getNewReader(std::vector<ControlTableItem> fields, std::vector<int> ids);
    void deleteWriter(Writer* writer);
    void deleteReader(Reader* reader);
    
    // Enable/disable motor torque
    void enableMotors();
    void disableMotors();

    // Motor setup
    void setControlModes(std::vector<ControlMode> controlModes);  
    void setControlModes(ControlMode controlMode);  
    void setReturnDelayTime(float val); 
    void setMaxVoltage(std::vector<float> maxVoltages);  
    void setMaxVoltage(float maxVoltage);  
    void setMinVoltage(std::vector<float> maxVoltages);   
    void setMinVoltage(float minVoltage);   

    // Multiturn functions
    void resetMultiturnMotors();

    // Set limits for different operating modes
    void setMinPosition(std::vector<float> minPositions);
    void setMaxPosition(std::vector<float> maxPositions);
    void setMaxSpeed(std::vector<float> maxSpeeds);
    void setMaxCurrent(std::vector<float> maxCurrents);
    void setMaxPWM(std::vector<float> maxPWMs);
    void setMinPosition(float minPosition);
    void setMaxPosition(float maxPosition);
    void setMaxSpeed(float maxSpeed);
    void setMaxCurrent(float maxCurrent);
    void setMaxPWM(float maxPWM);

    // Base controls
    void setPositions(std::vector<float> positions);
    bool getPositions(std::vector<float>& positions);
    void setSpeeds(std::vector<float> speeds);
    bool getSpeeds(std::vector<float>& speeds);
    void setCurrents(std::vector<float> currents);
    bool getCurrents(std::vector<float>& currents);
    void setPWMs(std::vector<float> pwms);
    bool getPWMs(std::vector<float>& pwms);
    void setHybrid(std::vector<float> positions, std::vector<float> currents);
    bool getHybrid(std::vector<float>& positions, std::vector<float>& currents);

    // Rebooting
    void reboot(int id);
    void reboot();

protected:
    dynamixel::PortHandler   *portHandler_ = nullptr;
    dynamixel::PacketHandler *packetHandler_ = nullptr;
    Hal* m_hal = nullptr;

    Writer* m_motorEnableWriter = nullptr;
    Writer* m_controlModeWriter = nullptr;

    // Base controls
    Writer* m_positionWriter = nullptr;
    Writer* m_speedWriter = nullptr;
    Writer* m_currentWriter = nullptr;
    Writer* m_PWMWriter = nullptr;
    Reader* m_positionReader = nullptr;
    Reader* m_speedReader = nullptr;
    Reader* m_currentReader = nullptr;
    Reader* m_PWMReader = nullptr;

    void init_comm(const char *port_name, int baudrate, float protocol_version);
    void check_comm();
};

}
#endif
