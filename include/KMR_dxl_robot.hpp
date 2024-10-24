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
    int *scanned_motor_models = nullptr;  // Dynamixel-defined model numbers of motors in the robot
    std::vector<int> m_ids; // List of IDs in the robot
    int m_nbrMotors;
    std::vector<int> m_models; // List of IDs in the robot

    BaseRobot(std::vector<int> ids, const char *port_name, int baudrate);
    ~BaseRobot();

    // Easy handlers creations
    Writer* getNewWriter(std::vector<ControlTableItem> fields, std::vector<int> ids);
    Reader* getNewReader(std::vector<ControlTableItem> fields, std::vector<int> ids);
    void deleteWriter(Writer* writer);
    void deleteReader(Reader* reader);
    
    void enableMotors();
    void disableMotors();
    //void resetMultiturnMotors();
    void setMinPosition(std::vector<float> minPositions, std::vector<int> ids);
    void setMaxPosition(std::vector<float> minPositions, std::vector<int> ids);

    void setMaxVoltage(std::vector<float> maxVoltages);  
    void setMaxVoltage(float maxVoltage);  
    void setMinVoltage(std::vector<float> maxVoltages);   
    void setMinVoltage(float minVoltage);  

    void setControlModes(std::vector<ControlMode> controlModes);  
    void setControlModes(ControlMode controlMode);  
    void setReturnDelayTime(float val); 

    // Base controls
    void setPositions(std::vector<float> positions);
    void getPositions(std::vector<float>& positions);
    void setSpeeds(std::vector<float> speeds);
    void getSpeeds(std::vector<float>& speeds);
    void setCurrents(std::vector<float> currents);
    void getCurrents(std::vector<float>& currents);
    void setPWMs(std::vector<float> pwms);
    void getPWMs(std::vector<float>& pwms);
    
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
    
    /*void setMultiturnControl_singleMotor(int id, Motor motor);
    void setPositionControl_singleMotor(int id, Motor motor);*/
};

}
#endif
