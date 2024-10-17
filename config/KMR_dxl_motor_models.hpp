/**
 ******************************************************************************
 * @file            KMR_dxl_motor_models.hpp
 * @brief           Header file including all motor models config files
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 03/2024
 * @authors kamilo.melo@km-robota.com, 03/2024
 ******************************************************************************
 */

#ifndef KMR_DXL_MOTOR_MODELS_HPP
#define KMR_DXL_MOTOR_MODELS_HPP

#include "KMR_dxl_structures.hpp"

#include "motor_models/MX_28.hpp"
#include "motor_models/MX_64.hpp"
#include "motor_models/MX_106.hpp"
#include "motor_models/XH540_W150.hpp"
#include "motor_models/XH540_W270.hpp"
#include "motor_models/XM430_W350.hpp"
#include "motor_models/XM540_W150.hpp"
#include "motor_models/XM540_W270.hpp"
#include "motor_models/XW430_T200.hpp"
#include "motor_models/XW540_T140.hpp"
#include "motor_models/XW540_T260.hpp"

namespace KMR::dxl
{

#define MODEL_NBR_MX_28         30
#define MODEL_NBR_MX_64         311
#define MODEL_NBR_MX_106        321
#define MODEL_NBR_XH540_W150    1110
#define MODEL_NBR_XH540_W270    1100
#define MODEL_NBR_XM430_W350    1020
#define MODEL_NBR_XM540_W150    1130
#define MODEL_NBR_XM540_W270    1120
#define MODEL_NBR_XW430_T200    1280
#define MODEL_NBR_XW540_T140    1180
#define MODEL_NBR_XW540_T260    1170

}

#endif