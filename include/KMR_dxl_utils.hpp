/**
 ******************************************************************************
 * @file            writer.hpp
 * @brief           Header for the writer.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

#ifndef KMR_DXL_UTILS_HPP
#define KMR_DXL_UTILS_HPP

#include <vector>
#include <algorithm>
#include <iostream>
#include <unistd.h> // Provides the usleep function


// Trigonometry
float deg2rad(float deg);
float rad2deg(float rad);

// Time-measuring functions
timespec time_s();
double get_delta_us(struct timespec t2, struct timespec t1);

// Conversions
std::string convertToHex(int dec) ;


template<typename T>
T saturate(T min, T max, T val)
{
    if (val < min)
        return min;
    else if (val > max)
        return max;
    else
        return val;
}

// Result between -b and +b
template<typename T>
T modulo(T a, T b)
{
    if (b == 0) {
        std::cout << "Error! Modulo by 0" << std::endl;
        exit(1);
    }

    int k = (int)( (float)a /(float)b );
    std::cout << "k = " << k << std::endl;
    return (a-k*b);
}

// Function to return the index of an element k
template<typename T>
int getIndex(std::vector<T> v, T k)
{
    auto it = std::find(v.begin(), v.end(), k); 
    int index = -1;
  
    // If element was found 
    if (it != v.end())  
        index = it - v.begin(); 
    return index;    
}


#endif