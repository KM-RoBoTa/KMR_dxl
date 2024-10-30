# Introduction {#mainpage}
[TOC]

This library is a C++ wrapper around Dynamixel's SDK library for **protocol 2** motors, written for Linux, currently supporting x86 and armv7l processor architectures. <br /> 
It uses SI units, the only exception being temperature expressed in °C instead of Kelvins.

Its main strength lies in its hardware abstraction layer that frees the user of having to check the motors' control tables. Many common functions are available out of the box, using Dynamixel's sync write and sync read functions in the background. <br /> 
For not implemented functionalities, as well as for the creation of custom readers and writers (including indirect ones), this library provides functions to create and destroy them very easily.

In addition to a full documentation, this library also provides several examples to illustrate its use.


## Working concept



Instead of having to manually create Dynamixel's GroupSyncWrite and GroupSyncRead objects and inputting the required addresses and byte lengths of their control fields, everything is done through this library's KMR::dxl::Reader and KMR::dxl::Writer classes.

The KMR::dxl::BaseRobot class provides a basis to be inherited by a Robot class specific to the project. BaseRobot contains general-use functions such as enabling/disabling motors and resetting them in multiturn control mode. <br /> 
The user needs to create custom Reader and Writer objects in their child Robot class, handling the fields they need, as well as their reading/writing functions. Those functions are very straightforward to implement.

This library is written in C++. <br />  



## Links

- Repository: https://github.com/KM-RoBoTa/KMR_dxl
- How to [setup](setup.md)
- How to [use](use.md)

## About

KM-RoBoTa SA's KMR_dxl library to facilitate Dynamixel control.

### Authors
Library written by Katarina Lichardova: katarina.lichardova@km-robota.com

based on code from:
- Laura Paez: laura.paez@km-robota.com
- Kamilo Melo: kamilo.melo@km-robota.com

### Copyright
Copyright 2021-2024, Kamilo Melo. <br /> 
This code is under MIT licence: https://opensource.org/licenses/MIT


## Supported models

- MX-28
- MX-64
- MX-106
- XH540-W150
- XH540-W270
- XM540-W150
- XM540-W270
- XM430-W350
- XW430-T200
- XW540-T260
- XW540-T140
