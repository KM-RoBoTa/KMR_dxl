# How to use {#how-to-use}
[TOC]

The library lives in the KMR::dxl namespace. 

## Important: motor angle redefinition

The angles of the motors have been redefined in this library so that they feel more natural.  <br /> 
Dynamixel libraries define the motor angles as indicated in black in the following image:

![](motor_new.png)

with the angle position being in the interval \f$ [ 0; 2\pi] \f$ rad. <br /> 
This library uses the redefined angles as indicated in blue, in the interval  \f$] - \pi, +\pi[\f$ rad, with the 0 position being in the center of the motor.


## Highest-level manager: the MotorHandler class

The MotorHandler class is the highest-level class that manages all communication with the motors. This library requires a single MotorHandler object to be created, and everything will go through it. <br />

On object creation, a MotorHandler object opens the port, initializes communication and pings the motors to make sure they respond correctly. <br />
Additionally, it automatically detects their models. In tandem with its hardware interface layer (hal), the motor handler thus has all the necessary low-level information such as adresses and unit conversions, which means the user does not need to take care of it.

Out of the box, the motor handler comes with the most common functions, such as enabling/disabling motors, setting control modes, sending control commands or getting common feedbacks. The exhaustive list of those functions will be described later. 

In the background, the motor handler has created instances of this library's custom Writer and Reader classes. Those classes contain respectively a Dynamixel's GroupSyncWrite and GroupSyncRead object, as well as additional functionalities such as automatic units-to-parameter conversions, automatic direct and indirect address assignments and motor compability checks. <br />
So for example, the function to assign control modes uses in the background a Writer object that works with the KMR::dxl::ControlTableItem::OPERATING_MODE field. But what happens if the user wants to write/read to fields that are not handled by a premade function, such as reading temperature, or wants a single handler to work with several fields through indirect addresses?

In this case, the user can create custom writers or readers by using functions provided by the MotorHandler class to create (and destroy) those custom handlers very easily. <br />
The user has to write functions that will use those custom handlers, but it is very quick and straightforward. <br />
Custom handlers will be explained later in this documentation.

It is very straightforward to create a MotorHandler object:

```cpp
#define BAUDRATE        1000000
#define PORTNAME        "/dev/ttyUSB0"

// IDs of all motors
std::vector<int> ids = {1, 2};
KMR::dxl::MotorHandler motorHandler(ids, PORTNAME, BAUDRATE);
```

## Provided functions

As mentioned above, the MotorHandler class provides common functions. The complete list of those functions can be found on the class page [documentation](@ref KMR::dxl::MotorHandler).

Almost none of those functions take motor identifiers as arguments. This is because **all of those functions work with all motors** inputted in the class constructor. Which means, for example when inputting the control modes to motors with the function ```MotorHandler::setControlModes(std::vector< ControlMode > controlModes)```, it is expected that the size of the vector ```controlModes``` is equal to the number of motors. The values in the argument vector are distributed to the motors in the same order as the motor IDs were inputted in the constructor. 

Reusing the code as above for the constructor example, the first value in the ```controlModes``` vector would be assigned to motor with ID 1, while the second will go to the motor with ID 2.

> [!warning]
> In order to avoid repetitive checks at every time step, most functions **do not** check if the size of the argument vector is equal to the number of motors. It is up to the user to make sure they match.

Many of provided functions have an overload. <br />
As just explained, the functions taking a vector as argument assign each value in the vector to the corresponding motor. However, in the case where every motor takes the same input value, the overload simplifies things by only taking that value as an argument, and subsequently distributes it to every motor. <br />
For example, when controlling all motors via speed control, one can simply use:
```cpp
#define BAUDRATE        1000000
#define PORTNAME        "/dev/ttyUSB0"

std::vector<int> ids = {1, 2}; // IDs of all motors
KMR::dxl::MotorHandler motorHandler(ids, PORTNAME, BAUDRATE);

motorHandler.disableMotors();  // The operating mode is written to the EEPROM,
                               // so need to disable first

// Set control mode
KMR::dxl::ControlMode mode = KMR::dxl::ControlMode::SPEED;
motorHandler.setControlModes(mode);
sleep(1);                     // Sleep to make sure the info was written
                              // to the EEPROM correctly
motorHandler.enableMotors();  // Reenable the torques 
```

## 


## Custom 






















## Note: multiturn reset
The public method KMR::dxl::BaseRobot::resetMultiturnMotors resets the motors flagged as in need of a reset. It is inherited by the Robot class, and needs to be called only if the project contains multiturn motors. 

It is up to the user where they want to call it. A good idea is to call it at the start of each control loop, before reading the sensor values. <br /> 
If wished, one can also add it for example at the end of the writing functions.

> **Warning** <br> 
> If the resetMultiturnMotors method is called at the end of the writing method, make sure the motors had enough time to execute the movement before calling the reset, such as by adding a short sleep time. If they are reset before they could execute the whole movement, it results in undefined behavior.

