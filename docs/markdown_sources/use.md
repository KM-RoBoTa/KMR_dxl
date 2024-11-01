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
motorHandler.setControlModes(mode); // Same mode distributed to all motors
sleep(1);                           // Sleep to make sure the info was written
                                    // to the EEPROM correctly
motorHandler.enableMotors();        // Reenable the torques 
```

## Control modes, setting respective limits and examples

The current-based position control mode, called Hybrid in this library, is not finished yet.  <br />
The rest are implemented and ready to use, as well as their respective limit-setting functions. However, it appears the motors' driver deals with limits in a counterintuitive manner.

### Limits

In all modes where the user can set limits (position, current, speed, PWM), the limit setting works in a counterintuitive, and **potentially dangerous** manner.

If a motor receives a command over the set limit (for example, a speed command higher than the value set with setMaxSpeed()), one would expect said command would be saturated and set equal to the limit before being applied by the motor. <br />
*This is not what happens*. If the motor receives a command higher than the set limit, its driver simply ignores it and applies the previous valid command it received. <br />
This means care needs to be taken to make sure any command sent to a motor is lower or equal to the set limits so that it does not get ignored.

> [!warning]
> This library does **not** take care of saturating commands so that they are valid. It is up to the user

Provided examples 1 through 4 are designed for 2 motors in order to illustrate this. The first motor serves as a ground truth, perfectly executing commands that are always lower than its limits. The second motor's limits are set on purpose at times higher than the command in order to showcase those commands being ignored.  


### Multiturn
This library deals with multiturn mode in a custom way. 

By default, dynamixel motors support 256 revolutions in each direction before saturating. In order to avoid this problem, the library is designed to reboot a motor as soon as it gets over \f$ 2\pi \f$ rad, which resets its position between \f$ [ -2\pi; 2\pi] \f$, practically allowing an infinite number of revolutions. 

In the background, as soon as a motor receives a position command with an absolute value higher than \f$ 2\pi \f$ rad, an internal flag requesting a reset is raised. At the start of each loop, the user needs to invoke MotorHandler::resetMultiturnMotors(), which reboots all motors with a raised flag. Those motors are thus re-centered in \f$ [ -2\pi; 2\pi] \f$ rad. This means 2 things:
- A user needs to send *exactly 1* command value over \f$ 2\pi \f$ rad to a motor in order to raise its flag. On the next control loop, the goal position needs to be re-centered in \f$ [ -2\pi; 2\pi] \f$ rad so that it's compatible with the freshly rebooted motor
- The user needs to give enough time to the motors to execute the position command before calling MotorHandler::resetMultiturnMotors(). <br />
**ATTENTION**: if the motors are rebooted before they could execute the whole movement, it results in undefined behavior. This is why it is recommended to call the function at the start of the control loop.

The example 5 illustrates the multiturn mode.

## Custom 

As mentioned earlier, in case the user needs to create custom Writer and Reader handlers, the MotorHandler class offers functions to create and destroy those handlers very easily. 

> [!note]
> This library supports having several indirect address handlers for a given motor. It keeps track of the allocated indirect memory for each motor, and thus assigns to each indirect handler an available indirect memory without creating conflicts. The user does not need to keep track of this

### Custom Writers

A custom Writer can be created by calling the MotorHandler::getNewWriter function. This function takes 2 arguments, the list of control fields and the motor ids the Writer will handle.




> [!caution]
> The Writer object is created on the heap, which means the memory needs to be cleaned to avoid memory leak. To cleanly destroy the object, simply call MotorHandler::deleteWriter();

```cpp
#define BAUDRATE        1000000
#define PORTNAME        "/dev/ttyUSB0"

using namespace std;

vector<int> ids = {1,2};

KMR::dxl::MotorHandler motorHandler(ids, PORTNAME, BAUDRATE);

// Create a custom Writer with only 1 field
// (which results in a direct handler)
vector<KMR::dxl::ControlTableItem> wTempFields =
    {KMR::dxl::ControlTableItem::TEMPERATURE_LIMIT};
KMR::dxl::Writer* tempWriter = motorHandler.getNewWriter(wTempFields, ids);

// Create a custom Writer handling goal velocity and LED
// (which results in an indirect handler) 
vector<KMR::dxl::ControlTableItem> wIndirectFields =
        {KMR::dxl::ControlTableItem::GOAL_VELOCITY,
        KMR::dxl::ControlTableItem::LED};
KMR::dxl::Writer* indirectWriter = motorHandler.getNewWriter(wIndirectFields, ids);

// Do stuff

// Free memory
motorHandler.deleteWriter(tempWriter);
motorHandler.deleteWriter(indirectWriter);
```

A writing function using the new Writer object needs to be created. It consists of 2 parts:
- Adding data we want to write into the handler's internal storage through Writer::addDataToWrite()
- Then sending that data with Writer::syncWrite()

Keeping the same example:

```cpp
// Writing function for the temperature limit writer
void writeTemperatureLimit(vector<float> temperatureLimits)
{
    // The temperature Writer handles only 1 field, so no need to precise 
    // to which field this data goes
    tempWriter->addDataToWrite(temperatureLimits);
    tempWriter->syncWrite();
}

// Writing function for the indirect writer
void writeCommands(vector<float> goalSpeeds, vector<int> leds)
{
    // This Writer handles 2 fields, so the user needs to precise to
    // which field each data vector goes
    writer->addDataToWrite(goalSpeeds, KMR::dxl::ControlTableItem::GOAL_VELOCITY);
    writer->addDataToWrite(leds, KMR::dxl::ControlTableItem::LED);
    writer->syncWrite();
}
```


### Custom Readers

Readers work conceptually in the exact same manner as Writers.

A custom Reader can be created by calling the MotorHandler::getNewReader function. This function takes 2 arguments, the list of control fields and the motor ids the Reader will handle.

> [!caution]
> The Reader object is created on the heap, which means the memory needs to be cleaned to avoid memory leak. To cleanly destroy the object, simply call MotorHandler::deleteReader();

```cpp
#define BAUDRATE        1000000
#define PORTNAME        "/dev/ttyUSB0"

using namespace std;

vector<int> ids = {1,2};

KMR::dxl::MotorHandler motorHandler(ids, PORTNAME, BAUDRATE);

// Create a custom Reader with only 1 field
// (which results in a direct handler)
vector<KMR::dxl::ControlTableItem> wTempFields =
    {KMR::dxl::ControlTableItem::PRESENT_TEMPERATURE};
KMR::dxl::Reader* tempReader = motorHandler.getNewReader(wTempFields, ids);

// Create a custom Reader handling present velocity and LED
// (which results in an indirect handler) 
vector<KMR::dxl::ControlTableItem> wIndirectFields =
        {KMR::dxl::ControlTableItem::PRESENT_VELOCITY,
        KMR::dxl::ControlTableItem::LED};
KMR::dxl::Reader* indirectReader = motorHandler.getNewReader(wIndirectFields, ids);

// Do stuff

// Free memory
motorHandler.deleteReader(tempReader);
motorHandler.deleteReader(indirectReader);
```

A reading function using the new Reader object needs to be created. It consists of 2 parts:
- First getting the feedback from the motors with Reader::syncRead()
- Then extracting the feedback data from the internal storage with Reader::getReadingResults(). Note: all feedback data are floats

Keeping the same example:

```cpp
// Reading function for the present temperature Reader
bool readTemperature(vector<float>& fbckTemperatures)
{
    // Read feedback from the motors
    bool readSuccess = tempReader->syncRead();

    // The temperature Reader handles only 1 field, so no need to precise 
    // from which field we're extracting data
    if (readSuccess) {
        fbckTemperatures = tempReader->getReadingResults();
        return true;
    }
    else
        return false;
}

// Reading function for the indirect reader
bool readFeedback(vector<float>& fbckSpeeds, vector<float>& fbckLeds)
{
    // Read feedback from the motors
    bool readSuccess = indirectReader->syncRead();

    // This Reader handles 2 fields, so the user needs to precise
    // from which field we're extracting
    if (readSuccess) {
        fbckSpeeds = indirectReader->getReadingResults(
                    KMR::dxl::ControlTableItem::PRESENT_VELOCITY);
        fbckLeds = indirectReader->getReadingResults(KMR::dxl::ControlTableItem::LED);    
        return true;
    }
    else
        return false;
}
```

The example 7 showcases how to use custom handlers.


## Examples summary

The examples are designed for 2 motors in protocol 2, with IDs 1 and 2, of any supported model.  <br />
The executables are created on library compilation, and can be found in the ```KMR_dxl/build``` folder.

In almost each example, the first motor shows a perfect execution of commands which are always lower than the set limits, while the second motor's limits are at times set higher than the command on purpose to show the commands being ignored.

The examples included are as follows:

| Example             | Description                  | Custom handlers used? |
|---------------------|------------------------------|-----------------------|
| ex1_position        | Position control showcase    | No                    |
| ex2_speed           | Speed control showcase       | No                    |
| ex3_current         | Current control showcase     | No                    |
| ex4_pwm             | PWM control showcase         | No                    |
| ex5_multiturn       | Multiturn showcase           | No                    |
| ex6_hybrid          | Reserved                     | -                     |
| ex7_custom_handlers | Custom handlers use showcase | Yes                   |