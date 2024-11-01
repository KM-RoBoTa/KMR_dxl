# How to setup

## Dependencies
This library is dependent on the following libraries:
- dynamixel SDK
- CMake
- (Doxygen and Graphviz if you wish to regenerate the documentation locally)

## Dynamixel SDK installation

Please read this whole section before starting installing.  <br /> 
The official installation guide of the SDK can be found on [Dynamixel's website](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/).
However, several corrections need to be done in the code before installing it. <br /> 

Instead, follow those steps: 
### 1. Clone the SDK repo to a path of your choosing:
```bash
cd <your-path>
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

### 2. Make corrections to the SDK
Here is where some things need to be corrected before installing.

**1. Fix compilation warnings** <br /> 
1. In ```<your-path>/DynamixelSDK/c++/src/dynamixel_sdk/group_sync_read.cpp```, add parentheses to the return of the last function ```getError```, so that its return is ```return (error[0] = error_list_[id][0]);```.
2. Make the same correction to the same function in ```<your-path>/DynamixelSDK/c++/src/dynamixel_sdk/group_bulk_read.cpp```.

**2. Fix the Makefile for SBCs** <br /> 
In ```<your-path>/DynamixelSDK/c++/build/linux_sbc/Makefile```, there are tabs at line 56 after ```src/dynamixel_sdk/group_handler.cpp \ ```, breaking the Makefile. Delete those tabs.

**3. Setting the serial port to low latency** <br /> 
By default, serial ports in Linux have a latency of 16ms, which results in Dynamixel reading functions taking forever (~16us for writing, ~16ms for reading). If you plug the motors, you can check the port latency with 
```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```
(edit the /ttyUSBx port if necessary). <br /> 
There are several ways to set a port to low latency (1ms instead of 16ms).
1. (Not recommended) Each time the USB is plugged or the connection dropped, write:
```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```
2. If you want to set it automatically, you can create a udev rule that will set the low latency by using:
```bash
echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\" > 99-dynamixelsdk-usb.rules
sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```
3. Set the low latency when opening the port directly by editing the library (**personal favorite**) <br />
In ```<your-path>/DynamixelSDK/c++/src/dynamixel_sdk/port_handler_linux.cpp```, add this code:
```cpp
// Enable linux FTDI low latency mode
struct serial_struct ser_info;
ioctl(socket_fd_, TIOCGSERIAL, &ser_info);
ser_info.flags |= ASYNC_LOW_LATENCY;
ioctl(socket_fd_, TIOCSSERIAL, &ser_info);
```
at line 222, just before the code that cleans the buffer and activates the settings of the port.

There is one last issue remaining. Setting the port to low latency makes reading functions take typically 0.8 to 1ms, which is what was wanted. However, in case of an unsuccessful reading, the port timeout is calculated depending on the expected packet size and the port latency, as one can read in the ```PortHandlerLinux::setPacketTimeout``` function in ```<your-path>/DynamixelSDK/c++/src/dynamixel_sdk/port_handler_linux.cpp```. <br /> 
However, the port latency is set as a hardcoded define to 16ms in that same file, which means that the timeout is going to be very long even if the port is set to low latency. To fix this, set 1 instead of 16 in the ```#define LATENCY_TIMER```.


### 3. Compile and install the SDK
Dynamixel's SDK can finally be compiled and installed by:
```bash
cd <your-path>/DynamixelSDK/c++/build/<choose-the-correct-architecture>
make
sudo make install
```
The library is installed to the default Linux installation directories, that is, ```/usr/local/lib``` for the (dynamic) library, and ```/usr/local/include/dynamixel_sdk/``` for its include files.

## Installation of the rest of dependencies
The rest of the dependencies can be installed with:
```bash
sudo pacman -S cmake doxygen graphviz
```

## Installation of KMR_dxl
```bash
cd <your-path>
git clone https://github.com/KM-RoBoTa/KMR_dxl
cd KMR_dxl
mkdir build && cd build
cmake ../
make
```
This will generate the static library ```libKMR_dxl.a``` in the "build" folder, as well as several executable examples. If doxygen and Graphviz are installed, it will also generate local documentation in ```KMR_dxl/docs/generated_docs/html/index.html```.

## Include in a project
It is very straightforward to include this library in a project. <br /> 
All the headers are located in "KMR_dxl/include", and, as already mentioned, the static libraries in "KMR_dxl/build".

In the root CMakeLists.txt of the project, add:
```cmake
# Path to KMR_dxl's CMakeLists
add_subdirectory(path-to-KMR_dxl)

# Path to KMR_dxl's header files
target_include_directories(project PUBLIC path-to-KMR_dxl/include)

# Path to the static (.a) library
target_link_directories(project PRIVATE path-to-KMR_dxl/build)

# Link the library
target_link_libraries(project KMR_dxl)

```

In the source code, only one header needs to be included:
```cpp
#include "KMR_dxl.hpp"
```

## Add a motor model
To add a motor model to the library, follow those steps:
1. Create a new configuration header file for the new model in ```KMR_dxl/config/motor_models```, inspired from the already existing models. Make sure to convert the units from Dynamixel's control tables to SI units.
2. In ```KMR_dxl/config/KMR_dxl_motor_models.hpp```, include the new header file and add the model number of the new model
3. In ```KMR_dxl/include/KMR_dxl_hal.hpp```, add a new private pointer to ```ControlTable```, preferably bearing the name of the new model. 
4. In ```KMR_dxl/include/KMR_dxl_hal.cpp```, there are 3 necessary edits:
    - In the constructor, initialize the newly created pointer from step 3 to the new motor structure you created in step 1
    - In the destructor, delete that same pointer and set it to ```nullptr``` after
    - In the method ```Hal::getControlTable```, add the case for the new model


Next: how to [use](use_git.md)