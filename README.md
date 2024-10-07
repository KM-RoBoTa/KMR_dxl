# C++ wrapper around Dynamixel's library for protocol 2 motors

Wrapper library for an easy use of Dynamixel motors.  
It abstracts the hardware (no need to concern yourself with memory addresses) and automates the creation of reading/writing handlers (automatically assigns indirect address reading/writing).

It works with Dynamixel's **protocol 2**.

Dependencies:
- dynamixel API
- yaml-cpp

The documentation explaining how to use the library can be found [here](docs/markdown_sources/mainpage.md).

To compile the library: go to the KMR_dxl folder, then: 
```bash
mkdir build
cd build
cmake ../
cmake --build .
```

If you have Doxygen and Graphviz installed, you can regenerate the documentation locally with
```bash
make docs
```
from the `build` folder after the cmake.  
The generated documentation can be found in "docs/generated_docs/html".

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

## About

KM-RoBoTa SA's KMR_dxl library to facilitate Dynamixel control.

### Authors
Library written by Katarina Lichardova: katarina.lichardova@km-robota.com

based on code from:
- Laura Paez: laura.paez@km-robota.com
- Kamilo Melo: kamilo.melo@km-robota.com

### Copyright
Copyright 2021-2024, Kamilo Melo. \n
This code is under MIT licence: https://opensource.org/licenses/MIT