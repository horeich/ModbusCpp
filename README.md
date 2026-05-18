# Modbus Library Documentation

Welcome to the Modbus Library repository! This library provides support for Modbus communication, allowing for seamless integration and interaction with Modbus-enabled devices.

## Repository Structure

The repository is structured as follows:

```
Modbus/
├── src/
│   ├── modbusRTUMaster.cpp
│   ├── modbusRTUSlave.cpp
│   └── modbusTCPClient.cpp
├── include/
│   ├── modbusRTUMaster.hpp
│   ├── modbusRTUSlave.hpp
│   ├── modbusTCPClient.hpp
├── .gitignore
├── LICENSE
├── README.md
├── mbed_app.json
└── platformio.ini
```

### Key Directories and Files
- **src/**: Contains the source code for the Modbus library.
- **include/**: Contains the header files for the Modbus library.

## Getting Started

To use the Modbus library in your project, follow these steps:

First, let's look at the steps to clone the necessary repositories and update the `platformio.ini` file.

### Step 1: Clone the Dependency Repositories

1. **Clone the Modbus Library Repository**:
   ```bash
   git clone https://github.com/HoreichFawad/Modbus.git
   cd Modbus
   ```

2. **Clone the Dependency Repositories**:
   ```bash
   # Example of cloning a dependency
   git clone https://github.com/HoreichFawad/W5500.git
   git clone https://github.com/HoreichFawad/Dependency2.git
   # Add more as necessary
   ```

   Ensure you clone these repositories in the same directory as the `Modbus` repository or in a location where you can easily reference them from your `platformio.ini` file.

### Step 2: Update `platformio.ini`

Open the `platformio.ini` file located in the `Modbus` repository and update the paths for the dependencies.

Here's an example `platformio.ini` file after updating the library paths:

```ini
[env:your_environment]
platform = your_platform
board = your_board
framework = your_framework

lib_deps =
    ${sysenv.HOME}/path_to_dependencies/Dependency1
    ${sysenv.HOME}/path_to_dependencies/Dependency2
    # Add paths for other dependencies

build_flags = 
    -I include
```

### Example `platformio.ini` File

Assuming the `Dependency1` and `Dependency2` repositories are cloned into the same directory as the `Modbus` repository, your `platformio.ini` might look something like this:

```ini
[env:your_environment]
platform = your_platform
board = your_board
framework = your_framework

lib_deps =
    ../Dependency1
    ../Dependency2

build_flags = 
    -I include
```

Replace `your_environment`, `your_platform`, `your_board`, and `your_framework` with the appropriate values for your setup.


### Example Structure

A cpp file may have a structure similar to the following:

```c
// Define the example you want to run
#define RTUSlave
//#define RTUMaster
//#define TCPClient

#ifdef RTUSlave
// RTU Slave code here
#endif

#ifdef RTUMaster
// RTU Master code here
#endif

#ifdef TCPClient
// TCP Client code here
#endif
```

### Running the Project

After updating the `platformio.ini` file, you can build and upload the project using PlatformIO commands:

```bash
pio run -t upload
```
Make sure to comment out or remove other `#define` directives in the same file to avoid conflicts.

---