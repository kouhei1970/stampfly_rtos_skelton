# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP-IDF component library for the PixArt PMW3901MB-TXQT optical flow sensor, specifically designed for the StampFly ESP32-S3 drone platform. The driver provides SPI communication, motion detection, velocity calculation, and frame capture capabilities.

## Build and Development Commands

### Setup ESP-IDF Environment
```bash
. $HOME/esp/esp-idf/export.sh
```

### Build and Flash Example (C)
```bash
cd examples/basic
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Build and Flash Example (C++)
```bash
cd examples/basic_cpp
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Monitor Only
```bash
idf.py -p /dev/ttyUSB0 monitor
```

## Component Integration

### Using as ESP-IDF Component

To integrate this driver into another ESP-IDF project:

1. Clone or copy the repository to `your_project/components/pmw3901/`:

```bash
cd your_project/components
git clone https://github.com/YOUR_USERNAME/stampfly_opticalflow.git pmw3901
# or as submodule
git submodule add https://github.com/YOUR_USERNAME/stampfly_opticalflow.git components/pmw3901
```

2. Add dependency in `main/CMakeLists.txt`:

**For C projects:**
```cmake
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES stampfly_opticalflow)
```

**For C++ projects:**
```cmake
idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES stampfly_opticalflow)
```

3. For C++ projects, ensure exception support is enabled in `sdkconfig.defaults`:
```
CONFIG_COMPILER_CXX_EXCEPTIONS=y
CONFIG_COMPILER_CXX_RTTI=y
```

**Important**:
- Directory name: `components/pmw3901/` (recommended for StampFly project)
- Component name: `stampfly_opticalflow` (as defined in CMakeLists.txt via `idf_component_register()`)
- Use the component name `stampfly_opticalflow` in REQUIRES, not the directory name

## Architecture

### Project Structure

```
stampfly_opticalflow/
├── src/                        # Source files
│   ├── pmw3901.c              # Core C driver implementation
│   └── pmw3901_wrapper.cpp    # C++ wrapper implementation
├── include/                    # Public headers
│   ├── pmw3901.h              # C API
│   ├── pmw3901_wrapper.hpp    # C++ wrapper API
│   └── pmw3901_exception.hpp  # C++ exception classes
├── docs/                       # Documentation
│   ├── PMW3901_IMPLEMENTATION_REFERENCE.md
│   ├── STAMPFLY_HARDWARE_SPEC.md
│   └── how_to_use_pwm3901.md
├── examples/                   # Example applications
│   ├── basic/                 # C language example
│   └── basic_cpp/             # C++ wrapper example
└── CMakeLists.txt             # ESP-IDF component definition
```

### Component Structure

- **src/pmw3901.c** + **include/pmw3901.h**: Core C driver implementation
  - SPI communication layer (2MHz, MODE3)
  - Register initialization sequence (8-step process per official PMW3901 manual)
  - Motion data reading (standard and burst modes)
  - Velocity calculation functions
  - Frame capture support (35×35 pixels)

- **src/pmw3901_wrapper.cpp** + **include/pmw3901_wrapper.hpp**: C++ wrapper (optional)
  - Modern C++ interface with RAII support
  - Exception-based error handling
  - Type-safe structs (MotionData, MotionBurst, Velocity)
  - Move semantics (copy disabled)
  - Namespace: `stampfly::`

- **include/pmw3901_exception.hpp**: C++ exception classes
  - `stampfly::PMW3901Exception` with error codes
  - Wraps ESP-IDF error codes

- **examples/basic/**: C language reference implementation
- **examples/basic_cpp/**: C++ wrapper reference implementation

### Hardware Configuration (StampFly)

**Critical**: The StampFly uses specific GPIO pins for PMW3901:
- MISO: GPIO 43
- MOSI: GPIO 14
- SCLK: GPIO 44
- CS: GPIO 12 (NOT GPIO 46 - that's for BMI270 IMU)

These are set via `pmw3901_get_default_config()` and should not be changed unless using different hardware.

### Velocity Estimation

**Note:** Velocity calculation functions have been removed from this driver.
For proper velocity estimation, use `ESKF::updateFlowRaw()` which provides:

- **Physically correct calculation**: Converts pixel displacement to angular velocity using sensor FOV
- **Gyro compensation**: Removes rotation component to extract pure translation
- **Camera-to-body transformation**: Handles axis mapping between sensor and aircraft frame
- **Kalman filter integration**: Fuses with other sensor data for robust estimation

See: `components/stampfly_eskf/include/eskf.hpp`

```cpp
// Example: ESKF receives raw flow data
eskf.updateFlowRaw(flow_dx, flow_dy, distance, dt, gyro_x, gyro_y);
```

### Data Quality Filtering

The driver checks data quality via:
- SQUAL (Surface Quality): Should be > 0x19
- Shutter_Upper: Should not be 0x1F (indicates saturation)

Burst read mode returns all quality metrics in one SPI transaction for efficiency.

## Teleplot Integration

The example outputs real-time data in Teleplot format (`>variable:value`):
- delta_x, delta_y (pixel displacement)
- velocity_x, velocity_y (m/s)
- squal, shutter, raw_sum (quality metrics)

View graphs at https://teleplot.fr/ or via VSCode Teleplot extension.

## Key Implementation Details

### Initialization Sequence

The driver follows the official PMW3901 8-step initialization:
1. Power-up delay (45ms)
2. CS pin toggle sequence
3. SPI reset
4. Power-up reset (0x3A register)
5. Performance optimization register writes (60+ registers across multiple banks)
6. C1/C2 calibration
7. Product ID verification (0x49)
8. Inverse Product ID verification (0xB6)

### SPI Timing Requirements

- Clock speed: 2MHz maximum
- Post-read delay: 50μs minimum
- Post-write delay: 50μs minimum
- These delays are critical for reliable communication

### Burst Read vs Standard Read

- **Standard Read**: Freeze motion → read 4 registers (DELTA_X_L/H, DELTA_Y_L/H) → 4 SPI transactions
- **Burst Read**: Single 0x16 command → 13 bytes in one transaction → includes motion, delta_x, delta_y, squal, shutter, and other quality data
- Burst read is more efficient and recommended for high-frequency sampling (100Hz+)

## Documentation Reference

- [docs/PMW3901_IMPLEMENTATION_REFERENCE.md](docs/PMW3901_IMPLEMENTATION_REFERENCE.md): Deep dive into velocity calculation methods, data quality filtering, and comparison with PX4/Bitcraze implementations
- [docs/STAMPFLY_HARDWARE_SPEC.md](docs/STAMPFLY_HARDWARE_SPEC.md): Complete StampFly hardware specifications including all sensors and pin assignments
- [docs/how_to_use_pwm3901.md](docs/how_to_use_pwm3901.md): Official PMW3901 manual implementation reference

## API Usage

### C API (Traditional)

```c
#include "pmw3901.h"  // include/pmw3901.h

pmw3901_t dev;
pmw3901_config_t config;
pmw3901_get_default_config(&config);

if (pmw3901_init(&dev, &config) == ESP_OK) {
    int16_t delta_x, delta_y;
    pmw3901_read_motion(&dev, &delta_x, &delta_y);
    pmw3901_deinit(&dev);  // Manual cleanup
}
```

### C++ API (Recommended for C++ projects)

```cpp
#include "pmw3901_wrapper.hpp"  // include/pmw3901_wrapper.hpp

try {
    stampfly::PMW3901 sensor;  // RAII - auto init

    auto motion = sensor.readMotion();
    auto burst = sensor.readMotionBurst();

    // Use ESKF for velocity estimation (not this driver)
    // eskf.updateFlowRaw(burst.delta_x, burst.delta_y, distance, dt, gyro_x, gyro_y);

    // Auto cleanup on scope exit
} catch (const stampfly::PMW3901Exception& e) {
    ESP_LOGE(TAG, "Error: %s", e.what());
}
```

**Key C++ advantages:**
- RAII: Automatic resource management, no need to call deinit()
- Exception handling: Clear error propagation with try-catch
- Type safety: Structured return types (MotionData, MotionBurst)
- Move semantics: Efficient resource transfer

## ESP-IDF Version

This driver is developed and tested with **ESP-IDF v5.4.1**. It uses:
- `driver/spi_master.h` for SPI communication
- `driver/gpio.h` for CS pin control
- `rom/ets_sys.h` for microsecond delays (`ets_delay_us()`)
