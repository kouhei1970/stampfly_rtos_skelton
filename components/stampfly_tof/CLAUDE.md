# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

VL53L3CX Time-of-Flight (ToF) distance sensor driver for StampFly (M5Stamp S3 quadcopter). This is an ESP-IDF component optimized for dual ToF sensors (front and bottom) with interrupt-based measurement and 1D Kalman filtering.

**Target Hardware**: ESP32-S3 (M5Stamp S3), VL53L3CX ToF sensors
**Framework**: ESP-IDF (CMake-based build system)
**Language**: C (embedded)

## Common Commands

### Building & Flashing Examples

Examples are standalone ESP-IDF projects. Navigate to an example directory first:

```bash
# Navigate to an example
cd examples/basic_polling  # or basic_interrupt

# Set target (first time only)
idf.py set-target esp32s3

# Build
idf.py build

# Flash and monitor
idf.py flash monitor

# Monitor only
idf.py monitor

# Clean build
idf.py fullclean
```

### Configuration

```bash
# Open menuconfig
idf.py menuconfig

# Navigate to: Component config → StampFly ToF Sensor Configuration
```

## Architecture Overview

### Project Structure

```
stampfly_tof/                  # Root ESP-IDF component
├── include/
│   ├── stampfly_tof_config.h  # Kconfig-generated hardware config
│   ├── vl53lx_outlier_filter.h # 1D Kalman filter API
│   └── vl53lx/                # VL53LX driver headers (20+ files)
├── src/
│   ├── vl53lx_platform.c      # !! CRITICAL: ESP-IDF I2C abstraction layer
│   ├── vl53lx_platform_ipp.c  # IPP (integer processing) layer
│   ├── vl53lx_outlier_filter.c # !! Custom 1D Kalman filter
│   └── vl53lx/                # VL53LX driver core (ST Microelectronics)
└── examples/
    ├── basic_polling/         # ⭐ Simple polling example (for beginners)
    ├── basic_interrupt/       # ⭐ Simple interrupt example (recommended)
    └── development/           # Detailed learning samples (stage1-8)
```

### Key Architectural Layers

1. **VL53LX Core Driver** ([src/vl53lx/](src/vl53lx/))
   - ST Microelectronics BareDriver 1.2.14
   - Device initialization, calibration, ranging APIs
   - DO NOT modify these files - they are vendor code

2. **Platform Abstraction Layer** ([src/vl53lx_platform.c](src/vl53lx_platform.c))
   - Wraps ESP-IDF I2C master API for VL53LX driver
   - Implements `VL53LX_ReadMulti()`, `VL53LX_WriteMulti()`, etc.
   - Uses ESP-IDF's `i2c_master_transmit/receive` APIs
   - IMPORTANT: Each `VL53LX_Dev_t` stores an `i2c_master_dev_handle_t` in `I2cHandle` field

3. **Outlier Filter** ([src/vl53lx_outlier_filter.c](src/vl53lx_outlier_filter.c))
   - 1D Kalman filter with prediction-only mode
   - Rejects outliers based on range status and rate-of-change
   - Default parameters: Q=1.0 (process noise), R=4.0 (measurement noise)
   - Can operate in prediction-only mode when measurements are invalid

### Hardware Configuration

**Two ToF Sensors:**
- **Bottom ToF**: GPIO7 (XSHUT), GPIO6 (INT), I2C addr 0x30 - **USB powered**
- **Front ToF**: GPIO9 (XSHUT), GPIO8 (INT), I2C addr 0x29 - **Requires battery**

**I2C Bus:**
- SDA: GPIO3, SCL: GPIO4, Frequency: 400kHz
- Both sensors share the same I2C bus but use different addresses

**Power Consideration:**
- Bottom sensor works with USB power alone (default for testing)
- Front sensor requires battery connection (partial I2C response without battery)

### Multi-Sensor Strategy

To use both sensors simultaneously:

1. Both sensors shutdown via XSHUT pins
2. Enable bottom sensor at default address (0x29)
3. Change bottom sensor address to 0x30 via I2C
4. Enable front sensor (stays at 0x29)

This is implemented in [examples/development/stage6_dual_sensor](examples/development/stage6_dual_sensor/) via XSHUT pin sequencing.

## Example Programs

### For Beginners (Start Here)

- [examples/basic_polling](examples/basic_polling/) - Simple polling measurement (easiest to understand)
- [examples/basic_interrupt](examples/basic_interrupt/) - Simple interrupt measurement (recommended, more efficient)

**Usage Pattern:**
1. Initialize I2C bus
2. Power on sensor (XSHUT pin)
3. Initialize VL53LX device
4. Configure measurement (distance mode, timing budget)
5. Start measurement
6. Read data (polling or interrupt)

### For Developers (Advanced Features)

- [examples/development/stage1-8](examples/development/) - Progressive learning samples
  - Stage 1: I2C bus scan
  - Stage 2: Register read/write
  - Stage 3: Device initialization
  - Stage 4: Polling measurement (detailed)
  - Stage 5: Interrupt measurement (detailed)
  - Stage 6: Dual sensor operation
  - Stage 7: Teleplot real-time visualization
  - Stage 8: Kalman filtered streaming (production-ready)

## Kalman Filter Implementation

The 1D Kalman filter ([src/vl53lx_outlier_filter.c](src/vl53lx_outlier_filter.c)) is critical for production use:

**Features:**
- Prediction-only mode when observations are invalid (status error, rapid change)
- Range status validation (status 0 = valid)
- Rate-of-change limiter (default: 500mm/sample max)
- Stationary model (distance only, no velocity estimation)

**Usage Pattern:**
```c
vl53lx_filter_t filter;
VL53LX_FilterInit(&filter);  // Q=1.0, R=4.0

uint16_t filtered_distance;
VL53LX_FilterUpdate(&filter, raw_distance, range_status, &filtered_distance);
```

**Tuning:**
- Increase Q for faster response (noisier)
- Decrease Q for smoother output (slower response)
- R represents sensor noise (~2mm std dev for VL53L3CX)

## Critical Implementation Details

### I2C Device Handle Management

The `VL53LX_Dev_t` structure stores an ESP-IDF `i2c_master_dev_handle_t`:
```c
VL53LX_Dev_t dev;
dev.I2cDevAddr = 0x29;  // 7-bit address
dev.I2cHandle = i2c_device_handle;  // ESP-IDF handle
```

The platform layer uses `dev.I2cHandle` for all I2C operations.

### Interrupt Handling Pattern

Examples use FreeRTOS semaphores triggered by GPIO ISRs:
```c
// ISR gives semaphore
static void IRAM_ATTR isr_handler(void* arg) {
    xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);
}

// Task waits for semaphore, then reads data
xSemaphoreTake(semaphore, timeout);
VL53LX_GetMultiRangingData(&dev, &data);
VL53LX_ClearInterruptAndStartMeasurement(&dev);
```

### Timing Budget

Default: 33ms (approximately 30Hz measurement rate)
- Configurable via Kconfig or `VL53LX_SetMeasurementTimingBudgetMicroSeconds()`
- Range: 8-500ms
- Shorter = faster but less accurate
- Longer = more accurate but slower

## Development Workflow

1. **Start with basic examples**: Modify [basic_polling](examples/basic_polling/) or [basic_interrupt](examples/basic_interrupt/) for most use cases
2. **Hardware testing order**: Test bottom sensor first (USB only), then front sensor (with battery)
3. **Platform layer changes**: Only modify [src/vl53lx_platform.c](src/vl53lx_platform.c) if changing I2C implementation
4. **Filter tuning**: Adjust Q/R parameters in [src/vl53lx_outlier_filter.c](src/vl53lx_outlier_filter.c) for your application

## Configuration via Kconfig

All hardware pins and I2C settings are configurable via `idf.py menuconfig`:
- Component config → StampFly ToF Sensor Configuration
- Defaults are optimized for M5Stamp S3 StampFly hardware

## Important Files

When modifying code, these are the key files:

- [src/vl53lx_platform.c](src/vl53lx_platform.c) - I2C abstraction layer
- [src/vl53lx_outlier_filter.c](src/vl53lx_outlier_filter.c) - Kalman filter
- [include/stampfly_tof_config.h](include/stampfly_tof_config.h) - Auto-generated from Kconfig
- [Kconfig](Kconfig) - Component configuration options
- [examples/basic_polling/main/main.c](examples/basic_polling/main/main.c) - Simple reference
- [examples/basic_interrupt/main/main.c](examples/basic_interrupt/main/main.c) - Efficient reference
- [examples/development/stage8_filtered_streaming/main/main.c](examples/development/stage8_filtered_streaming/main/main.c) - Production reference

## Debugging Tips

**I2C Communication Issues:**
- Check with [stage1_i2c_scan](examples/development/stage1_i2c_scan/) first
- Verify XSHUT pin states (sensors must be powered on)
- Front sensor requires battery power

**Measurement Failures:**
- Check range status (0 = valid, 4 = out of range, etc.)
- Verify timing budget is appropriate
- Ensure `ClearInterruptAndStartMeasurement()` is called after each read

**Filter Instability:**
- Check Q/R tuning (see Kalman filter section above)
- Verify rate limiter settings for your application
- Consider disabling rate limiter for highly dynamic scenarios

## Testing Strategy

1. Start with [basic_polling](examples/basic_polling/) - simplest, easiest to debug
2. Move to [basic_interrupt](examples/basic_interrupt/) - more efficient
3. Use [stage7](examples/development/stage7_teleplot_streaming/) for visualization
4. Apply [stage8](examples/development/stage8_filtered_streaming/) for production (Kalman filter)

## Common Patterns

### Single Sensor (Bottom, USB Powered)

```c
// Power on bottom sensor only
gpio_set_level(STAMPFLY_TOF_FRONT_XSHUT, 0);   // Front: OFF
gpio_set_level(STAMPFLY_TOF_BOTTOM_XSHUT, 1);  // Bottom: ON

// Use default I2C address
dev.I2cDevAddr = 0x29;
```

### Dual Sensors (Requires Battery)

See [examples/development/stage6_dual_sensor](examples/development/stage6_dual_sensor/) for complete implementation.

## License

Original code: MIT
VL53L3CX BareDriver: GPL-2.0+ OR BSD-3-Clause (ST Microelectronics)
