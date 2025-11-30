#!/usr/bin/env python3
"""
visualize_device_log.py - StampFly Device Log Visualization Tool

Visualizes binary log data captured from StampFly device.
Supports both V1 (sensor only) and V2 (sensor + ESKF) packet formats.

Usage:
    python visualize_device_log.py sensor_log.bin
    python visualize_device_log.py eskf_log.bin --output eskf_analysis.png
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Import from log_capture
from log_capture import (
    parse_log_file, detect_file_version,
    PACKET_SIZE_V1, PACKET_SIZE_V2
)


def visualize_v1_log(packets, output_file: str = None, show: bool = True):
    """Visualize V1 log (sensor data only)"""
    if not packets:
        print("No packets to visualize")
        return

    # Extract data
    timestamps = np.array([p.timestamp_ms / 1000.0 for p in packets])  # seconds
    t0 = timestamps[0]
    t = timestamps - t0

    accel_x = np.array([p.accel_x for p in packets])
    accel_y = np.array([p.accel_y for p in packets])
    accel_z = np.array([p.accel_z for p in packets])

    gyro_x = np.array([p.gyro_x for p in packets])
    gyro_y = np.array([p.gyro_y for p in packets])
    gyro_z = np.array([p.gyro_z for p in packets])

    mag_x = np.array([p.mag_x for p in packets])
    mag_y = np.array([p.mag_y for p in packets])
    mag_z = np.array([p.mag_z for p in packets])

    baro_alt = np.array([p.baro_alt for p in packets])
    tof_bottom = np.array([p.tof_bottom for p in packets])

    flow_dx = np.array([p.flow_dx for p in packets])
    flow_dy = np.array([p.flow_dy for p in packets])
    flow_squal = np.array([p.flow_squal for p in packets])

    # Create figure
    fig, axes = plt.subplots(4, 2, figsize=(14, 12))
    fig.suptitle(f'StampFly Sensor Log (V1) - {len(packets)} packets, {t[-1]:.1f}s', fontsize=14)

    # Accelerometer
    ax = axes[0, 0]
    ax.plot(t, accel_x, 'r-', label='X', alpha=0.8)
    ax.plot(t, accel_y, 'g-', label='Y', alpha=0.8)
    ax.plot(t, accel_z, 'b-', label='Z', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Acceleration [m/s²]')
    ax.set_title('Accelerometer')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Gyroscope
    ax = axes[0, 1]
    ax.plot(t, np.degrees(gyro_x), 'r-', label='X', alpha=0.8)
    ax.plot(t, np.degrees(gyro_y), 'g-', label='Y', alpha=0.8)
    ax.plot(t, np.degrees(gyro_z), 'b-', label='Z', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angular Rate [°/s]')
    ax.set_title('Gyroscope')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Magnetometer
    ax = axes[1, 0]
    ax.plot(t, mag_x, 'r-', label='X', alpha=0.8)
    ax.plot(t, mag_y, 'g-', label='Y', alpha=0.8)
    ax.plot(t, mag_z, 'b-', label='Z', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Magnetic Field [uT]')
    ax.set_title('Magnetometer')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Altitude sensors
    ax = axes[1, 1]
    ax.plot(t, baro_alt, 'b-', label='Baro', alpha=0.8)
    ax.plot(t, tof_bottom, 'r-', label='ToF', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Altitude [m]')
    ax.set_title('Altitude Sensors')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Optical Flow delta
    ax = axes[2, 0]
    ax.plot(t, flow_dx, 'r-', label='dx', alpha=0.8)
    ax.plot(t, flow_dy, 'g-', label='dy', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Flow Delta [counts]')
    ax.set_title('Optical Flow Delta')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Flow quality
    ax = axes[2, 1]
    ax.plot(t, flow_squal, 'b-', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Surface Quality')
    ax.set_title('Flow Surface Quality')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 255)

    # Sample rate analysis
    ax = axes[3, 0]
    dt = np.diff(timestamps) * 1000  # ms
    ax.hist(dt, bins=50, color='blue', alpha=0.7, edgecolor='black')
    ax.axvline(np.mean(dt), color='r', linestyle='--', label=f'Mean: {np.mean(dt):.2f}ms')
    ax.axvline(10, color='g', linestyle='--', label='Target: 10ms')
    ax.set_xlabel('Sample Period [ms]')
    ax.set_ylabel('Count')
    ax.set_title('Sample Rate Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Statistics text
    ax = axes[3, 1]
    ax.axis('off')
    stats_text = f"""
    === Log Statistics ===

    Duration: {t[-1]:.2f} seconds
    Packets: {len(packets)}
    Sample Rate: {len(packets) / t[-1]:.1f} Hz

    === Accelerometer ===
    X: {accel_x.mean():.3f} ± {accel_x.std():.3f} m/s²
    Y: {accel_y.mean():.3f} ± {accel_y.std():.3f} m/s²
    Z: {accel_z.mean():.3f} ± {accel_z.std():.3f} m/s²

    === Gyroscope ===
    X: {np.degrees(gyro_x.mean()):.3f} ± {np.degrees(gyro_x.std()):.3f} °/s
    Y: {np.degrees(gyro_y.mean()):.3f} ± {np.degrees(gyro_y.std()):.3f} °/s
    Z: {np.degrees(gyro_z.mean()):.3f} ± {np.degrees(gyro_z.std()):.3f} °/s

    === Altitude ===
    Baro: {baro_alt.mean():.3f} ± {baro_alt.std():.3f} m
    ToF: {tof_bottom.mean():.3f} ± {tof_bottom.std():.3f} m
    """
    ax.text(0.1, 0.5, stats_text, transform=ax.transAxes, fontsize=9,
            verticalalignment='center', fontfamily='monospace')

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved visualization to {output_file}")

    if show:
        plt.show()


def visualize_v2_log(packets, output_file: str = None, show: bool = True):
    """Visualize V2 log (sensor data + ESKF estimates)"""
    if not packets:
        print("No packets to visualize")
        return

    # Extract data
    timestamps = np.array([p.timestamp_ms / 1000.0 for p in packets])  # seconds
    t0 = timestamps[0]
    t = timestamps - t0

    # Sensor data
    accel_x = np.array([p.accel_x for p in packets])
    accel_y = np.array([p.accel_y for p in packets])
    accel_z = np.array([p.accel_z for p in packets])

    gyro_x = np.array([p.gyro_x for p in packets])
    gyro_y = np.array([p.gyro_y for p in packets])
    gyro_z = np.array([p.gyro_z for p in packets])

    baro_alt = np.array([p.baro_alt for p in packets])
    tof_bottom = np.array([p.tof_bottom for p in packets])

    flow_dx = np.array([p.flow_dx for p in packets])
    flow_dy = np.array([p.flow_dy for p in packets])

    # ESKF estimates
    pos_x = np.array([p.pos_x for p in packets])
    pos_y = np.array([p.pos_y for p in packets])
    pos_z = np.array([p.pos_z for p in packets])

    vel_x = np.array([p.vel_x for p in packets])
    vel_y = np.array([p.vel_y for p in packets])
    vel_z = np.array([p.vel_z for p in packets])

    roll = np.array([np.degrees(p.roll) for p in packets])
    pitch = np.array([np.degrees(p.pitch) for p in packets])
    yaw = np.array([np.degrees(p.yaw) for p in packets])

    gyro_bias_z = np.array([np.degrees(p.gyro_bias_z) for p in packets])
    accel_bias_x = np.array([p.accel_bias_x for p in packets])
    accel_bias_y = np.array([p.accel_bias_y for p in packets])

    # Create figure with 3 columns
    fig, axes = plt.subplots(4, 3, figsize=(16, 14))
    fig.suptitle(f'StampFly ESKF Log (V2) - {len(packets)} packets, {t[-1]:.1f}s', fontsize=14)

    # Row 0: IMU
    ax = axes[0, 0]
    ax.plot(t, accel_x, 'r-', label='X', alpha=0.8)
    ax.plot(t, accel_y, 'g-', label='Y', alpha=0.8)
    ax.plot(t, accel_z, 'b-', label='Z', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Acceleration [m/s²]')
    ax.set_title('Accelerometer')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, np.degrees(gyro_x), 'r-', label='X', alpha=0.8)
    ax.plot(t, np.degrees(gyro_y), 'g-', label='Y', alpha=0.8)
    ax.plot(t, np.degrees(gyro_z), 'b-', label='Z', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angular Rate [°/s]')
    ax.set_title('Gyroscope')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 2]
    ax.plot(t, baro_alt, 'b-', label='Baro', alpha=0.8)
    ax.plot(t, tof_bottom, 'r-', label='ToF', alpha=0.8)
    ax.plot(t, -pos_z, 'g--', label='ESKF (-pos_z)', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Altitude [m]')
    ax.set_title('Altitude (Sensor vs ESKF)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Row 1: ESKF Attitude
    ax = axes[1, 0]
    ax.plot(t, roll, 'r-', label='Roll', alpha=0.8)
    ax.plot(t, pitch, 'g-', label='Pitch', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angle [°]')
    ax.set_title('ESKF Roll/Pitch')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, yaw, 'b-', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Yaw [°]')
    ax.set_title('ESKF Yaw')
    ax.grid(True, alpha=0.3)

    ax = axes[1, 2]
    ax.plot(t, gyro_bias_z, 'b-', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Bias [°/s]')
    ax.set_title('Gyro Bias Z (Yaw)')
    ax.grid(True, alpha=0.3)

    # Row 2: ESKF Position/Velocity
    ax = axes[2, 0]
    ax.plot(t, pos_x, 'r-', label='X', alpha=0.8)
    ax.plot(t, pos_y, 'g-', label='Y', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Position [m]')
    ax.set_title('ESKF Position (NED X/Y)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    ax.plot(t, vel_x, 'r-', label='Vx', alpha=0.8)
    ax.plot(t, vel_y, 'g-', label='Vy', alpha=0.8)
    ax.plot(t, vel_z, 'b-', label='Vz', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Velocity [m/s]')
    ax.set_title('ESKF Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2, 2]
    ax.plot(pos_x, pos_y, 'b-', alpha=0.8)
    ax.plot(pos_x[0], pos_y[0], 'go', markersize=10, label='Start')
    ax.plot(pos_x[-1], pos_y[-1], 'rx', markersize=10, label='End')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('ESKF 2D Trajectory (NED)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')

    # Row 3: Flow and Biases
    ax = axes[3, 0]
    ax.plot(t, flow_dx, 'r-', label='dx', alpha=0.8)
    ax.plot(t, flow_dy, 'g-', label='dy', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Flow Delta [counts]')
    ax.set_title('Optical Flow')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[3, 1]
    ax.plot(t, accel_bias_x, 'r-', label='Bias X', alpha=0.8)
    ax.plot(t, accel_bias_y, 'g-', label='Bias Y', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Bias [m/s²]')
    ax.set_title('Accel Bias X/Y')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Statistics text
    ax = axes[3, 2]
    ax.axis('off')

    # Calculate drift
    pos_drift = np.sqrt((pos_x[-1] - pos_x[0])**2 + (pos_y[-1] - pos_y[0])**2)
    yaw_drift = yaw[-1] - yaw[0]

    stats_text = f"""
    === ESKF Statistics ===

    Duration: {t[-1]:.2f} seconds
    Sample Rate: {len(packets) / t[-1]:.1f} Hz

    === Attitude ===
    Roll:  {roll.mean():.2f} ± {roll.std():.2f} °
    Pitch: {pitch.mean():.2f} ± {pitch.std():.2f} °
    Yaw:   {yaw[0]:.1f}° → {yaw[-1]:.1f}° (drift: {yaw_drift:.1f}°)

    === Position Drift ===
    X: {pos_x[0]:.3f}m → {pos_x[-1]:.3f}m
    Y: {pos_y[0]:.3f}m → {pos_y[-1]:.3f}m
    Total drift: {pos_drift:.3f}m

    === Biases (final) ===
    Gyro Z: {gyro_bias_z[-1]:.3f} °/s
    Accel X: {accel_bias_x[-1]:.4f} m/s²
    Accel Y: {accel_bias_y[-1]:.4f} m/s²
    """
    ax.text(0.05, 0.5, stats_text, transform=ax.transAxes, fontsize=9,
            verticalalignment='center', fontfamily='monospace')

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved visualization to {output_file}")

    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='StampFly Device Log Visualization')
    parser.add_argument('input', help='Input binary log file (.bin)')
    parser.add_argument('--output', '-o', help='Output image file (.png)')
    parser.add_argument('--no-show', action='store_true', help='Do not show plot window')

    args = parser.parse_args()

    # Check input file exists
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: Input file not found: {args.input}")
        sys.exit(1)

    # Detect version
    version = detect_file_version(args.input)
    print(f"Detected packet version: V{version}")
    print(f"Packet size: {PACKET_SIZE_V2 if version == 2 else PACKET_SIZE_V1} bytes")

    # Parse log file
    packets = parse_log_file(args.input, version)

    if not packets:
        print("No valid packets found")
        sys.exit(1)

    print(f"Loaded {len(packets)} packets")

    # Generate output filename if not specified
    output_file = args.output
    if not output_file:
        output_file = input_path.stem + '_analysis.png'

    # Visualize based on version
    if version == 1:
        visualize_v1_log(packets, output_file, show=not args.no_show)
    else:
        visualize_v2_log(packets, output_file, show=not args.no_show)


if __name__ == '__main__':
    main()
