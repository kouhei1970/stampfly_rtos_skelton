#!/usr/bin/env python3
"""
visualize_eskf.py - ESKF Output Visualization Tool

Plots ESKF state estimates and compares with raw sensor data.

Usage:
    python visualize_eskf.py --eskf output_states.csv [--log sensor_log.bin] [--output plots/]
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import struct

# Import log_capture for binary parsing
from log_capture import parse_log_file, BinaryLogPacket


def load_eskf_csv(filepath: str) -> pd.DataFrame:
    """Load ESKF output CSV file"""
    df = pd.read_csv(filepath)
    # Convert timestamp to seconds
    df['time_s'] = df['timestamp_ms'] / 1000.0
    return df


def load_raw_log(filepath: str) -> pd.DataFrame:
    """Load raw binary log and convert to DataFrame"""
    packets = parse_log_file(filepath)

    data = {
        'timestamp_ms': [],
        'accel_x': [], 'accel_y': [], 'accel_z': [],
        'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
        'mag_x': [], 'mag_y': [], 'mag_z': [],
        'pressure': [], 'baro_alt': [],
        'tof_bottom': [], 'tof_front': [],
        'flow_dx': [], 'flow_dy': [], 'flow_squal': []
    }

    for pkt in packets:
        data['timestamp_ms'].append(pkt.timestamp_ms)
        data['accel_x'].append(pkt.accel_x)
        data['accel_y'].append(pkt.accel_y)
        data['accel_z'].append(pkt.accel_z)
        data['gyro_x'].append(pkt.gyro_x)
        data['gyro_y'].append(pkt.gyro_y)
        data['gyro_z'].append(pkt.gyro_z)
        data['mag_x'].append(pkt.mag_x)
        data['mag_y'].append(pkt.mag_y)
        data['mag_z'].append(pkt.mag_z)
        data['pressure'].append(pkt.pressure)
        data['baro_alt'].append(pkt.baro_alt)
        data['tof_bottom'].append(pkt.tof_bottom)
        data['tof_front'].append(pkt.tof_front)
        data['flow_dx'].append(pkt.flow_dx)
        data['flow_dy'].append(pkt.flow_dy)
        data['flow_squal'].append(pkt.flow_squal)

    df = pd.DataFrame(data)
    df['time_s'] = df['timestamp_ms'] / 1000.0
    return df


def plot_attitude(eskf_df: pd.DataFrame, raw_df: pd.DataFrame = None,
                  output_dir: Path = None):
    """Plot attitude estimation (roll, pitch, yaw)"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    time = eskf_df['time_s'] - eskf_df['time_s'].iloc[0]

    # Roll
    axes[0].plot(time, eskf_df['roll_deg'], 'b-', label='ESKF Roll', linewidth=1)
    if raw_df is not None:
        # Calculate roll from accelerometer
        raw_time = raw_df['time_s'] - raw_df['time_s'].iloc[0]
        roll_acc = np.degrees(np.arctan2(raw_df['accel_y'], raw_df['accel_z']))
        axes[0].plot(raw_time, roll_acc, 'r-', alpha=0.3, label='Accel Roll', linewidth=0.5)
    axes[0].set_ylabel('Roll [deg]')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)

    # Pitch
    axes[1].plot(time, eskf_df['pitch_deg'], 'g-', label='ESKF Pitch', linewidth=1)
    if raw_df is not None:
        pitch_acc = np.degrees(np.arctan2(-raw_df['accel_x'],
                                          np.sqrt(raw_df['accel_y']**2 + raw_df['accel_z']**2)))
        axes[1].plot(raw_time, pitch_acc, 'r-', alpha=0.3, label='Accel Pitch', linewidth=0.5)
    axes[1].set_ylabel('Pitch [deg]')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    # Yaw
    axes[2].plot(time, eskf_df['yaw_deg'], 'm-', label='ESKF Yaw', linewidth=1)
    if raw_df is not None:
        yaw_mag = np.degrees(np.arctan2(raw_df['mag_y'], raw_df['mag_x']))
        axes[2].plot(raw_time, yaw_mag, 'r-', alpha=0.3, label='Mag Yaw', linewidth=0.5)
    axes[2].set_ylabel('Yaw [deg]')
    axes[2].set_xlabel('Time [s]')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    fig.suptitle('Attitude Estimation')
    plt.tight_layout()

    if output_dir:
        plt.savefig(output_dir / 'attitude.png', dpi=150)
        print(f"Saved: {output_dir / 'attitude.png'}")

    return fig


def plot_position_velocity(eskf_df: pd.DataFrame, raw_df: pd.DataFrame = None,
                           output_dir: Path = None):
    """Plot position and velocity estimation"""
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))

    time = eskf_df['time_s'] - eskf_df['time_s'].iloc[0]

    # Position
    axes[0, 0].plot(time, eskf_df['pos_x'], 'b-', linewidth=1)
    axes[0, 0].set_ylabel('Position X [m]')
    axes[0, 0].set_title('Position X')
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(time, eskf_df['pos_y'], 'g-', linewidth=1)
    axes[0, 1].set_ylabel('Position Y [m]')
    axes[0, 1].set_title('Position Y')
    axes[0, 1].grid(True, alpha=0.3)

    axes[0, 2].plot(time, eskf_df['pos_z'], 'r-', label='ESKF', linewidth=1)
    if raw_df is not None:
        raw_time = raw_df['time_s'] - raw_df['time_s'].iloc[0]
        axes[0, 2].plot(raw_time, raw_df['baro_alt'], 'c-', alpha=0.5, label='Baro', linewidth=0.5)
        axes[0, 2].plot(raw_time, -raw_df['tof_bottom'], 'm-', alpha=0.5, label='-ToF', linewidth=0.5)
    axes[0, 2].set_ylabel('Position Z [m]')
    axes[0, 2].set_title('Altitude (NED: down is positive)')
    axes[0, 2].legend(loc='upper right')
    axes[0, 2].grid(True, alpha=0.3)

    # Velocity
    axes[1, 0].plot(time, eskf_df['vel_x'], 'b-', linewidth=1)
    axes[1, 0].set_ylabel('Velocity X [m/s]')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_title('Velocity X')
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(time, eskf_df['vel_y'], 'g-', linewidth=1)
    axes[1, 1].set_ylabel('Velocity Y [m/s]')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_title('Velocity Y')
    axes[1, 1].grid(True, alpha=0.3)

    axes[1, 2].plot(time, eskf_df['vel_z'], 'r-', linewidth=1)
    axes[1, 2].set_ylabel('Velocity Z [m/s]')
    axes[1, 2].set_xlabel('Time [s]')
    axes[1, 2].set_title('Velocity Z')
    axes[1, 2].grid(True, alpha=0.3)

    fig.suptitle('Position and Velocity Estimation')
    plt.tight_layout()

    if output_dir:
        plt.savefig(output_dir / 'position_velocity.png', dpi=150)
        print(f"Saved: {output_dir / 'position_velocity.png'}")

    return fig


def plot_biases(eskf_df: pd.DataFrame, output_dir: Path = None):
    """Plot estimated biases"""
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))

    time = eskf_df['time_s'] - eskf_df['time_s'].iloc[0]

    # Gyro biases
    axes[0, 0].plot(time, eskf_df['gyro_bias_x'] * 1000, 'b-', linewidth=1)
    axes[0, 0].set_ylabel('Gyro Bias X [mrad/s]')
    axes[0, 0].set_title('Gyro Bias X')
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(time, eskf_df['gyro_bias_y'] * 1000, 'g-', linewidth=1)
    axes[0, 1].set_ylabel('Gyro Bias Y [mrad/s]')
    axes[0, 1].set_title('Gyro Bias Y')
    axes[0, 1].grid(True, alpha=0.3)

    axes[0, 2].plot(time, eskf_df['gyro_bias_z'] * 1000, 'r-', linewidth=1)
    axes[0, 2].set_ylabel('Gyro Bias Z [mrad/s]')
    axes[0, 2].set_title('Gyro Bias Z')
    axes[0, 2].grid(True, alpha=0.3)

    # Accel biases
    axes[1, 0].plot(time, eskf_df['accel_bias_x'] * 1000, 'b-', linewidth=1)
    axes[1, 0].set_ylabel('Accel Bias X [mm/s²]')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_title('Accel Bias X')
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(time, eskf_df['accel_bias_y'] * 1000, 'g-', linewidth=1)
    axes[1, 1].set_ylabel('Accel Bias Y [mm/s²]')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_title('Accel Bias Y')
    axes[1, 1].grid(True, alpha=0.3)

    axes[1, 2].plot(time, eskf_df['accel_bias_z'] * 1000, 'r-', linewidth=1)
    axes[1, 2].set_ylabel('Accel Bias Z [mm/s²]')
    axes[1, 2].set_xlabel('Time [s]')
    axes[1, 2].set_title('Accel Bias Z')
    axes[1, 2].grid(True, alpha=0.3)

    fig.suptitle('Estimated Biases')
    plt.tight_layout()

    if output_dir:
        plt.savefig(output_dir / 'biases.png', dpi=150)
        print(f"Saved: {output_dir / 'biases.png'}")

    return fig


def plot_raw_sensors(raw_df: pd.DataFrame, output_dir: Path = None):
    """Plot raw sensor data"""
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))

    time = raw_df['time_s'] - raw_df['time_s'].iloc[0]

    # Accelerometer
    axes[0, 0].plot(time, raw_df['accel_x'], 'r-', alpha=0.7, label='X', linewidth=0.5)
    axes[0, 0].plot(time, raw_df['accel_y'], 'g-', alpha=0.7, label='Y', linewidth=0.5)
    axes[0, 0].plot(time, raw_df['accel_z'], 'b-', alpha=0.7, label='Z', linewidth=0.5)
    axes[0, 0].set_ylabel('Accel [m/s²]')
    axes[0, 0].set_title('Accelerometer')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # Gyroscope
    axes[0, 1].plot(time, np.degrees(raw_df['gyro_x']), 'r-', alpha=0.7, label='X', linewidth=0.5)
    axes[0, 1].plot(time, np.degrees(raw_df['gyro_y']), 'g-', alpha=0.7, label='Y', linewidth=0.5)
    axes[0, 1].plot(time, np.degrees(raw_df['gyro_z']), 'b-', alpha=0.7, label='Z', linewidth=0.5)
    axes[0, 1].set_ylabel('Gyro [deg/s]')
    axes[0, 1].set_title('Gyroscope')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # Magnetometer
    axes[1, 0].plot(time, raw_df['mag_x'], 'r-', alpha=0.7, label='X', linewidth=0.5)
    axes[1, 0].plot(time, raw_df['mag_y'], 'g-', alpha=0.7, label='Y', linewidth=0.5)
    axes[1, 0].plot(time, raw_df['mag_z'], 'b-', alpha=0.7, label='Z', linewidth=0.5)
    axes[1, 0].set_ylabel('Mag [uT]')
    axes[1, 0].set_title('Magnetometer')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Barometer / ToF
    axes[1, 1].plot(time, raw_df['baro_alt'], 'b-', alpha=0.7, label='Baro Alt', linewidth=0.5)
    axes[1, 1].plot(time, raw_df['tof_bottom'], 'r-', alpha=0.7, label='ToF Bottom', linewidth=0.5)
    axes[1, 1].set_ylabel('Altitude [m]')
    axes[1, 1].set_title('Altitude Sensors')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    # Optical Flow
    axes[2, 0].plot(time, raw_df['flow_dx'], 'r-', alpha=0.7, label='dx', linewidth=0.5)
    axes[2, 0].plot(time, raw_df['flow_dy'], 'g-', alpha=0.7, label='dy', linewidth=0.5)
    axes[2, 0].set_ylabel('Flow [counts]')
    axes[2, 0].set_xlabel('Time [s]')
    axes[2, 0].set_title('Optical Flow')
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)

    # Flow quality
    axes[2, 1].plot(time, raw_df['flow_squal'], 'b-', alpha=0.7, linewidth=0.5)
    axes[2, 1].axhline(y=30, color='r', linestyle='--', label='Min valid')
    axes[2, 1].set_ylabel('Surface Quality')
    axes[2, 1].set_xlabel('Time [s]')
    axes[2, 1].set_title('Flow Quality')
    axes[2, 1].legend()
    axes[2, 1].grid(True, alpha=0.3)

    fig.suptitle('Raw Sensor Data')
    plt.tight_layout()

    if output_dir:
        plt.savefig(output_dir / 'raw_sensors.png', dpi=150)
        print(f"Saved: {output_dir / 'raw_sensors.png'}")

    return fig


def main():
    parser = argparse.ArgumentParser(description='ESKF Output Visualization Tool')
    parser.add_argument('--eskf', '-e', required=True, help='ESKF output CSV file')
    parser.add_argument('--log', '-l', help='Raw binary log file (.bin)')
    parser.add_argument('--output', '-o', default='plots', help='Output directory for plots')
    parser.add_argument('--show', '-s', action='store_true', help='Show plots interactively')

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading ESKF output: {args.eskf}")
    eskf_df = load_eskf_csv(args.eskf)
    print(f"  {len(eskf_df)} samples, {eskf_df['time_s'].iloc[-1] - eskf_df['time_s'].iloc[0]:.2f}s duration")

    raw_df = None
    if args.log:
        print(f"Loading raw log: {args.log}")
        raw_df = load_raw_log(args.log)
        print(f"  {len(raw_df)} samples")

    print("\nGenerating plots...")

    # Generate all plots
    plot_attitude(eskf_df, raw_df, output_dir)
    plot_position_velocity(eskf_df, raw_df, output_dir)
    plot_biases(eskf_df, output_dir)

    if raw_df is not None:
        plot_raw_sensors(raw_df, output_dir)

    print(f"\nPlots saved to: {output_dir}")

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
