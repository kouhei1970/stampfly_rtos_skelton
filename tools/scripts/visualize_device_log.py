#!/usr/bin/env python3
"""
visualize_device_log.py - StampFly ESKF Log Unified Visualization Tool

Visualizes binary log data captured from StampFly device and/or PC ESKF results.
Supports both V1 (sensor only) and V2 (sensor + ESKF) packet formats.

Usage:
    # Device log only (V1 or V2)
    python visualize_device_log.py sensor_log.bin
    python visualize_device_log.py sensor_log.bin --mode device

    # PC ESKF CSV only
    python visualize_device_log.py --pc eskf_output.csv --mode pc

    # Compare device (V2) and PC ESKF results
    python visualize_device_log.py sensor_log.bin --pc eskf_output.csv --mode both

    # Show magnetometer XY plot
    python visualize_device_log.py sensor_log.bin --mag-xy

    # Output options
    python visualize_device_log.py sensor_log.bin --output result.png --no-show
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Import from log_capture
from log_capture import (
    parse_log_file, detect_file_version,
    PACKET_SIZE_V1, PACKET_SIZE_V2
)


def load_pc_eskf_csv(filepath: str) -> pd.DataFrame:
    """Load PC ESKF output CSV file"""
    df = pd.read_csv(filepath)
    # Convert timestamp to seconds from start
    df['time_s'] = df['timestamp_ms'] / 1000.0
    df['time_s'] = df['time_s'] - df['time_s'].iloc[0]
    return df


def packets_to_dataframe(packets) -> pd.DataFrame:
    """Convert packet list to DataFrame"""
    if not packets:
        return pd.DataFrame()

    version = packets[0].version

    data = {
        'timestamp_ms': [p.timestamp_ms for p in packets],
        'accel_x': [p.accel_x for p in packets],
        'accel_y': [p.accel_y for p in packets],
        'accel_z': [p.accel_z for p in packets],
        'gyro_x': [p.gyro_x for p in packets],
        'gyro_y': [p.gyro_y for p in packets],
        'gyro_z': [p.gyro_z for p in packets],
        'mag_x': [p.mag_x for p in packets],
        'mag_y': [p.mag_y for p in packets],
        'mag_z': [p.mag_z for p in packets],
        'baro_alt': [p.baro_alt for p in packets],
        'tof_bottom': [p.tof_bottom for p in packets],
        'flow_dx': [p.flow_dx for p in packets],
        'flow_dy': [p.flow_dy for p in packets],
        'flow_squal': [p.flow_squal for p in packets],
    }

    if version == 2:
        data.update({
            'pos_x': [p.pos_x for p in packets],
            'pos_y': [p.pos_y for p in packets],
            'pos_z': [p.pos_z for p in packets],
            'vel_x': [p.vel_x for p in packets],
            'vel_y': [p.vel_y for p in packets],
            'vel_z': [p.vel_z for p in packets],
            'roll': [p.roll for p in packets],
            'pitch': [p.pitch for p in packets],
            'yaw': [p.yaw for p in packets],
            'gyro_bias_z': [p.gyro_bias_z for p in packets],
            'accel_bias_x': [p.accel_bias_x for p in packets],
            'accel_bias_y': [p.accel_bias_y for p in packets],
        })

    df = pd.DataFrame(data)
    df['time_s'] = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    df['version'] = version
    return df


def visualize_device_only(dev_df: pd.DataFrame, output_file: str = None, show: bool = True):
    """Visualize device log only (V1 or V2)"""
    t = dev_df['time_s'].values
    version = dev_df['version'].iloc[0] if 'version' in dev_df else 1

    if version == 1:
        # V1: Sensor data only
        fig, axes = plt.subplots(4, 2, figsize=(14, 12))
        fig.suptitle(f'StampFly Device Log (V1) - {len(dev_df)} packets, {t[-1]:.1f}s', fontsize=14)

        # Accelerometer
        ax = axes[0, 0]
        ax.plot(t, dev_df['accel_x'], 'r-', label='X', alpha=0.8)
        ax.plot(t, dev_df['accel_y'], 'g-', label='Y', alpha=0.8)
        ax.plot(t, dev_df['accel_z'], 'b-', label='Z', alpha=0.8)
        ax.set_ylabel('Acceleration [m/s²]')
        ax.set_title('Accelerometer')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Gyroscope
        ax = axes[0, 1]
        ax.plot(t, np.degrees(dev_df['gyro_x']), 'r-', label='X', alpha=0.8)
        ax.plot(t, np.degrees(dev_df['gyro_y']), 'g-', label='Y', alpha=0.8)
        ax.plot(t, np.degrees(dev_df['gyro_z']), 'b-', label='Z', alpha=0.8)
        ax.set_ylabel('Angular Rate [°/s]')
        ax.set_title('Gyroscope')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Magnetometer
        ax = axes[1, 0]
        ax.plot(t, dev_df['mag_x'], 'r-', label='X', alpha=0.8)
        ax.plot(t, dev_df['mag_y'], 'g-', label='Y', alpha=0.8)
        ax.plot(t, dev_df['mag_z'], 'b-', label='Z', alpha=0.8)
        ax.set_ylabel('Magnetic Field [uT]')
        ax.set_title('Magnetometer')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Altitude sensors
        ax = axes[1, 1]
        ax.plot(t, dev_df['baro_alt'], 'b-', label='Baro', alpha=0.8)
        ax.plot(t, dev_df['tof_bottom'], 'r-', label='ToF', alpha=0.8)
        ax.set_ylabel('Altitude [m]')
        ax.set_title('Altitude Sensors')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Optical Flow delta
        ax = axes[2, 0]
        ax.plot(t, dev_df['flow_dx'], 'r-', label='dx', alpha=0.8)
        ax.plot(t, dev_df['flow_dy'], 'g-', label='dy', alpha=0.8)
        ax.set_ylabel('Flow Delta [counts]')
        ax.set_title('Optical Flow Delta')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Flow quality
        ax = axes[2, 1]
        ax.plot(t, dev_df['flow_squal'], 'b-', alpha=0.8)
        ax.set_ylabel('Surface Quality')
        ax.set_title('Flow Surface Quality')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 255)

        # Sample rate analysis
        ax = axes[3, 0]
        dt = np.diff(dev_df['timestamp_ms'].values)
        ax.hist(dt, bins=50, color='blue', alpha=0.7, edgecolor='black')
        ax.axvline(np.mean(dt), color='r', linestyle='--', label=f'Mean: {np.mean(dt):.2f}ms')
        ax.axvline(10, color='g', linestyle='--', label='Target: 10ms')
        ax.set_xlabel('Sample Period [ms]')
        ax.set_ylabel('Count')
        ax.set_title('Sample Rate Distribution')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Statistics
        axes[3, 1].axis('off')

    else:
        # V2: Sensor + ESKF data
        fig, axes = plt.subplots(4, 3, figsize=(16, 14))
        fig.suptitle(f'StampFly Device Log (V2) - {len(dev_df)} packets, {t[-1]:.1f}s', fontsize=14)

        # Row 0: IMU
        ax = axes[0, 0]
        ax.plot(t, dev_df['accel_x'], 'r-', label='X', alpha=0.8)
        ax.plot(t, dev_df['accel_y'], 'g-', label='Y', alpha=0.8)
        ax.plot(t, dev_df['accel_z'], 'b-', label='Z', alpha=0.8)
        ax.set_ylabel('Acceleration [m/s²]')
        ax.set_title('Accelerometer')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[0, 1]
        ax.plot(t, np.degrees(dev_df['gyro_x']), 'r-', label='X', alpha=0.8)
        ax.plot(t, np.degrees(dev_df['gyro_y']), 'g-', label='Y', alpha=0.8)
        ax.plot(t, np.degrees(dev_df['gyro_z']), 'b-', label='Z', alpha=0.8)
        ax.set_ylabel('Angular Rate [°/s]')
        ax.set_title('Gyroscope')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[0, 2]
        ax.plot(t, dev_df['baro_alt'], 'b-', label='Baro', alpha=0.8)
        ax.plot(t, dev_df['tof_bottom'], 'r-', label='ToF', alpha=0.8)
        ax.plot(t, -dev_df['pos_z'], 'g--', label='ESKF (-pos_z)', alpha=0.8)
        ax.set_ylabel('Altitude [m]')
        ax.set_title('Altitude (Sensor vs ESKF)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Row 1: ESKF Attitude
        ax = axes[1, 0]
        ax.plot(t, np.degrees(dev_df['roll']), 'r-', label='Roll', alpha=0.8)
        ax.plot(t, np.degrees(dev_df['pitch']), 'g-', label='Pitch', alpha=0.8)
        ax.set_ylabel('Angle [°]')
        ax.set_title('ESKF Roll/Pitch')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1, 1]
        ax.plot(t, np.degrees(dev_df['yaw']), 'b-', alpha=0.8)
        ax.set_ylabel('Yaw [°]')
        ax.set_title('ESKF Yaw')
        ax.grid(True, alpha=0.3)

        ax = axes[1, 2]
        ax.plot(t, np.degrees(dev_df['gyro_bias_z']), 'b-', alpha=0.8)
        ax.set_ylabel('Bias [°/s]')
        ax.set_title('Gyro Bias Z (Yaw)')
        ax.grid(True, alpha=0.3)

        # Row 2: ESKF Position/Velocity
        ax = axes[2, 0]
        ax.plot(t, dev_df['pos_x'], 'r-', label='X', alpha=0.8)
        ax.plot(t, dev_df['pos_y'], 'g-', label='Y', alpha=0.8)
        ax.set_ylabel('Position [m]')
        ax.set_title('ESKF Position (NED X/Y)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[2, 1]
        ax.plot(t, dev_df['vel_x'], 'r-', label='Vx', alpha=0.8)
        ax.plot(t, dev_df['vel_y'], 'g-', label='Vy', alpha=0.8)
        ax.plot(t, dev_df['vel_z'], 'b-', label='Vz', alpha=0.8)
        ax.set_ylabel('Velocity [m/s]')
        ax.set_title('ESKF Velocity')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[2, 2]
        # NED座標系で上から見た図（北が上、東が右）
        ax.plot(dev_df['pos_y'], dev_df['pos_x'], 'b-', alpha=0.8)
        ax.plot(dev_df['pos_y'].iloc[0], dev_df['pos_x'].iloc[0], 'go', markersize=10, label='Start')
        ax.plot(dev_df['pos_y'].iloc[-1], dev_df['pos_x'].iloc[-1], 'rx', markersize=10, label='End')
        ax.set_xlabel('Y (East) [m]')
        ax.set_ylabel('X (North) [m]')
        ax.set_title('ESKF 2D Trajectory (Top View)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='datalim')

        # Row 3: Flow and Biases
        ax = axes[3, 0]
        ax.plot(t, dev_df['flow_dx'], 'r-', label='dx', alpha=0.8)
        ax.plot(t, dev_df['flow_dy'], 'g-', label='dy', alpha=0.8)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Flow Delta [counts]')
        ax.set_title('Optical Flow')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[3, 1]
        ax.plot(t, dev_df['accel_bias_x'], 'r-', label='Bias X', alpha=0.8)
        ax.plot(t, dev_df['accel_bias_y'], 'g-', label='Bias Y', alpha=0.8)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Bias [m/s²]')
        ax.set_title('Accel Bias X/Y')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Statistics
        ax = axes[3, 2]
        ax.axis('off')
        pos_drift = np.sqrt((dev_df['pos_x'].iloc[-1] - dev_df['pos_x'].iloc[0])**2 +
                           (dev_df['pos_y'].iloc[-1] - dev_df['pos_y'].iloc[0])**2)
        yaw_start = np.degrees(dev_df['yaw'].iloc[0])
        yaw_end = np.degrees(dev_df['yaw'].iloc[-1])
        stats_text = f"""
    === Device ESKF Statistics ===

    Duration: {t[-1]:.2f} seconds
    Sample Rate: {len(dev_df) / t[-1]:.1f} Hz

    === Attitude ===
    Yaw: {yaw_start:.1f}° → {yaw_end:.1f}° (drift: {yaw_end - yaw_start:.1f}°)

    === Position Drift ===
    X: {dev_df['pos_x'].iloc[0]:.3f}m → {dev_df['pos_x'].iloc[-1]:.3f}m
    Y: {dev_df['pos_y'].iloc[0]:.3f}m → {dev_df['pos_y'].iloc[-1]:.3f}m
    Total drift: {pos_drift:.3f}m

    === Biases (final) ===
    Gyro Z: {np.degrees(dev_df['gyro_bias_z'].iloc[-1]):.3f} °/s
    Accel X: {dev_df['accel_bias_x'].iloc[-1]:.4f} m/s²
    Accel Y: {dev_df['accel_bias_y'].iloc[-1]:.4f} m/s²
        """
        ax.text(0.05, 0.5, stats_text, transform=ax.transAxes, fontsize=9,
                verticalalignment='center', fontfamily='monospace')

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved: {output_file}")

    if show:
        plt.show()

    return fig


def visualize_pc_only(pc_df: pd.DataFrame, output_file: str = None, show: bool = True):
    """Visualize PC ESKF CSV only"""
    t = pc_df['time_s'].values

    fig, axes = plt.subplots(3, 3, figsize=(16, 12))
    fig.suptitle(f'PC ESKF Results - {len(pc_df)} samples, {t[-1]:.1f}s', fontsize=14)

    # Attitude
    ax = axes[0, 0]
    ax.plot(t, pc_df['roll_deg'], 'r-', label='Roll', alpha=0.8)
    ax.plot(t, pc_df['pitch_deg'], 'g-', label='Pitch', alpha=0.8)
    ax.set_ylabel('Angle [°]')
    ax.set_title('PC ESKF Roll/Pitch')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, pc_df['yaw_deg'], 'b-', alpha=0.8)
    ax.set_ylabel('Yaw [°]')
    ax.set_title('PC ESKF Yaw')
    ax.grid(True, alpha=0.3)

    ax = axes[0, 2]
    ax.plot(t, pc_df['gyro_bias_z'] * 1000, 'b-', alpha=0.8)
    ax.set_ylabel('Bias [mrad/s]')
    ax.set_title('PC Gyro Bias Z')
    ax.grid(True, alpha=0.3)

    # Position
    ax = axes[1, 0]
    ax.plot(t, pc_df['pos_x'], 'r-', label='X', alpha=0.8)
    ax.plot(t, pc_df['pos_y'], 'g-', label='Y', alpha=0.8)
    ax.set_ylabel('Position [m]')
    ax.set_title('PC ESKF Position (NED X/Y)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, pc_df['pos_z'], 'b-', alpha=0.8)
    ax.set_ylabel('Position Z [m]')
    ax.set_title('PC ESKF Position Z (NED)')
    ax.grid(True, alpha=0.3)

    # Velocity
    ax = axes[1, 2]
    ax.plot(t, pc_df['vel_x'], 'r-', label='Vx', alpha=0.8)
    ax.plot(t, pc_df['vel_y'], 'g-', label='Vy', alpha=0.8)
    ax.plot(t, pc_df['vel_z'], 'b-', label='Vz', alpha=0.8)
    ax.set_ylabel('Velocity [m/s]')
    ax.set_title('PC ESKF Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Trajectory - NED座標系で上から見た図（北が上、東が右）
    ax = axes[2, 0]
    ax.plot(pc_df['pos_y'], pc_df['pos_x'], 'b-', alpha=0.8)
    ax.plot(pc_df['pos_y'].iloc[0], pc_df['pos_x'].iloc[0], 'go', markersize=10, label='Start')
    ax.plot(pc_df['pos_y'].iloc[-1], pc_df['pos_x'].iloc[-1], 'rx', markersize=10, label='End')
    ax.set_xlabel('Y (East) [m]')
    ax.set_ylabel('X (North) [m]')
    ax.set_title('PC ESKF 2D Trajectory (Top View)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')

    # Biases
    ax = axes[2, 1]
    ax.plot(t, pc_df['accel_bias_x'] * 1000, 'r-', label='Bias X', alpha=0.8)
    ax.plot(t, pc_df['accel_bias_y'] * 1000, 'g-', label='Bias Y', alpha=0.8)
    ax.plot(t, pc_df['accel_bias_z'] * 1000, 'b-', label='Bias Z', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Bias [mm/s²]')
    ax.set_title('PC Accel Bias')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Statistics
    ax = axes[2, 2]
    ax.axis('off')
    pos_drift = np.sqrt((pc_df['pos_x'].iloc[-1] - pc_df['pos_x'].iloc[0])**2 +
                       (pc_df['pos_y'].iloc[-1] - pc_df['pos_y'].iloc[0])**2)
    stats_text = f"""
    === PC ESKF Statistics ===

    Duration: {t[-1]:.2f} seconds

    === Attitude ===
    Yaw: {pc_df['yaw_deg'].iloc[0]:.1f}° → {pc_df['yaw_deg'].iloc[-1]:.1f}°
    Drift: {pc_df['yaw_deg'].iloc[-1] - pc_df['yaw_deg'].iloc[0]:.1f}°

    === Position Drift ===
    X: {pc_df['pos_x'].iloc[0]:.3f}m → {pc_df['pos_x'].iloc[-1]:.3f}m
    Y: {pc_df['pos_y'].iloc[0]:.3f}m → {pc_df['pos_y'].iloc[-1]:.3f}m
    Total drift: {pos_drift:.3f}m

    === Biases (final) ===
    Gyro Z: {pc_df['gyro_bias_z'].iloc[-1] * 1000:.3f} mrad/s
    Accel X: {pc_df['accel_bias_x'].iloc[-1] * 1000:.4f} mm/s²
    """
    ax.text(0.05, 0.5, stats_text, transform=ax.transAxes, fontsize=9,
            verticalalignment='center', fontfamily='monospace')

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved: {output_file}")

    if show:
        plt.show()

    return fig


def visualize_comparison(dev_df: pd.DataFrame, pc_df: pd.DataFrame,
                        output_file: str = None, show: bool = True):
    """Visualize comparison between device (V2) and PC ESKF results"""

    t_dev = dev_df['time_s'].values
    t_pc = pc_df['time_s'].values

    fig, axes = plt.subplots(4, 3, figsize=(18, 16))
    fig.suptitle(f'Device vs PC ESKF Comparison - Device: {t_dev[-1]:.1f}s, PC: {t_pc[-1]:.1f}s', fontsize=14)

    # Row 0: Attitude comparison
    ax = axes[0, 0]
    ax.plot(t_dev, np.degrees(dev_df['roll']), 'r-', label='Device Roll', alpha=0.8)
    ax.plot(t_pc, pc_df['roll_deg'], 'r--', label='PC Roll', alpha=0.8)
    ax.plot(t_dev, np.degrees(dev_df['pitch']), 'g-', label='Device Pitch', alpha=0.8)
    ax.plot(t_pc, pc_df['pitch_deg'], 'g--', label='PC Pitch', alpha=0.8)
    ax.set_ylabel('Angle [°]')
    ax.set_title('Roll/Pitch Comparison')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t_dev, np.degrees(dev_df['yaw']), 'b-', label='Device', alpha=0.8, linewidth=1.5)
    ax.plot(t_pc, pc_df['yaw_deg'], 'r--', label='PC', alpha=0.8, linewidth=1.5)
    ax.set_ylabel('Yaw [°]')
    ax.set_title('Yaw Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 2]
    # Calculate yaw difference (interpolate PC to device timestamps)
    pc_yaw_interp = np.interp(t_dev, t_pc, pc_df['yaw_deg'].values)
    yaw_diff = np.degrees(dev_df['yaw'].values) - pc_yaw_interp
    ax.plot(t_dev, yaw_diff, 'purple', alpha=0.8)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax.set_ylabel('Yaw Diff [°]')
    ax.set_title(f'Yaw Difference (Device - PC), Mean: {np.mean(yaw_diff):.2f}°')
    ax.grid(True, alpha=0.3)

    # Row 1: Position comparison
    ax = axes[1, 0]
    ax.plot(t_dev, dev_df['pos_x'], 'r-', label='Device X', alpha=0.8)
    ax.plot(t_pc, pc_df['pos_x'], 'r--', label='PC X', alpha=0.8)
    ax.plot(t_dev, dev_df['pos_y'], 'g-', label='Device Y', alpha=0.8)
    ax.plot(t_pc, pc_df['pos_y'], 'g--', label='PC Y', alpha=0.8)
    ax.set_ylabel('Position [m]')
    ax.set_title('Position X/Y Comparison')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t_dev, -dev_df['pos_z'], 'b-', label='Device (-Z)', alpha=0.8)
    ax.plot(t_pc, -pc_df['pos_z'], 'r--', label='PC (-Z)', alpha=0.8)
    ax.plot(t_dev, dev_df['tof_bottom'], 'g:', label='ToF', alpha=0.5)
    ax.set_ylabel('Altitude [m]')
    ax.set_title('Altitude Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # NED座標系で上から見た図（北が上、東が右）
    ax = axes[1, 2]
    ax.plot(dev_df['pos_y'], dev_df['pos_x'], 'b-', label='Device', alpha=0.8, linewidth=1.5)
    ax.plot(pc_df['pos_y'], pc_df['pos_x'], 'r--', label='PC', alpha=0.8, linewidth=1.5)
    ax.plot(dev_df['pos_y'].iloc[0], dev_df['pos_x'].iloc[0], 'go', markersize=10, label='Start')
    ax.plot(dev_df['pos_y'].iloc[-1], dev_df['pos_x'].iloc[-1], 'bx', markersize=10, label='Dev End')
    ax.plot(pc_df['pos_y'].iloc[-1], pc_df['pos_x'].iloc[-1], 'rx', markersize=10, label='PC End')
    ax.set_xlabel('Y (East) [m]')
    ax.set_ylabel('X (North) [m]')
    ax.set_title('2D Trajectory Comparison (Top View)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')

    # Row 2: Velocity comparison (色分けして実線)
    ax = axes[2, 0]
    ax.plot(t_dev, dev_df['vel_x'], 'b-', label='Device', alpha=0.8, linewidth=1)
    ax.plot(t_pc, pc_df['vel_x'], 'r-', label='PC', alpha=0.8, linewidth=1)
    ax.set_ylabel('Velocity X [m/s]')
    ax.set_title('Velocity X Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    ax.plot(t_dev, dev_df['vel_y'], 'b-', label='Device', alpha=0.8, linewidth=1)
    ax.plot(t_pc, pc_df['vel_y'], 'r-', label='PC', alpha=0.8, linewidth=1)
    ax.set_ylabel('Velocity Y [m/s]')
    ax.set_title('Velocity Y Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2, 2]
    ax.plot(t_dev, dev_df['vel_z'], 'b-', label='Device', alpha=0.8, linewidth=1)
    ax.plot(t_pc, pc_df['vel_z'], 'r-', label='PC', alpha=0.8, linewidth=1)
    ax.set_ylabel('Velocity Z [m/s]')
    ax.set_title('Velocity Z Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Row 3: Biases and Statistics
    ax = axes[3, 0]
    ax.plot(t_dev, np.degrees(dev_df['gyro_bias_z']), 'b-', label='Device', alpha=0.8)
    ax.plot(t_pc, np.degrees(pc_df['gyro_bias_z']), 'r--', label='PC', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Gyro Bias Z [°/s]')
    ax.set_title('Gyro Bias Z Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[3, 1]
    ax.plot(t_dev, dev_df['accel_bias_x'], 'r-', label='Dev X', alpha=0.8)
    ax.plot(t_pc, pc_df['accel_bias_x'], 'r--', label='PC X', alpha=0.8)
    ax.plot(t_dev, dev_df['accel_bias_y'], 'g-', label='Dev Y', alpha=0.8)
    ax.plot(t_pc, pc_df['accel_bias_y'], 'g--', label='PC Y', alpha=0.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Accel Bias [m/s²]')
    ax.set_title('Accel Bias X/Y Comparison')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Statistics
    ax = axes[3, 2]
    ax.axis('off')

    dev_pos_drift = np.sqrt((dev_df['pos_x'].iloc[-1])**2 + (dev_df['pos_y'].iloc[-1])**2)
    pc_pos_drift = np.sqrt((pc_df['pos_x'].iloc[-1])**2 + (pc_df['pos_y'].iloc[-1])**2)
    dev_yaw_drift = np.degrees(dev_df['yaw'].iloc[-1] - dev_df['yaw'].iloc[0])
    pc_yaw_drift = pc_df['yaw_deg'].iloc[-1] - pc_df['yaw_deg'].iloc[0]

    stats_text = f"""
    === Comparison Statistics ===

    Duration: Device {t_dev[-1]:.2f}s, PC {t_pc[-1]:.2f}s

    === Yaw Drift ===
    Device: {dev_yaw_drift:.1f}°
    PC:     {pc_yaw_drift:.1f}°
    Diff:   {dev_yaw_drift - pc_yaw_drift:.1f}°

    === Position Drift (from origin) ===
    Device: {dev_pos_drift:.3f}m
    PC:     {pc_pos_drift:.3f}m

    === Final Position ===
    Device: ({dev_df['pos_x'].iloc[-1]:.3f}, {dev_df['pos_y'].iloc[-1]:.3f})m
    PC:     ({pc_df['pos_x'].iloc[-1]:.3f}, {pc_df['pos_y'].iloc[-1]:.3f})m

    === Gyro Bias Z (final) ===
    Device: {np.degrees(dev_df['gyro_bias_z'].iloc[-1]):.3f} °/s
    PC:     {np.degrees(pc_df['gyro_bias_z'].iloc[-1]):.3f} °/s
    """
    ax.text(0.05, 0.5, stats_text, transform=ax.transAxes, fontsize=9,
            verticalalignment='center', fontfamily='monospace')

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved: {output_file}")

    if show:
        plt.show()

    return fig


def visualize_mag_xy(dev_df: pd.DataFrame, output_file: str = None, show: bool = True):
    """Visualize magnetometer XY data for calibration verification"""
    t = dev_df['time_s'].values
    mag_x = dev_df['mag_x'].values
    mag_y = dev_df['mag_y'].values
    mag_z = dev_df['mag_z'].values

    # Calculate statistics
    center_x = (np.max(mag_x) + np.min(mag_x)) / 2
    center_y = (np.max(mag_y) + np.min(mag_y)) / 2
    range_x = np.max(mag_x) - np.min(mag_x)
    range_y = np.max(mag_y) - np.min(mag_y)
    norm = np.sqrt(mag_x**2 + mag_y**2 + mag_z**2)

    print(f"\n=== Magnetometer Statistics ===")
    print(f"Samples: {len(dev_df)}")
    print(f"X: min={np.min(mag_x):.1f}, max={np.max(mag_x):.1f}, center={center_x:.1f}")
    print(f"Y: min={np.min(mag_y):.1f}, max={np.max(mag_y):.1f}, center={center_y:.1f}")
    print(f"Z: min={np.min(mag_z):.1f}, max={np.max(mag_z):.1f}, mean={np.mean(mag_z):.1f}")
    print(f"Norm: mean={np.mean(norm):.1f}, std={np.std(norm):.1f}")

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Magnetometer XY Calibration Verification', fontsize=14)

    # XY scatter plot
    ax = axes[0]
    scatter = ax.scatter(mag_x, mag_y, c=t, cmap='viridis', s=10, alpha=0.7)
    plt.colorbar(scatter, ax=ax, label='Time [s]')

    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax.axvline(x=0, color='gray', linestyle='--', alpha=0.5)
    ax.plot(center_x, center_y, 'r+', markersize=15, markeredgewidth=2,
            label=f'Center ({center_x:.1f}, {center_y:.1f})')
    ax.plot(0, 0, 'ko', markersize=8, label='Origin')

    # Draw fitted circle
    radius = (range_x + range_y) / 4
    theta = np.linspace(0, 2*np.pi, 100)
    ax.plot(center_x + radius * np.cos(theta), center_y + radius * np.sin(theta),
            'r--', alpha=0.5, label=f'Fitted circle (r={radius:.1f})')

    ax.set_xlabel('Mag X [uT]')
    ax.set_ylabel('Mag Y [uT]')
    ax.set_title('Magnetometer XY (colored by time)')
    ax.legend(loc='upper right')
    ax.axis('equal')
    ax.grid(True, alpha=0.3)

    # Norm over time
    ax = axes[1]
    ax.plot(t, norm, 'b-', alpha=0.7, linewidth=0.5)
    ax.axhline(y=np.mean(norm), color='r', linestyle='--', label=f'Mean: {np.mean(norm):.1f} uT')
    ax.axhline(y=45, color='g', linestyle=':', label='Expected: ~45 uT')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Mag Norm [uT]')
    ax.set_title('Magnetometer Norm over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved: {output_file}")

    if show:
        plt.show()

    return fig


def main():
    parser = argparse.ArgumentParser(
        description='StampFly ESKF Log Unified Visualization Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Device log only
    python visualize_device_log.py sensor.bin

    # PC ESKF CSV only
    python visualize_device_log.py --pc eskf.csv --mode pc

    # Compare device and PC
    python visualize_device_log.py sensor.bin --pc eskf.csv --mode both

    # Magnetometer XY plot
    python visualize_device_log.py sensor.bin --mag-xy
        """)

    parser.add_argument('input', nargs='?', help='Input binary log file (.bin)')
    parser.add_argument('--pc', '-p', help='PC ESKF output CSV file')
    parser.add_argument('--mode', '-m', choices=['device', 'pc', 'both'], default='device',
                        help='Visualization mode: device, pc, or both (default: device)')
    parser.add_argument('--mag-xy', action='store_true', help='Show magnetometer XY plot')
    parser.add_argument('--output', '-o', help='Output image file (.png)')
    parser.add_argument('--no-show', action='store_true', help='Do not show plot window')

    args = parser.parse_args()

    # Validate arguments
    if args.mode == 'pc' and not args.pc:
        print("Error: --pc required for mode 'pc'")
        sys.exit(1)

    if args.mode in ['device', 'both'] and not args.input:
        print("Error: Input file required for mode 'device' or 'both'")
        sys.exit(1)

    if args.mode == 'both' and not args.pc:
        print("Error: --pc required for mode 'both'")
        sys.exit(1)

    if args.mag_xy and not args.input:
        print("Error: Input file required for --mag-xy")
        sys.exit(1)

    # Load device log if needed
    dev_df = None
    if args.input:
        input_path = Path(args.input)
        if not input_path.exists():
            print(f"Error: Input file not found: {args.input}")
            sys.exit(1)

        version = detect_file_version(args.input)
        print(f"Detected packet version: V{version}")
        print(f"Packet size: {PACKET_SIZE_V2 if version == 2 else PACKET_SIZE_V1} bytes")

        packets = parse_log_file(args.input, version)
        if not packets:
            print("No valid packets found")
            sys.exit(1)

        print(f"Loaded {len(packets)} packets")
        dev_df = packets_to_dataframe(packets)

    # Load PC ESKF CSV if needed
    pc_df = None
    if args.pc:
        pc_path = Path(args.pc)
        if not pc_path.exists():
            print(f"Error: PC CSV file not found: {args.pc}")
            sys.exit(1)

        print(f"Loading PC ESKF CSV: {args.pc}")
        pc_df = load_pc_eskf_csv(args.pc)
        print(f"Loaded {len(pc_df)} samples")

    # Generate output filename if not specified
    output_file = args.output
    if not output_file and args.input:
        input_path = Path(args.input)
        suffix = '_mag_xy' if args.mag_xy else '_analysis'
        output_file = input_path.stem + suffix + '.png'

    show = not args.no_show

    # Magnetometer XY plot
    if args.mag_xy:
        visualize_mag_xy(dev_df, output_file, show)
        return

    # Regular visualization
    if args.mode == 'device':
        visualize_device_only(dev_df, output_file, show)
    elif args.mode == 'pc':
        visualize_pc_only(pc_df, output_file, show)
    elif args.mode == 'both':
        if dev_df['version'].iloc[0] == 1:
            print("Warning: Device log is V1 (sensor only). Cannot compare ESKF results.")
            print("Use V2 log (binlog v2) for comparison.")
            visualize_device_only(dev_df, output_file, show)
        else:
            visualize_comparison(dev_df, pc_df, output_file, show)


if __name__ == '__main__':
    main()
