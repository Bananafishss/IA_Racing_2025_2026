#!/usr/bin/env python3
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import glob


def find_latest_data(data_dir="/home/pi/mycar/data_plot"):
    """Find latest data_N.csv file"""
    pattern = os.path.join(data_dir, "data_*.csv")
    files = glob.glob(pattern)
    
    if not files:
        print("❌ No data files found!")
        return None
    
    files_sorted = sorted(files, key=lambda x: int(x.split('_')[-1].replace('.csv', '')))
    latest = files_sorted[-1]
    
    print(f"📂 Latest data: {os.path.basename(latest)}")
    return latest


def load_data(csv_path):
    """Load CSV handling empty values"""
    df = pd.read_csv(csv_path)
    df.replace('', np.nan, inplace=True)
    
    for col in ['offset_m', 'angle_deg', 'steer', 'throttle', 'current_speed', 'current_yaw']:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    
    print(f"✅ Loaded {len(df)} frames ({df['timestamp'].iloc[-1]:.1f}s)")
    return df


def plot_perception_control(df, output_path):
    """Generate plot with 2 subplots: Perception + Control"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    
    t = df['timestamp']
    
    # Perception subplot
    ax1.plot(t, df['offset_m'], 'b-', label='offset_m', linewidth=1.5)
    ax1_twin = ax1.twinx()
    ax1_twin.plot(t, df['angle_deg'], 'r-', label='angle_deg', linewidth=1.5)
    
    ax1.set_ylabel('Offset (m)', color='b', fontsize=12)
    ax1_twin.set_ylabel('Angle (deg)', color='r', fontsize=12)
    ax1.tick_params(axis='y', labelcolor='b')
    ax1_twin.tick_params(axis='y', labelcolor='r')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Perception: Offset & Angle', fontsize=14, fontweight='bold')
    
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_twin.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    
    # Control subplot
    ax2.plot(t, df['steer'], 'g-', label='steer', linewidth=1.5)
    ax2_twin = ax2.twinx()
    ax2_twin.plot(t, df['throttle'], 'm-', label='throttle', linewidth=1.5)
    
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Steer [-1,1]', color='g', fontsize=12)
    ax2_twin.set_ylabel('Throttle [0,1]', color='m', fontsize=12)
    ax2.tick_params(axis='y', labelcolor='g')
    ax2_twin.tick_params(axis='y', labelcolor='m')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Control: Steer & Throttle', fontsize=14, fontweight='bold')
    
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    print(f"📊 Saved: {output_path}")


def main():
    print("=" * 50)
    print("🚗 DonkeyCar Plot Generator (Offline)")
    print("=" * 50)
    
    csv_path = find_latest_data()
    if csv_path is None:
        return
    
    df = load_data(csv_path)
    
    base_name = os.path.basename(csv_path).replace('.csv', '')
    plot_dir = f"/home/pi/mycar/plots_{base_name}"
    os.makedirs(plot_dir, exist_ok=True)
    
    print("\n📊 Generating plot...")
    plot_perception_control(df, os.path.join(plot_dir, f"{base_name}_perception_control.png"))
    
    print("\n✅ Done! Plot saved in:", plot_dir)
    print("=" * 50)


if __name__ == "__main__":
    main()
