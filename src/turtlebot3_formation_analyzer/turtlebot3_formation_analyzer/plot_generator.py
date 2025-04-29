#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

def generate_plots_from_file(filename='formation_data.csv', output_prefix='formation_plots'):
    """Generate static plots from saved data file"""
    if not os.path.exists(filename):
        print(f"Error: File {filename} not found!")
        return
    
    try:
        # Load data from CSV
        data = np.loadtxt(filename, delimiter=',', skiprows=1)
        
        # Extract data columns
        times = data[:, 0]
        long_dist_l1_f1 = data[:, 1]
        long_dist_f1_f2 = data[:, 2]
        lat_dist_l1_f1 = data[:, 3]
        lat_dist_f1_f2 = data[:, 4]
        long_vel_diff_l1_f1 = data[:, 5]
        long_vel_diff_f1_f2 = data[:, 6]
        lat_vel_diff_l1_f1 = data[:, 7]
        lat_vel_diff_f1_f2 = data[:, 8]
        
        # Create distance plots (2 subplots stacked vertically)
        fig1, axs1 = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot longitudinal distance
        axs1[0].plot(times, long_dist_l1_f1, 'r-', label='Longitudinal distance between L1 and F1')
        axs1[0].plot(times, long_dist_f1_f2, 'b-', label='Longitudinal distance between F1 and F2')
        axs1[0].set_title('(a) Longitudinal distance')
        axs1[0].set_xlabel('Time(s)')
        axs1[0].set_ylabel('Longitudinal distance(m)')
        axs1[0].legend()
        axs1[0].grid(True)
        
        # Plot lateral distance
        axs1[1].plot(times, lat_dist_l1_f1, 'r-', label='Lateral distance between L1 and F1')
        axs1[1].plot(times, lat_dist_f1_f2, 'b-', label='Lateral distance between F1 and F2')
        axs1[1].set_title('(b) Lateral distance')
        axs1[1].set_xlabel('Time(s)')
        axs1[1].set_ylabel('Lateral distance(m)')
        axs1[1].legend()
        axs1[1].grid(True)
        
        plt.tight_layout()
        distance_plot_file = f"{output_prefix}_distances"
        plt.savefig(f"{distance_plot_file}.png", dpi=300)
        plt.savefig(f"{distance_plot_file}.pdf")
        print(f"Distance plots saved to {distance_plot_file}.png and {distance_plot_file}.pdf")
        
        # Create velocity difference plots (2 subplots stacked vertically)
        fig2, axs2 = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot longitudinal velocity difference
        axs2[0].plot(times, long_vel_diff_l1_f1, 'r-', label='Longitudinal velocity difference between L1 and F1')
        axs2[0].plot(times, long_vel_diff_f1_f2, 'b-', label='Longitudinal velocity difference between F1 and F2')
        axs2[0].set_title('(a) Longitudinal velocity difference')
        axs2[0].set_xlabel('Time(s)')
        axs2[0].set_ylabel('Longitudinal velocity difference(m/s)')
        axs2[0].legend()
        axs2[0].grid(True)
        
        # Plot lateral velocity difference
        axs2[1].plot(times, lat_vel_diff_l1_f1, 'r-', label='Lateral velocity difference between L1 and F1')
        axs2[1].plot(times, lat_vel_diff_f1_f2, 'b-', label='Lateral velocity difference between F1 and F2')
        axs2[1].set_title('(b) Lateral velocity difference')
        axs2[1].set_xlabel('Time(s)')
        axs2[1].set_ylabel('Lateral velocity difference(m/s)')
        axs2[1].legend()
        axs2[1].grid(True)
        
        plt.tight_layout()
        velocity_plot_file = f"{output_prefix}_velocities"
        plt.savefig(f"{velocity_plot_file}.png", dpi=300)
        plt.savefig(f"{velocity_plot_file}.pdf")
        print(f"Velocity plots saved to {velocity_plot_file}.png and {velocity_plot_file}.pdf")
        
    except Exception as e:
        print(f"Error generating plots from file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate plots from formation data CSV file')
    parser.add_argument('-f', '--file', default='formation_data.csv', help='Input CSV file path')
    parser.add_argument('-o', '--output', default='formation_plots', help='Output file prefix')
    
    args = parser.parse_args()
    generate_plots_from_file(args.file, args.output)

