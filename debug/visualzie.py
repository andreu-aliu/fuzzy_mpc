#!/usr/bin/env python3

import pandas as pd
import numpy as np
from pathlib import Path

import matplotlib.pyplot as plt

def visualize_mpc_debug(debug_folder="/home/andreu/ros_ws/src/as/control/fuzzy_mpc/debug",
                        n_inputs=1, n_states=6, n_horizon=60, row=-1):
    """
    Visualize MPC debug data from CSV files.
    
    Args:
        debug_folder: Path to folder containing CSV files
        n_inputs: Number of control inputs
        n_states: Number of states
        n_horizon: Prediction horizon
        row: Row index to plot (default: -1 for last row)
    """
    folder = Path(debug_folder)
    
    # Read CSV files
    delta_u_opt = pd.read_csv(folder / "delta_u_opt.csv", header=None).iloc[row].values
    pred_states = pd.read_csv(folder / "pred_states.csv", header=None).iloc[row].values
    x_ref = pd.read_csv(folder / "x_ref.csv", header=None).iloc[row].values
    x0 = pd.read_csv(folder / "x0.csv", header=None).iloc[row].values
    
    # Reshape vectors
    u_opt = delta_u_opt.reshape(n_horizon, n_inputs)
    pred_st = pred_states.reshape(n_horizon, n_states)
    ref_st = x_ref.reshape(n_horizon, n_states)
    
    total_plots = n_states + n_inputs
    horizon_range = np.arange(n_horizon)
    fig_height = max(2.8 * total_plots, 6)
    fig, axes = plt.subplots(
        total_plots,
        1,
        figsize=(12, fig_height),
        sharex=True,
        squeeze=False,
    )
    axes = axes.flatten()

    # Plot each state on its own subplot.
    for i in range(n_states):
        axes[i].plot(horizon_range, ref_st[:, i], 'o-', label=f'Ref State {i}', linewidth=2)
        axes[i].plot(horizon_range, pred_st[:, i], 's--', label=f'Pred State {i}', linewidth=2)
        axes[i].set_ylabel(f"State {i}")
        axes[i].set_title(f"State {i}: Reference vs Predicted")
        axes[i].legend()
        axes[i].grid(True, alpha=0.3)

    # Plot each control action on its own subplot below the states.
    for i in range(n_inputs):
        axis = axes[n_states + i]
        axis.plot(horizon_range, u_opt[:, i], 'o-', label=f'Input {i}', linewidth=2)
        axis.set_ylabel(f"Input {i}")
        axis.set_title(f"Optimal Control Input {i}")
        axis.legend()
        axis.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Horizon Step")
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    visualize_mpc_debug(n_inputs=1, n_states=6, n_horizon=60)
