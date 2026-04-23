#!/usr/bin/env python3

import pandas as pd
import numpy as np
from pathlib import Path

import matplotlib.pyplot as plt

def visualize_mpc_debug(debug_folder="/home/andreu/ros_ws/src/as/control/fuzzy_mpc/debug",
                        n_inputs=2, n_states=6, n_horizon=60, row=-1, state_names=None, input_names=None,
                        show_prev=True):
    """
    Visualize MPC debug data from CSV files.
    
    Args:
        debug_folder: Path to folder containing CSV files
        n_inputs: Number of control inputs
        n_states: Number of states
        n_horizon: Prediction horizon
        row: Row index to plot (default: -1 for last row)
        state_names: Optional list of state names for display (e.g. ["y", "vy", ...])
        input_names: Optional list of input names for display (e.g. ["steering", "mz"])
        show_prev: If True and files exist, overlay x_prev/u_prev (linearization points)
    """
    folder = Path(debug_folder)

    if state_names is None:
        state_names = ["y", "vy", "phi", "r", "delta", "delta_dot"]
    # Ensure we always have a usable label per state even if n_states differs.
    state_names = list(state_names)
    if len(state_names) < n_states:
        state_names.extend(f"state_{i}" for i in range(len(state_names), n_states))
    else:
        state_names = state_names[:n_states]

    if input_names is None:
        input_names = ["steering", "mz"]
    input_names = list(input_names)
    if len(input_names) < n_inputs:
        input_names.extend(f"input_{i}" for i in range(len(input_names), n_inputs))
    else:
        input_names = input_names[:n_inputs]

    # Read CSV files
    delta_u_opt = pd.read_csv(folder / "u_opt.csv", header=None).iloc[row].values
    pred_states = pd.read_csv(folder / "x_pred.csv", header=None).iloc[row].values
    x_ref = pd.read_csv(folder / "x_ref.csv", header=None).iloc[row].values
    x0 = pd.read_csv(folder / "x0.csv", header=None).iloc[row].values

    prev_states = None
    prev_controls = None
    if show_prev:
        x_prev_path = folder / "x_prev.csv"
        u_prev_path = folder / "u_prev.csv"
        if x_prev_path.exists():
            prev_states = pd.read_csv(x_prev_path, header=None).iloc[row].values
        if u_prev_path.exists():
            prev_controls = pd.read_csv(u_prev_path, header=None).iloc[row].values
    
    # Reshape vectors
    u_opt = delta_u_opt.reshape(n_horizon, n_inputs)
    pred_st = pred_states.reshape(n_horizon, n_states)
    ref_st = x_ref.reshape(n_horizon, n_states)
    prev_st = prev_states.reshape(n_horizon, n_states) if prev_states is not None else None
    prev_u = prev_controls.reshape(n_horizon, n_inputs) if prev_controls is not None else None
    
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
        state_name = state_names[i]
        axes[i].plot(horizon_range, ref_st[:, i], 'o-', label=f'Ref {state_name}', linewidth=2)
        if prev_st is not None:
            axes[i].plot(horizon_range, prev_st[:, i], 'x:', label=f'Prev {state_name}', linewidth=2)
        axes[i].plot(horizon_range, pred_st[:, i], 's--', label=f'Pred {state_name}', linewidth=2)
        axes[i].set_ylabel(state_name)
        axes[i].set_title(f"{state_name}: Ref / Prev / Pred")
        axes[i].legend()
        axes[i].grid(True, alpha=0.3)

    # Plot each control action on its own subplot below the states.
    for i in range(n_inputs):
        input_name = input_names[i]
        axis = axes[n_states + i]
        axis.plot(horizon_range, u_opt[:, i], 'o-', label=f'u_opt {input_name}', linewidth=2)
        if prev_u is not None:
            axis.plot(horizon_range, prev_u[:, i], 'x:', label=f'u_prev {input_name}', linewidth=2)
        axis.set_ylabel(input_name)
        axis.set_title(f"{input_name}: u_opt / u_prev")
        axis.legend()
        axis.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Horizon Step")
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    visualize_mpc_debug(n_inputs=2, n_states=6, n_horizon=60, row=3)
