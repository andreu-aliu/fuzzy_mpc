#!/usr/bin/env python3

import pandas as pd
import numpy as np
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

def visualize_mpc_debug(debug_folder="/home/andreu/ros_ws/src/as/control/fuzzy_mpc/debug/csv",
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

    def _read_csv_if_exists(path: Path):
        if not path.exists():
            return None
        return pd.read_csv(path, header=None).values

    # Load all rows once; the slider will pick which row to display.
    u_opt_raw = _read_csv_if_exists(folder / "u_opt.csv")
    pred_states_raw = _read_csv_if_exists(folder / "x_pred.csv")
    x_ref_raw = _read_csv_if_exists(folder / "x_ref.csv")
    x0_raw = _read_csv_if_exists(folder / "x0.csv")  # optional (starting point marker)

    if u_opt_raw is None or pred_states_raw is None or x_ref_raw is None:
        raise FileNotFoundError("Missing required CSV(s): u_opt.csv, x_pred.csv, x_ref.csv")

    x_prev_raw = None
    u_prev_raw = None
    if show_prev:
        x_prev_raw = _read_csv_if_exists(folder / "x_prev.csv")
        u_prev_raw = _read_csv_if_exists(folder / "u_prev.csv")

    # Slider max is based on how many rows exist in the CSVs.
    n_rows = min(
        u_opt_raw.shape[0],
        pred_states_raw.shape[0],
        x_ref_raw.shape[0],
        x0_raw.shape[0] if x0_raw is not None else u_opt_raw.shape[0],
        x_prev_raw.shape[0] if x_prev_raw is not None else u_opt_raw.shape[0],
        u_prev_raw.shape[0] if u_prev_raw is not None else u_opt_raw.shape[0],
    )
    if n_rows <= 0:
        raise ValueError("CSV files have no rows to visualize")

    if row < 0:
        row = n_rows - 1
    row = int(np.clip(row, 0, n_rows - 1))

    def _prep_x0(x0: np.ndarray | None):
        if x0 is None:
            return None
        x0 = np.asarray(x0)
        if x0.ndim == 1:
            x0 = x0.reshape(1, -1)
        if x0.shape[1] == 1 and x0.shape[0] == n_states:
            x0 = x0.reshape(1, n_states)
        elif x0.shape[1] == 1 and x0.shape[0] != 1 and x0.shape[0] == n_rows * n_states:
            x0 = x0.reshape(n_rows, n_states)
        elif x0.shape[1] >= n_states:
            x0 = x0[:, :n_states]
        else:
            padded = np.full((x0.shape[0], n_states), np.nan, dtype=float)
            padded[:, : x0.shape[1]] = x0
            x0 = padded
        if x0.shape[0] == 1 and n_rows > 1:
            x0 = np.repeat(x0, n_rows, axis=0)
        return x0

    # Pre-reshape into 3D arrays for quick slider updates.
    u_opt_all = u_opt_raw[:n_rows].reshape(n_rows, n_horizon, n_inputs)
    pred_st_all = pred_states_raw[:n_rows].reshape(n_rows, n_horizon, n_states)
    ref_st_all = x_ref_raw[:n_rows].reshape(n_rows, n_horizon, n_states)
    x0_all = _prep_x0(x0_raw[:n_rows] if x0_raw is not None else None)
    prev_st_all = x_prev_raw[:n_rows].reshape(n_rows, n_horizon, n_states) if x_prev_raw is not None else None
    prev_u_all = u_prev_raw[:n_rows].reshape(n_rows, n_horizon, n_inputs) if u_prev_raw is not None else None

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

    fig.suptitle(f"CSV row: {row}/{n_rows - 1}", y=0.995)

    def _autoscale_axis_y(ax, pad_frac=0.05):
        ys = []
        for line in ax.lines:
            y = np.asarray(line.get_ydata())
            if y.size:
                ys.append(y)
        if not ys:
            return
        y_all = np.concatenate(ys)
        y_all = y_all[np.isfinite(y_all)]
        if y_all.size == 0:
            return

        y_min = float(np.min(y_all))
        y_max = float(np.max(y_all))
        if y_min == y_max:
            pad = max(1e-3, abs(y_min) * pad_frac)
        else:
            pad = (y_max - y_min) * pad_frac
        ax.set_ylim(y_min - pad, y_max + pad)

    ref_lines = []
    pred_lines = []
    prev_state_lines = []
    x0_dots = []

    # Plot each state on its own subplot.
    for i in range(n_states):
        state_name = state_names[i]
        (ref_line,) = axes[i].plot(horizon_range, ref_st_all[row, :, i], 'o-', label=f'Ref {state_name}', linewidth=2)
        ref_lines.append(ref_line)

        if x0_all is not None:
            (x0_dot,) = axes[i].plot([0], [x0_all[row, i]], 'o', color='red', label='x0', markersize=7, zorder=5)
        else:
            x0_dot = None
        x0_dots.append(x0_dot)

        if prev_st_all is not None:
            (prev_line,) = axes[i].plot(horizon_range, prev_st_all[row, :, i], 'x:', label=f'Prev {state_name}', linewidth=2)
        else:
            prev_line = None
        prev_state_lines.append(prev_line)

        (pred_line,) = axes[i].plot(horizon_range, pred_st_all[row, :, i], 's--', label=f'Pred {state_name}', linewidth=2)
        pred_lines.append(pred_line)
        axes[i].set_ylabel(state_name)
        axes[i].set_title(f"{state_name}: Ref / Prev / Pred")
        axes[i].legend()
        axes[i].grid(True, alpha=0.3)

    input_lines = []
    prev_input_lines = []

    # Plot each control action on its own subplot below the states.
    for i in range(n_inputs):
        input_name = input_names[i]
        axis = axes[n_states + i]
        (u_line,) = axis.plot(horizon_range, u_opt_all[row, :, i], 'o-', label=f'u_opt {input_name}', linewidth=2)
        input_lines.append(u_line)

        if prev_u_all is not None:
            (u_prev_line,) = axis.plot(horizon_range, prev_u_all[row, :, i], 'x:', label=f'u_prev {input_name}', linewidth=2)
        else:
            u_prev_line = None
        prev_input_lines.append(u_prev_line)
        axis.set_ylabel(input_name)
        axis.set_title(f"{input_name}: u_opt / u_prev")
        axis.legend()
        axis.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Horizon Step")
    axes[-1].set_xlim(horizon_range[0], horizon_range[-1])

    for ax in axes:
        _autoscale_axis_y(ax)
    
    # Slider: select which CSV row/iteration to visualize.
    fig.subplots_adjust(bottom=0.06)
    ax_slider = fig.add_axes([0.15, 0.015, 0.7, 0.02])
    row_slider = Slider(
        ax=ax_slider,
        label="Row",
        valmin=0,
        valmax=n_rows - 1,
        valinit=row,
        valstep=1,
        valfmt="%0.0f",
    )

    def _update_plot(selected_row):
        r = int(selected_row)
        fig.suptitle(f"CSV row: {r}/{n_rows - 1}", y=0.995)

        for i in range(n_states):
            ref_lines[i].set_ydata(ref_st_all[r, :, i])
            pred_lines[i].set_ydata(pred_st_all[r, :, i])
            if x0_dots[i] is not None:
                x0_dots[i].set_ydata([x0_all[r, i]])
            if prev_state_lines[i] is not None:
                prev_state_lines[i].set_ydata(prev_st_all[r, :, i])

        for i in range(n_inputs):
            input_lines[i].set_ydata(u_opt_all[r, :, i])
            if prev_input_lines[i] is not None:
                prev_input_lines[i].set_ydata(prev_u_all[r, :, i])

        for ax in axes:
            _autoscale_axis_y(ax)

        fig.canvas.draw_idle()

    row_slider.on_changed(_update_plot)

    fig.tight_layout(rect=[0, 0.04, 1, 0.985])
    plt.show()

if __name__ == "__main__":
    visualize_mpc_debug(n_inputs=2, n_states=6, n_horizon=60)
