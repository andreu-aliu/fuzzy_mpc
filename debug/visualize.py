#!/usr/bin/env python3

import pandas as pd
import numpy as np
from pathlib import Path
import warnings

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
        try:
            df = pd.read_csv(path, header=None)
        except Exception as exc:
            warnings.warn(f"Failed to read {path}: {exc}")
            return None
        if df.empty:
            warnings.warn(f"CSV is empty: {path}")
            return None
        return df.values

    def _reshape_or_none(raw: np.ndarray | None, shape: tuple[int, ...], name: str):
        if raw is None:
            return None
        try:
            return raw.reshape(*shape)
        except Exception as exc:
            warnings.warn(f"Skipping {name}: cannot reshape {raw.shape} to {shape}: {exc}")
            return None

    def _pad_rows(arr3: np.ndarray | None, target_rows: int):
        if arr3 is None:
            return None
        if arr3.shape[0] >= target_rows:
            return arr3[:target_rows]
        padded = np.full((target_rows, *arr3.shape[1:]), np.nan, dtype=float)
        padded[: arr3.shape[0]] = arr3
        return padded

    # Load all rows once; the slider will pick which row to display.
    u_opt_raw = _read_csv_if_exists(folder / "u_opt.csv")
    pred_states_raw = _read_csv_if_exists(folder / "x_pred.csv")
    x_ref_raw = _read_csv_if_exists(folder / "x_ref.csv")
    x0_raw = _read_csv_if_exists(folder / "x0.csv")  # optional (starting point marker)

    x_prev_raw = None
    u_prev_raw = None
    if show_prev:
        x_prev_raw = _read_csv_if_exists(folder / "x_prev.csv")
        u_prev_raw = _read_csv_if_exists(folder / "u_prev.csv")

    if all(v is None for v in (u_opt_raw, pred_states_raw, x_ref_raw, x0_raw, x_prev_raw, u_prev_raw)):
        raise FileNotFoundError(
            f"No CSV files found in {folder}. Expected any of: "
            "u_opt.csv, x_pred.csv, x_ref.csv, x0.csv, x_prev.csv, u_prev.csv"
        )

    # Slider max is based on how many rows exist in the main CSVs.
    # Optional overlays (x0/x_prev/u_prev) should not clamp the slider range.
    main_row_sources = [u_opt_raw, pred_states_raw, x_ref_raw]
    fallback_row_sources = [x_prev_raw, u_prev_raw]
    if any(v is not None for v in main_row_sources):
        row_sources = main_row_sources
    elif any(v is not None for v in fallback_row_sources):
        row_sources = fallback_row_sources
    else:
        row_sources = [x0_raw]
    n_rows = min(arr.shape[0] for arr in row_sources if arr is not None)
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

    # Pre-reshape into 3D arrays for quick slider updates (skip missing or malformed files).
    u_opt_all = _reshape_or_none(u_opt_raw[:n_rows] if u_opt_raw is not None else None, (n_rows, n_horizon, n_inputs), "u_opt.csv")
    pred_st_all = _reshape_or_none(pred_states_raw[:n_rows] if pred_states_raw is not None else None, (n_rows, n_horizon, n_states), "x_pred.csv")
    ref_st_all = _reshape_or_none(x_ref_raw[:n_rows] if x_ref_raw is not None else None, (n_rows, n_horizon, n_states), "x_ref.csv")
    x0_all = _prep_x0(x0_raw[:n_rows] if x0_raw is not None else None)
    prev_rows = min(n_rows, x_prev_raw.shape[0]) if x_prev_raw is not None else 0
    prev_st_all = _reshape_or_none(x_prev_raw[:prev_rows] if x_prev_raw is not None else None, (prev_rows, n_horizon, n_states), "x_prev.csv")
    prev_st_all = _pad_rows(prev_st_all, n_rows)

    prev_u_rows = min(n_rows, u_prev_raw.shape[0]) if u_prev_raw is not None else 0
    prev_u_all = _reshape_or_none(u_prev_raw[:prev_u_rows] if u_prev_raw is not None else None, (prev_u_rows, n_horizon, n_inputs), "u_prev.csv")
    prev_u_all = _pad_rows(prev_u_all, n_rows)

    has_state_data = any(v is not None for v in (ref_st_all, pred_st_all, prev_st_all, x0_all))
    has_input_data = any(v is not None for v in (u_opt_all, prev_u_all))
    if not has_state_data and not has_input_data:
        raise ValueError(
            "No plottable data found (files missing or incompatible shapes). "
            "Check n_horizon/n_states/n_inputs or the CSV contents."
        )

    state_offset = 0
    input_offset = (n_states if has_state_data else 0)
    total_plots = (n_states if has_state_data else 0) + (n_inputs if has_input_data else 0)
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

    # Plot each state on its own subplot (if any state-related CSV exists).
    if has_state_data:
        for i in range(n_states):
            state_name = state_names[i]
            ax = axes[state_offset + i]

            if ref_st_all is not None:
                (ref_line,) = ax.plot(horizon_range, ref_st_all[row, :, i], 'o-', label=f'Ref {state_name}', linewidth=2)
            else:
                ref_line = None
            ref_lines.append(ref_line)

            if x0_all is not None:
                (x0_dot,) = ax.plot([0], [x0_all[row, i]], 'o', color='red', label='x0', markersize=7, zorder=5)
            else:
                x0_dot = None
            x0_dots.append(x0_dot)

            if prev_st_all is not None:
                (prev_line,) = ax.plot(horizon_range, prev_st_all[row, :, i], 'x:', label=f'Prev {state_name}', linewidth=2)
            else:
                prev_line = None
            prev_state_lines.append(prev_line)

            if pred_st_all is not None:
                (pred_line,) = ax.plot(horizon_range, pred_st_all[row, :, i], 's--', label=f'Pred {state_name}', linewidth=2)
            else:
                pred_line = None
            pred_lines.append(pred_line)

            ax.set_ylabel(state_name)
            ax.set_title(f"{state_name}")
            if ax.lines:
                ax.legend()
            ax.grid(True, alpha=0.3)

    input_lines = []
    prev_input_lines = []

    # Plot each control action on its own subplot (if any input-related CSV exists).
    if has_input_data:
        for i in range(n_inputs):
            input_name = input_names[i]
            axis = axes[input_offset + i]

            if u_opt_all is not None:
                (u_line,) = axis.plot(horizon_range, u_opt_all[row, :, i], 'o-', label=f'u_opt {input_name}', linewidth=2)
            else:
                u_line = None
            input_lines.append(u_line)

            if prev_u_all is not None:
                (u_prev_line,) = axis.plot(horizon_range, prev_u_all[row, :, i], 'x:', label=f'u_prev {input_name}', linewidth=2)
            else:
                u_prev_line = None
            prev_input_lines.append(u_prev_line)

            axis.set_ylabel(input_name)
            axis.set_title(f"{input_name}")
            if axis.lines:
                axis.legend()
            axis.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Horizon Step")
    axes[-1].set_xlim(horizon_range[0], horizon_range[-1])

    for ax in axes:
        _autoscale_axis_y(ax)
    
    def _update_plot(r: int):
        fig.suptitle(f"CSV row: {r}/{n_rows - 1}", y=0.995)

        if has_state_data:
            for i in range(n_states):
                if ref_lines[i] is not None:
                    ref_lines[i].set_ydata(ref_st_all[r, :, i])
                if pred_lines[i] is not None:
                    pred_lines[i].set_ydata(pred_st_all[r, :, i])
                if x0_dots[i] is not None:
                    x0_dots[i].set_ydata([x0_all[r, i]])
                if prev_state_lines[i] is not None:
                    prev_state_lines[i].set_ydata(prev_st_all[r, :, i])

        if has_input_data:
            for i in range(n_inputs):
                if input_lines[i] is not None:
                    input_lines[i].set_ydata(u_opt_all[r, :, i])
                if prev_input_lines[i] is not None:
                    prev_input_lines[i].set_ydata(prev_u_all[r, :, i])

        for ax in axes:
            _autoscale_axis_y(ax)

        fig.canvas.draw_idle()

    # Slider: select which CSV row/iteration to visualize (skip if only 1 row).
    if n_rows > 1:
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

        def _on_slider(selected_row):
            _update_plot(int(selected_row))

        row_slider.on_changed(_on_slider)

    fig.tight_layout(rect=[0, 0.04, 1, 0.985])
    plt.show()

if __name__ == "__main__":
    visualize_mpc_debug(n_inputs=2, n_states=6, n_horizon=60)
