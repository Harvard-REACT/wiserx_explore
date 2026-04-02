#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
import warnings
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import matplotlib.patches as mpatches
from typing import List, Optional, Dict, Tuple

# -----------------------------
# I/O helpers (unchanged)
# -----------------------------
def find_one_csv(dir_path: Path, pattern: str) -> Path:
    matches = sorted(dir_path.glob(pattern))
    if not matches:
        raise FileNotFoundError(f"No file matching '{pattern}' in {dir_path}")
    if len(matches) > 1:
        matches.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return matches[0]

def load_pair_as_df(dir_path: Path, cov_pattern: str, det_pattern: str) -> pd.DataFrame:
    cov_csv = find_one_csv(dir_path, cov_pattern)
    det_csv = find_one_csv(dir_path, det_pattern)

    df_cov = pd.read_csv(cov_csv)
    df_det = pd.read_csv(det_csv)

    if 'coverage_overlap_percent' not in df_cov.columns:
        raise ValueError(f"{cov_csv} does not contain 'coverage_overlap_percent'")

    cov_val = df_cov['coverage_overlap_percent']
    if cov_val.size == 1:
        df_det['coverage_overlap_percent'] = cov_val.iloc[0]
    else:
        if len(df_cov) == len(df_det):
            df_det['coverage_overlap_percent'] = cov_val.values
        else:
            df_det['coverage_overlap_percent'] = cov_val.mean()
            print(f"[WARN] Length mismatch between coverage and details in {dir_path}, using mean.")

    # Prioritize 'merged_coverage_percent' from the coverage file if it exists,
    # as it's more likely to be the correct source for this metric.
    if 'merged_coverage_percent' in df_cov.columns:
        if 'merged_coverage_percent' not in df_det.columns:
            print(f"[INFO] Found 'merged_coverage_percent' in {cov_csv.name}, adding to details.")
        else:
            print(f"[INFO] Found 'merged_coverage_percent' in {cov_csv.name}, using it to overwrite column from details file.")
        mcp_val = df_cov['merged_coverage_percent']
        if mcp_val.size == 1:
            df_det['merged_coverage_percent'] = mcp_val.iloc[0]
        else:
            if len(df_cov) == len(df_det):
                df_det['merged_coverage_percent'] = mcp_val.values
            else:
                df_det['merged_coverage_percent'] = mcp_val.mean()
                print(f"[WARN] Length mismatch for 'merged_coverage_percent' in {dir_path}, using mean.")

    return df_det

# -----------------------------
# Static plotters (unchanged)
# -----------------------------
def make_lineplot_ci(df: pd.DataFrame, x: str, y: str, out_path: Path,
                     title: str, x_label: str, y_label: str):
    if x not in df.columns or y not in df.columns:
        print(f"[WARN] Missing columns for plot: {x} or {y}. Skipping {out_path.name}", file=sys.stderr)
        return
    df_plot = df.dropna(subset=[x, y]).sort_values(by=x)

    # Compute mean & 100% CI using normal approx (fast & version-proof)
    grp = df_plot.groupby(x)[y]
    mean = grp.mean()
    std = grp.std(ddof=1)
    n = grp.count().clip(lower=1)
    se = (std / np.sqrt(n)).fillna(0)
    ci = 1.96 * se

    xs = mean.index.values
    mu = mean.values
    lo = (mean - ci).values
    hi = (mean + ci).values

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(xs, mu, lw=2)
    ax.fill_between(xs, lo, hi, alpha=0.2)
    ax.set_xlabel(x_label, fontsize=16)
    ax.set_ylabel(y_label, fontsize=16)
    ax.set_ylim(0, 105)
    ax.tick_params(labelsize=14)
    fig.tight_layout()
    fig.savefig(out_path, dpi=600)
    plt.close(fig)

def make_multi_lineplot_ci(df: pd.DataFrame, x: str, y_cols: List[str],
                           out_path: Path, title: str, x_label: str, y_label: str) -> None:
    missing_cols = [col for col in [x, *y_cols] if col not in df.columns]
    if missing_cols:
        print(f"[WARN] Missing columns: {missing_cols}. Skipping {out_path.name}")
        return
    if "merged_coverage_percent" in df.columns:
        df = df[df["merged_coverage_percent"] <= 100]

    fig, ax = plt.subplots(figsize=(18, 8))
    label_map = {
        'r_1_per': 'Robot_1',
        'r_2_per': 'Robot_2',
        'r_3_per': 'Robot_3',
        'merged_coverage_percent': 'Merged Coverage'
    }

    for label in y_cols:
        sub = df.dropna(subset=[x, label]).sort_values(by=x)
        grp = sub.groupby(x)[label]
        mean = grp.mean()
        std = grp.std(ddof=1)
        n = grp.count().clip(lower=1)
        se = (std / np.sqrt(n)).fillna(0)
        ci = 1.96 * se

        xs = mean.index.values
        mu = mean.values
        lo = (mean - ci).values
        hi = (mean + ci).values

        display_label = label_map.get(label, label)
        plot_kwargs = {'lw': 2, 'label': display_label}
        if label == 'merged_coverage_percent':
            plot_kwargs['color'] = 'red'

        line = ax.plot(xs, mu, **plot_kwargs)
        # Use the line's color for the fill
        ax.fill_between(xs, lo, hi, alpha=0.15, color=line[0].get_color())

    # Vertical lines for r2_stop_time == -1
    if "r2_stop_time" in df.columns:
        stop_points = df.loc[df["r2_stop_time"] == -1, x].dropna().unique()
        for stop_x in stop_points:
            ax.axvline(float(stop_x), color="red", linestyle="--", alpha=0.7)

    ax.set_xlabel(x_label, fontsize=32)
    ax.set_ylabel(y_label, fontsize=32)
    ax.set_ylim(0, 105)
    ax.tick_params(labelsize=32)
    ax.legend(fontsize=32)
    fig.tight_layout()
    fig.savefig(out_path, dpi=600)
    plt.close(fig)

def make_coverage_comparison_plot(df: pd.DataFrame, out_path: Path):
    """
    Generates a plot comparing merged coverage vs. overlap for different robot counts.
    """
    fig, ax = plt.subplots(figsize=(10, 14))

    x = "merged_coverage_percent_ceil"
    y = "coverage_overlap_percent"

    if x not in df.columns or y not in df.columns:
        print(f"[WARN] Missing columns for comparison plot: {x} or {y}. Skipping.", file=sys.stderr)
        return

    robot_counts = sorted(df['num_robots'].unique())

    for count in robot_counts:
        df_group = df[df['num_robots'] == count]

        # Reusing logic from make_lineplot_ci
        df_plot = df_group.dropna(subset=[x, y]).sort_values(by=x)
        grp = df_plot.groupby(x)[y]
        mean = grp.mean()
        std = grp.std(ddof=1)
        n = grp.count().clip(lower=1)
        se = (std / np.sqrt(n)).fillna(0)
        ci = 1.96 * se

        xs = mean.index.values
        mu = mean.values
        lo = (mean - ci).values
        hi = (mean + ci).values

        line = ax.plot(xs, mu, lw=2, label=f'{int(count)} Robots')
        ax.fill_between(xs, lo, hi, alpha=0.2, color=line[0].get_color())

    ax.set_xlabel("Total Map Coverage (%)", fontsize=32)
    ax.set_ylabel("Coverage Overlap (%)", fontsize=32)
    ax.set_ylim(0, 105)
    ax.tick_params(labelsize=32)
    ax.legend(fontsize=36)
    fig.tight_layout()
    fig.savefig(out_path, dpi=600)
    plt.close(fig)

def make_individual_robot_coverage_plot(df: pd.DataFrame, out_path: Path):
    """
    Generates a single plot showing coverage vs. time for each individual robot
    from each trial, with annotations for termination and coverage statistics.
    """
    fig, ax = plt.subplots(figsize=(24, 8))
 
    robot_cols = {
        'r_1_per': ('r1_stop_time', 'Robot 1', 'C0'),
        'r_2_per': ('r2_stop_time', 'Robot 2', 'C1'),
        'r_3_per': ('r3_stop_time', 'Robot 3', 'C2'),
    }

    if 'trial_id' not in df.columns:
        print("[WARN] 'trial_id' column not found. Cannot generate individual robot coverage plot.",
              file=sys.stderr)
        return

    legend_handles = {}
    num_trials = df['trial_id'].nunique()
    all_termination_times = []
    final_coverages = []
 
    for trial_idx in range(num_trials):
        trial_df = df[df['trial_id'] == trial_idx]

        for r_col, (stop_col, label, color) in robot_cols.items():
            if r_col not in trial_df.columns or stop_col not in trial_df.columns:
                continue

            stop_time_series = trial_df[trial_df[stop_col] > 0][stop_col]
            stop_time = stop_time_series.iloc[0] if not stop_time_series.empty else None

            plot_df = trial_df[['time_elapsed_sec', r_col]].dropna().sort_values(by='time_elapsed_sec')

            if stop_time is not None:
                all_termination_times.append(stop_time)
                plot_df = plot_df[plot_df['time_elapsed_sec'] <= stop_time]

            if not plot_df.empty:
                final_coverages.append(plot_df[r_col].iloc[-1])

            line, = ax.plot(plot_df['time_elapsed_sec'], plot_df[r_col], color=color, alpha=0.6)

            if label not in legend_handles:
                legend_handles[label] = line

    num_robots_str = f"{int(df['num_robots'].iloc[0])} Robots" if 'num_robots' in df.columns and not df.empty else "All Trials"
    ax.set_xlabel("Time Elapsed (seconds)", fontsize=32)
    ax.set_ylabel("Individual Map Coverage (%)", fontsize=32)
    ax.tick_params(labelsize=32)
    ax.set_ylim(0, 105)

    stats_text = ""
    if all_termination_times:
        mean_term_time = np.mean(all_termination_times)
        std_term_time = np.std(all_termination_times)
        stats_text += (f"Termination Time:\n  Mean: {mean_term_time:.1f} s, Std Dev: {std_term_time:.1f} s\n")

        # Annotate time statistics visually on the plot
        mean_line = ax.axvline(float(mean_term_time), color='k', linestyle='--', label='Mean Term. Time')
        std_region_start = mean_term_time - std_term_time
        std_region_end = mean_term_time + std_term_time
        ax.axvspan(std_region_start, std_region_end, alpha=0.2, color='gray')

        # Add handles for the legend
        std_patch = mpatches.Patch(color='gray', alpha=0.2, label='Std Dev Term. Time')
        legend_handles['Mean Term. Time'] = mean_line
        legend_handles['Std Dev Term. Time'] = std_patch

    if final_coverages:
        mean_final_cov = np.mean(final_coverages)
        std_final_cov = np.std(final_coverages)
        stats_text += (f"\nFinal Coverage:\n  Mean: {mean_final_cov:.1f} %, Std Dev: {std_final_cov:.1f} %")

    # Increase axis tick sizes
    ax.tick_params(axis='both', which='major', labelsize=32)

    # Place legend in the top-left corner with a larger font
    ax.legend(legend_handles.values(), legend_handles.keys(), loc='upper left', fontsize=24)

    if stats_text:
        # Place statistics beside legend with a larger font and padding
        ax.text(0.25, 0.95, stats_text, transform=ax.transAxes, fontsize=24,
                verticalalignment='top', horizontalalignment='left',
                bbox=dict(boxstyle='round,pad=0.5', fc='wheat', alpha=0.7))

    fig.tight_layout()
    fig.savefig(out_path, dpi=600)
    plt.close(fig)

def make_termination_time_cdf_plot(df: pd.DataFrame, out_path: Path):
    """Generates a CDF plot of robot termination times across all trials."""
    if 'trial_id' not in df.columns:
        print("[WARN] 'trial_id' column not found. Cannot generate termination time CDF plot.", file=sys.stderr)
        return

    stop_time_cols = sorted([c for c in df.columns if c.startswith('r') and c.endswith('_stop_time')])

    all_termination_times = []
    num_trials = df['trial_id'].nunique()

    for trial_idx in range(num_trials):
        trial_df = df[df['trial_id'] == trial_idx]
        for stop_col in stop_time_cols:
            if stop_col in trial_df.columns:
                # Get the single unique stop time > 0 for this robot in this trial
                stop_time_series = trial_df[trial_df[stop_col] > 0][stop_col]
                if not stop_time_series.empty:
                    all_termination_times.append(stop_time_series.iloc[0])

    if not all_termination_times:
        print(f"[WARN] No termination times found in {len(df)} rows. Skipping CDF plot.", file=sys.stderr)
        return

    sorted_times = np.sort(all_termination_times)
    # y-values for CDF go from 1/N to 1
    y_cdf = np.arange(1, len(sorted_times) + 1) / len(sorted_times)

    fig, ax = plt.subplots(figsize=(8, 6))
    # Use a step plot for the empirical CDF
    ax.plot(sorted_times, y_cdf, drawstyle='steps-post', marker='o', markersize=4, linestyle='-')
    
    ax.set_xlabel("Termination Time (seconds)")
    ax.set_ylabel("Cumulative Probability")
    ax.set_xlim(left=0)
    ax.set_ylim(0, 1.05)
    ax.grid(True, linestyle='--', alpha=0.6)
    
    fig.tight_layout()
    fig.savefig(out_path, dpi=600)
    plt.close(fig)

# -----------------------------
# Animation helpers (NEW)
# -----------------------------
def _aggregate_xy(df: pd.DataFrame, x: str, y: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Aggregate by x with mean and normal-approx 100% CI."""
    sub = df.dropna(subset=[x, y]).sort_values(by=x)
    grp = sub.groupby(x)[y]
    mean = grp.mean()
    std = grp.std(ddof=1)
    n = grp.count().clip(lower=1)
    se = (std / np.sqrt(n)).fillna(0)
    ci = 1.96 * se
    xs = mean.index.values
    mu = mean.values
    lo = (mean - ci).values
    hi = (mean + ci).values
    return xs, mu, lo, hi

def _pick_writer(anim_format: str):
    """Return (writer, ext) tuple."""
    anim_format = anim_format.lower()
    if anim_format == "mp4":
        try:
            return animation.FFMpegWriter(fps=20), ".mp4"
        except Exception:
            warnings.warn("FFMpeg not available; falling back to GIF.")
            return animation.PillowWriter(fps=20), ".gif"
    elif anim_format == "gif":
        return animation.PillowWriter(fps=20), ".gif"
    else:
        return animation.PillowWriter(fps=20), ".gif"

def animate_line_with_ci(df, x, y, out_path, title, x_label, y_label,
                         frames=None, fps=20, anim_format="gif"):
    xs, mu, lo, hi = _aggregate_xy(df, x, y)
    if xs.size == 0:
        print(f"[WARN] No data for animation: {x} vs {y}")
        return

    N = len(xs)
    frames = frames or N

    fig, ax = plt.subplots(figsize=(8, 5))
    (line,) = ax.plot([], [], lw=2)
    ci_fill = None  # keep a handle to the CI PolyCollection
    ax.set_xlim(xs.min(), xs.max())
    ax.set_ylim(0, 105)
    ax.set_xlabel(x_label, fontsize=16)
    ax.set_ylabel(y_label, fontsize=16)
    ax.tick_params(labelsize=14)
    fig.tight_layout()

    def init():
        nonlocal ci_fill
        line.set_data([], [])
        if ci_fill is not None:
            # remove any existing fill
            try:
                ci_fill.remove()
            except Exception:
                pass
            ci_fill = None
        return (line,)

    def update(f):
        nonlocal ci_fill
        k = max(1, int(np.ceil((f / max(frames - 1, 1)) * N)))
        line.set_data(xs[:k], mu[:k])
        # rebuild CI fill so it grows with the line
        if ci_fill is not None:
            try:
                ci_fill.remove()
            except Exception:
                pass
        ci_fill = ax.fill_between(xs[:k], lo[:k], hi[:k], alpha=0.2)
        return (line,)

    anim = animation.FuncAnimation(fig, update, init_func=init, frames=frames,
                                   blit=False, interval=1050/fps)

    writer, ext = _pick_writer(anim_format)
    save_path = out_path.with_suffix(ext)
    anim.save(save_path, writer=writer)
    plt.close(fig)
    print(f"[INFO] Animated plot saved to {save_path.resolve()}")


def animate_multi_line_with_ci(df, x, y_cols, out_path, title, x_label, y_label,
                               frames=None, fps=20, anim_format="gif"):
    agg = {}
    for label in y_cols:
        if label in df.columns:
            xs, mu, lo, hi = _aggregate_xy(df, x, label)
            if xs.size:
                agg[label] = (xs, mu, lo, hi)

    if not agg:
        print("[WARN] Nothing to animate for multi-line plot.")
        return

    all_x = np.concatenate([v[0] for v in agg.values()])
    xmin, xmax = np.min(all_x), np.max(all_x)
    Nmax = max(len(v[0]) for v in agg.values())
    frames = frames or Nmax

    fig, ax = plt.subplots(figsize=(8, 5))
    lines = {}
    fills = {}  # label -> PolyCollection (or None)
    for label in agg.keys():
        (ln,) = ax.plot([], [], lw=2, label=label)
        lines[label] = ln
        fills[label] = None

    if "r2_stop_time" in df.columns:
        stops = df.loc[df["r2_stop_time"] == -1, x].dropna().unique()
        for sx in stops:
            ax.axvline(float(sx), color="red", linestyle="--", alpha=0.7)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(0, 105)
    ax.set_xlabel(x_label, fontsize=16)
    ax.set_ylabel(y_label, fontsize=16)
    ax.tick_params(labelsize=14)
    ax.legend()
    fig.tight_layout()

    def init():
        for ln in lines.values():
            ln.set_data([], [])
        # remove any existing fills explicitly
        for lab, poly in list(fills.items()):
            if poly is not None:
                try:
                    poly.remove()
                except Exception:
                    pass
                fills[lab] = None
        return tuple(lines.values())

    def update(f):
        frac = f / max(frames - 1, 1)
        for label, (xs, mu, lo, hi) in agg.items():
            k = max(1, int(np.ceil(frac * len(xs))))
            lines[label].set_data(xs[:k], mu[:k])
            # refresh CI fill for this series
            if fills[label] is not None:
                try:
                    fills[label].remove()
                except Exception:
                    pass
            fills[label] = ax.fill_between(xs[:k], lo[:k], hi[:k], alpha=0.15)
        return tuple(lines.values())

    anim = animation.FuncAnimation(fig, update, init_func=init, frames=frames,
                                   blit=False, interval=1050/fps)

    writer, ext = _pick_writer(anim_format)
    save_path = out_path.with_suffix(ext)
    anim.save(save_path, writer=writer)
    plt.close(fig)
    print(f"[INFO] Animated multi-line plot saved to {save_path.resolve()}")


# -----------------------------
# CLI
# -----------------------------
def main():
    parser = argparse.ArgumentParser(description="Aggregate CSVs from directories and plot relationships.")
    parser.add_argument("--dirs2", nargs="*", default=[], help="Directories for 2-robot trials")
    parser.add_argument("--dirs3", nargs="*", default=[], help="Directories for 3-robot trials")
    parser.add_argument("--coverage-pattern", default="coverage_overlap_*.csv",
                        help="Glob for coverage/overlap CSV (default: coverage_overlap_*.csv)")
    parser.add_argument("--details-pattern", default="details_*.csv",
                        help="Glob for details CSV (default: details_*.csv)")
    parser.add_argument("--out", default="aggregate_output", help="Output directory")
    parser.add_argument("--compare-coverage", action="store_true",
                        help="Generate a plot comparing coverage overlap for 2 vs 3 robots.")

    # Animation flags
    parser.add_argument("--animate", action="store_true", help="Generate animated plots instead of/static in addition to PNGs")
    parser.add_argument("--anim-format", choices=["gif", "mp4"], default="gif", help="Animation output format")
    parser.add_argument("--fps", type=int, default=20, help="Frames per second for animations")
    parser.add_argument("--frames", type=int, default=None, help="Total frames (defaults to number of x-steps)")

    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    per_dir_dfs = []
    trial_counter = 0

    dir_groups = {
        2: args.dirs2,
        3: args.dirs3,
    }

    for num_robots, dirs in dir_groups.items():
        for d in dirs:
            dir_path = Path(d)
            if not dir_path.is_dir():
                print(f"[WARN] Skipping non-directory: {dir_path}", file=sys.stderr)
                continue
            try:
                df_pair = load_pair_as_df(dir_path, args.coverage_pattern, args.details_pattern)
                df_pair['trial_id'] = trial_counter
                df_pair['num_robots'] = num_robots
                per_dir_dfs.append(df_pair)
                print(f"[INFO] Loaded {num_robots}-robot data from {dir_path}: {df_pair.shape[0]} rows")
                trial_counter += 1
            except (FileNotFoundError, ValueError) as e:
                print(f"[WARN] {e}", file=sys.stderr)

    if not per_dir_dfs:
        print("[ERROR] No data loaded. Check your directories and file patterns.", file=sys.stderr)
        sys.exit(1)

    combined = pd.concat(per_dir_dfs, axis=0, ignore_index=True, sort=False)
    if "merged_coverage_percent" in combined.columns:
        combined = combined[combined["merged_coverage_percent"] <= 105]
    if "time_elapsed_sec" in combined.columns:
        combined['time_elapsed_sec'] = np.floor(combined['time_elapsed_sec'])
    for col in ["merged_coverage_percent", "r_1_per", "r_2_per", "r_3_per"]:
        if col in combined.columns:
            combined[col + ('_ceil' if col == "merged_coverage_percent" else '')] = np.ceil(combined[col])
    for col in ["r1_stop_time", "r2_stop_time", "r3_stop_time"]:
        if col in combined.columns:
            combined[col] = combined[col].fillna(0)

    print(combined.head())

    # Split data for separate plotting
    df_2_robots = combined[combined['num_robots'] == 2]
    df_3_robots = combined[combined['num_robots'] == 3]

    # --- Generate plots for each robot count group ---
    for num, df_group in [(2, df_2_robots), (3, df_3_robots)]:
        if df_group.empty:
            continue

        print(f"\n[INFO] Generating plots for {num}-robot trials...")

        # Plot 1: Merged Coverage vs Overlap
        plot1 = out_dir / f"merged_coverage_vs_overlap_{num}_robots.png"
        make_lineplot_ci(
            df_group,
            x="merged_coverage_percent_ceil",
            y="coverage_overlap_percent",
            out_path=plot1,
            title=f"Coverage vs. Overlap ({num} Robots)",
            x_label="Total Map Coverage (%)",
            y_label="Coverage Overlap (%)"
        )

        # Plot 2: Time vs Merged Coverage
        plot2 = out_dir / f"time_vs_merged_coverage_{num}_robots.png"
        make_lineplot_ci(
            df_group,
            x="time_elapsed_sec",
            y="merged_coverage_percent",
            out_path=plot2,
            title=f"Time vs. Merged Coverage ({num} Robots)",
            x_label="Time Elapsed (seconds)",
            y_label="Total Map Coverage (%)"
        )

        # Plot 3: Time vs All Coverages
        plot3 = out_dir / f"time_vs_all_coverages_{num}_robots.png"
        y_cols = sorted([c for c in df_group.columns if c.startswith('r') and c.endswith('_per')])
        y_cols.append("merged_coverage_percent")
        make_multi_lineplot_ci(
            df_group,
            x="time_elapsed_sec",
            y_cols=y_cols,
            out_path=plot3,
            title=f"Time vs. Coverage ({num} Robots)",
            x_label="Time Elapsed (seconds)",
            y_label="Coverage (%)"
        )

        # Plot 4: Individual Robot Coverage
        plot4 = out_dir / f"individual_robot_coverage_{num}_robots.png"
        make_individual_robot_coverage_plot(df_group, plot4)

        # Plot 5: Termination Time CDF
        plot5 = out_dir / f"termination_time_cdf_{num}_robots.png"
        make_termination_time_cdf_plot(df_group, plot5)

    # --- Generate comparison plot if requested ---
    if args.compare_coverage:
        if not df_2_robots.empty and not df_3_robots.empty:
            print("\n[INFO] Generating comparison plot...")
            plot_compare = out_dir / "comparison_coverage_overlap.png"
            make_coverage_comparison_plot(combined, plot_compare)
        else:
            print("[WARN] Cannot generate comparison plot, data for both 2 and 3 robots is required.")

    # Animated outputs
    if args.animate:
        print("\n[INFO] Generating animations...")

        for num, df_group in [(2, df_2_robots), (3, df_3_robots)]:
            if df_group.empty:
                continue

            print(f"\n[INFO] Generating animations for {num}-robot trials...")

            print(f"\n[INFO] Merged Coverage vs Overlap animation for {num} robots...")
            try:
                animate_line_with_ci(
                    df_group,
                    x="merged_coverage_percent_ceil",
                    y="coverage_overlap_percent",
                    out_path=out_dir / f"merged_coverage_vs_overlap_{num}_robots_anim",
                    title=f"Merged Coverage vs Overlap ({num} Robots, animated)",
                    x_label="Total Map Coverage (%)",
                    y_label="Coverage Overlap (%)",
                    frames=args.frames,
                    fps=args.fps,
                    anim_format=args.anim_format
                )
            except Exception as e:
                print(f"[ERROR] merged_coverage_vs_overlap_{num}_robots_anim failed: {e}", file=sys.stderr)


            print(f"\n[INFO] Time vs Merged Coverage animation for {num} robots...")
            try:
                y_cols_anim = sorted([c for c in df_group.columns if c.startswith('r') and c.endswith('_per')])
                y_cols_anim.append("merged_coverage_percent")
                animate_multi_line_with_ci(
                    df_group,
                    x="time_elapsed_sec",
                    y_cols=y_cols_anim,
                    out_path=out_dir / f"time_vs_all_coverages_{num}_robots_anim",
                    title=f"Time vs All Coverages ({num} Robots, animated)",
                    x_label="Time Elapsed (seconds)",
                    y_label="Coverage (%)",
                    frames=args.frames,
                    fps=args.fps,
                    anim_format=args.anim_format
                )
            except Exception as e:
                print(f"[ERROR] time_vs_all_coverages_{num}_robots_anim failed: {e}", file=sys.stderr)

    print(f"[INFO] Outputs in: {out_dir.resolve()}")

if __name__ == "__main__":
    main()
