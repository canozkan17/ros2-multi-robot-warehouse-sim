#!/usr/bin/env python3
"""
generate_chapter8_plots.py
Generates high-resolution vector PDF plots for Chapter 8 of the PhD/MSc thesis.
Inputs: Empirical metrics from total_audit_time.csv & reallocation_latency.csv
Outputs: fig_8_1_speedup_efficiency.pdf, fig_8_2_ftl_boxplot.pdf, fig_8_3_battery_decay.pdf
"""

import matplotlib.pyplot as plt
import numpy as np

# Global Academic Styling Config
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 10,
    "axes.labelsize": 10,
    "axes.titlesize": 11,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 9,
    "figure.titlesize": 11,
    "text.usetex": False,
    "pdf.fonttype": 42,
    "ps.fonttype": 42
})

def plot_speedup_efficiency():
    """Generates Figure 8.1: Dual-axis Speedup and Parallel Efficiency vs Fleet Size (100% Zero Overlap)."""
    N = np.array([1, 2, 3])
    tat_mean = np.array([2443.72, 1229.78, 901.71])
    speedup = 2443.72 / tat_mean
    efficiency = (speedup / N) * 100.0
    ideal_speedup = N

    fig, ax1 = plt.subplots(figsize=(7.2, 4.5), dpi=300)

    # Padding on X axis
    ax1.set_xlim(0.3, 3.7)

    # Bar chart for Speedup (Left Axis)
    color_bar = '#1f77b4'
    bars = ax1.bar(N, speedup, width=0.32, color=color_bar, alpha=0.85, edgecolor='navy', linewidth=1.2, label=r'Measured Speedup ($S_N$)')
    line_ideal, = ax1.plot(N, ideal_speedup, 'k--', linewidth=1.8, marker='s', markersize=6, label='Ideal Linear Speedup')
    
    ax1.set_xlabel(r'Fleet Size ($N$)', fontweight='bold', labelpad=6)
    ax1.set_ylabel(r'Speedup Factor ($S_N$)', color='black', fontweight='bold')
    ax1.set_xticks(N)
    ax1.set_xticklabels([r'$N = 1$', r'$N = 2$', r'$N = 3$'])
    ax1.set_ylim(0, 4.5)  # Extended upper limit to open white space for legend
    ax1.grid(True, linestyle=':', alpha=0.5, zorder=0)

    # Bar value annotations: Centered INSIDE the bars in bold white text
    for bar in bars:
        height = bar.get_height()
        ax1.annotate(f'{height:.2f}' + r'$\times$',
                    xy=(bar.get_x() + bar.get_width() / 2.0, height / 2.0),
                    ha='center', va='center', fontsize=10, fontweight='bold', color='white')

    # Line chart for Efficiency (Right Axis)
    ax2 = ax1.twinx()
    color_line = '#d62728'
    line_eff, = ax2.plot(N, efficiency, color=color_line, marker='o', linewidth=2.2, markersize=8, label=r'Parallel Efficiency ($\eta_N$)')
    ax2.set_ylabel(r'Parallel Efficiency ($\eta_N$ %)', color=color_line, fontweight='bold')
    ax2.tick_params(axis='y', labelcolor=color_line)
    ax2.set_ylim(30, 130)  # Extended upper limit on right axis

    # Efficiency Value annotations: Placed cleanly ABOVE the markers with white background box
    for x, y in zip(N, efficiency):
        ax2.annotate(f'{y:.1f}%',
                    xy=(x, y),
                    xytext=(0, 12), textcoords="offset points",
                    ha='center', va='bottom', fontsize=9.5, fontweight='bold', color=color_line,
                    bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor=color_line, alpha=0.9))

    # Unified Legend: Placed INSIDE top-left empty space (100% isolated from Title)
    handles = [bars, line_ideal, line_eff]
    labels = [h.get_label() for h in handles]
    ax1.legend(handles, labels, loc='upper left', frameon=True, facecolor='white', framealpha=0.95, edgecolor='gray')

    plt.title('Fleet Scalability: Speedup Factor and Parallel Efficiency', pad=12, fontweight='bold')
    plt.tight_layout()
    plt.savefig('fig_8_1_speedup_efficiency.pdf', format='pdf', bbox_inches='tight')
    plt.close()
    print("[SUCCESS] Generated fig_8_1_speedup_efficiency.pdf")

def plot_ftl_distribution():
    """Generates Figure 8.2: Log-scale FTL Distribution and Outlier Isolation."""
    ftl_data = [
        [1.1, 477.3, 1.7],       # N=1 Pre-fail
        [2.8, 2.1, 5.8],         # N=2 Pre-fail
        [3.4, 2.2, 1.3],         # 25% Inj
        [1.0, 1.2, 0.9],         # 50% Inj
        [1.6, 215.5, 4.0]        # 75% Inj
    ]
    
    conditions = ['$N=1$\nPre-fail', '$N=2$\nPre-fail', '25% Inj.\nMid-Mission', '50% Inj.\nMid-Mission', '75% Inj.\nMid-Mission']

    fig, ax = plt.subplots(figsize=(7.2, 4.2), dpi=300)

    # Boxplot
    bp = ax.boxplot(ftl_data, patch_artist=True,
                    boxprops=dict(facecolor='#e377c2', alpha=0.4, color='#1f77b4'),
                    capprops=dict(color='#1f77b4', linewidth=1.2),
                    whiskerprops=dict(color='#1f77b4', linewidth=1.2),
                    flierprops=dict(marker='o', color='red', alpha=0.0),
                    medianprops=dict(color='#d62728', linewidth=2.0))

    # Set x-ticks explicitly
    ax.set_xticks([1, 2, 3, 4, 5])
    ax.set_xticklabels(conditions)

    # Scatter overlay for individual points (Jitter)
    np.random.seed(42)
    for i, pts in enumerate(ftl_data):
        x = np.random.normal(i + 1, 0.04, size=len(pts))
        ax.scatter(x, pts, color='#1f77b4', edgecolor='black', zorder=3, s=35, alpha=0.8)

    # Threshold lines
    ax.axhline(y=5000.0, color='red', linestyle='--', linewidth=1.5, label=r'Heartbeat Timeout Gate ($T_{\mathrm{timeout}} = 5.0\mathrm{s}$)')
    ax.axhline(y=6.0, color='green', linestyle=':', linewidth=1.2, label='Nominal Edge Execution Bound (6.0 ms)')

    ax.set_yscale('log')
    ax.set_ylabel('Failure-to-Reallocation Latency (ms) [Log Scale]', fontweight='bold')
    ax.set_ylim(0.1, 15000.0)
    ax.grid(True, which="both", linestyle=':', alpha=0.5)

    # Outlier Annotations
    ax.annotate('WSL2 Scheduler Outlier\n(477.3 ms)', xy=(1, 477.3), xytext=(1.35, 1200),
                arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=4),
                fontsize=8, ha='center')

    ax.annotate('WSL2 Scheduler Outlier\n(215.5 ms)', xy=(5, 215.5), xytext=(4.35, 800),
                arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=4),
                fontsize=8, ha='center')

    ax.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9)
    plt.title('Failure-to-Reallocation Latency (FTL) Distribution across Injection Regimes', fontweight='bold')
    plt.tight_layout()
    plt.savefig('fig_8_2_ftl_boxplot.pdf', format='pdf', bbox_inches='tight')
    plt.close()
    print("[SUCCESS] Generated fig_8_2_ftl_boxplot.pdf")

def plot_battery_decay_simulation():
    """Generates Figure 8.3: Hybrid Discharge Model State of Charge (SOC) vs Time."""
    lambda_idle = 0.02
    eta_dynamic = 0.15
    
    t_n1 = np.linspace(0, 2443.72, 500)
    v_n1 = 453.82 / 2443.72
    d_n1 = v_n1 * t_n1
    soc_n1_unconstrained = 100.0 - (lambda_idle * t_n1 + eta_dynamic * d_n1)
    
    t_n2 = np.linspace(0, 1229.78, 500)
    v_n2 = 231.5 / 1229.78
    d_n2 = v_n2 * t_n2
    soc_n2 = 100.0 - (lambda_idle * t_n2 + eta_dynamic * d_n2)

    t_n3 = np.linspace(0, 901.71, 500)
    v_n3 = 150.36 / 901.71
    d_n3 = v_n3 * t_n3
    soc_n3 = 100.0 - (lambda_idle * t_n3 + eta_dynamic * d_n3)

    fig, ax = plt.subplots(figsize=(7.0, 4.2), dpi=300)

    ax.plot(t_n1, soc_n1_unconstrained, color='#d62728', linestyle='-', linewidth=2.0, label=r'$N=1$ Agent (Theoretical Unconstrained)')
    ax.plot(t_n2, soc_n2, color='#1f77b4', linestyle='--', linewidth=2.0, label=r'$N=2$ Active Agent')
    ax.plot(t_n3, soc_n3, color='#2ca02c', linestyle='-.', linewidth=2.0, label=r'$N=3$ Active Agent')

    ax.axhline(y=0.0, color='black', linestyle='-', linewidth=1.5, label=r'Fail-Stop Depletion Boundary ($E_{\mathrm{fail}} = 0.0\%$)')

    depletion_time = 100.0 / (lambda_idle + eta_dynamic * v_n1)
    ax.axvline(x=depletion_time, color='#d62728', linestyle=':', alpha=0.7)
    ax.scatter([depletion_time], [0.0], color='#d62728', s=50, zorder=5)
    
    annot_text = f"Fail-Stop Trigger\n($t \\approx {depletion_time:.1f}" + r"\mathrm{s}$)"
    ax.annotate(annot_text,
                xy=(depletion_time, 0.0), xytext=(depletion_time - 420, 22),
                arrowprops=dict(facecolor='#d62728', shrink=0.05, width=1, headwidth=4),
                fontsize=8, color='#d62728', fontweight='bold')

    ax.set_xlabel('Mission Elapsed Time (s)', fontweight='bold')
    ax.set_ylabel('State of Charge (SOC %)', fontweight='bold')
    ax.set_xlim(0, 2600)
    ax.set_ylim(-15, 105)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9)

    plt.title('Hybrid Battery Model SOC Trajectories across Fleet Configurations', fontweight='bold')
    plt.tight_layout()
    plt.savefig('fig_8_3_battery_decay.pdf', format='pdf', bbox_inches='tight')
    plt.close()
    print("[SUCCESS] Generated fig_8_3_battery_decay.pdf")

if __name__ == "__main__":
    plot_speedup_efficiency()
    plot_ftl_distribution()
    plot_battery_decay_simulation()
    print("[COMPLETE] All Chapter 8 vector figures generated successfully.")
