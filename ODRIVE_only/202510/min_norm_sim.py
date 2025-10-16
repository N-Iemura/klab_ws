
#!/usr/bin/env python3
"""
min_norm_sim.py
----------------
Two-input one-output mechanism simulation with:
- Output-side PID (with derivative low-pass)
- Weighted minimum-norm torque split
- Nullspace damping & posture restore
- Joint dynamics (2x decoupled second-order, simple)
- Torque magnitude & rate limits
- Step reference for output angle

Usage:
    python min_norm_sim.py
Optionally edit the PARAMS dict inside the file.

Dependencies:
    numpy, matplotlib, pandas

Outputs:
    - Figures (PNG) and CSV saved under ./sim_outputs/<timestamp>/
"""
import math
import time
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime

# -------------------- Parameters --------------------
@dataclass
class Params:
    # Timing
    fs: float = 500.0             # Hz
    T: float = 20.0               # s

    # Mechanism mapping theta = a1*theta1 + a2*theta2
    a1: float = 1.0/20.0
    a2: float = 163.0/2000.0

    # Output PID
    Kp: float = 4.0
    Ki: float = 0.0
    Kd: float = 0.4
    # Derivative LPF cutoff [Hz]
    f_c_d: float = 10.0

    # Weighted min-norm weights (penalize motor2 more to protect it)
    w1: float = 1.0
    w2: float = 25.0

    # Nullspace damping / posture restore
    Knu_diag: tuple = (0.6, 0.6)  # viscous (Nms/rad-ish units in this toy model)
    Kq_diag:  tuple = (0.05, 0.05)
    q_ref: tuple = (0.0, 0.0)

    # Output torque shaping
    tau_max: float = 3.0          # Nm (magnitude limit)
    tau_rate: float = 15.0        # Nm/s (rate limit)

    # Joint dynamics
    J1: float = 0.01              # inertia
    J2: float = 0.005
    B1: float = 0.05              # viscous damping
    B2: float = 0.03

    # Output step profile
    step_amp: float = 0.3         # [turn]
    step_hold: float = 3.0        # seconds per plateau
    step_wait: float = 1.0        # initial wait

def theta_ref_signal(t: float, amp: float, hold: float, wait: float) -> float:
    if t < wait:
        return 0.0
    cyc = (t - wait) % (4.0*hold)
    if cyc < hold:
        return +amp
    elif cyc < 2.0*hold:
        return 0.0
    elif cyc < 3.0*hold:
        return -amp
    else:
        return 0.0

def weighted_min_norm_torque_split(A: np.ndarray, tau_out: float, w1: float, w2: float) -> np.ndarray:
    """tau* = W^{-1} A^T (A W^{-1} A^T)^{-1} tau_out"""
    W_inv = np.diag([1.0/w1, 1.0/w2])
    den = float(A @ W_inv @ A.T)
    if den < 1e-9: den = 1e-9
    return (W_inv @ A.T * (tau_out/den)).reshape(2)

def simulate(p: Params, save_dir: Path):
    dt = 1.0/p.fs
    N = int(p.T * p.fs)
    t = np.arange(N)*dt

    # Mechanism & matrices
    A = np.array([[p.a1, p.a2]])
    J = np.diag([p.J1, p.J2])
    B = np.diag([p.B1, p.B2])
    Knu = np.diag(p.Knu_diag)
    Kq  = np.diag(p.Kq_diag)
    q_ref = np.array(p.q_ref)

    # States
    q  = np.zeros(2)    # joint positions [turn]
    dq = np.zeros(2)    # joint velocities [turn/s]
    e_int = 0.0
    tau_prev = 0.0
    theta_dot_lp = 0.0
    alpha = (2.0*math.pi*p.f_c_d)/(2.0*math.pi*p.f_c_d + p.fs)

    # Logs
    logs = dict(
        time=t.copy(),
        theta=np.zeros(N),
        theta_ref=np.zeros(N),
        tau_out=np.zeros(N),
        tau1=np.zeros(N),
        tau2=np.zeros(N),
        q1=np.zeros(N),
        q2=np.zeros(N),
        dq1=np.zeros(N),
        dq2=np.zeros(N),
        z=np.zeros(N),           # nullspace coordinate a2*q1 - a1*q2
    )

    for k in range(N):
        th = float(A @ q.reshape(2,1))
        dth = float(A @ dq.reshape(2,1))
        logs['theta'][k] = th
        theta_ref = theta_ref_signal(t[k], p.step_amp, p.step_hold, p.step_wait)
        logs['theta_ref'][k] = theta_ref

        # PID on output (with derivative LPF)
        theta_dot_lp = (1.0-alpha)*theta_dot_lp + alpha*dth
        e = theta_ref - th
        e_int += e*dt
        de = -theta_dot_lp  # reference theta_dot is 0 for steps
        tau_out = p.Kp*e + p.Ki*e_int + p.Kd*de

        # Rate limit then magnitude limit
        tau_dot_cmd = (tau_out - tau_prev)/dt
        tau_dot_cmd = np.clip(tau_dot_cmd, -p.tau_rate, p.tau_rate)
        tau_out = tau_prev + tau_dot_cmd*dt
        tau_out = float(np.clip(tau_out, -p.tau_max, p.tau_max))
        tau_prev = tau_out
        logs['tau_out'][k] = tau_out

        # Weighted min-norm split
        tau_min = weighted_min_norm_torque_split(A, tau_out, p.w1, p.w2)

        # Nullspace projection P_N = I - A^T (A A^T)^{-1} A
        s = float(A @ A.T)
        invs = 1.0/max(s, 1e-9)
        PN = np.eye(2) - A.T * invs @ A
        tau_null = -Knu @ (PN @ dq) - Kq @ (PN @ (q - q_ref))

        tau = tau_min + tau_null

        # Optional joint torque saturation (asymmetric example)
        tau[0] = float(np.clip(tau[0], -6.0, 6.0))
        tau[1] = float(np.clip(tau[1], -0.2, 0.2))

        # Joint dynamics: J ddq + B dq = tau
        ddq = np.linalg.solve(J, (tau - B @ dq))
        dq = dq + ddq*dt
        q  = q + dq*dt

        # Log joints & nullspace
        logs['tau1'][k] = tau[0]
        logs['tau2'][k] = tau[1]
        logs['q1'][k] = q[0]
        logs['q2'][k] = q[1]
        logs['dq1'][k] = dq[0]
        logs['dq2'][k] = dq[1]
        logs['z'][k] = p.a2*q[0] - p.a1*q[1]

    # Save CSV
    df = pd.DataFrame(logs)
    save_dir.mkdir(parents=True, exist_ok=True)
    csv_path = save_dir / "simulation_log.csv"
    df.to_csv(csv_path, index=False)

    # Plots
    def plot_and_save(x, y, title, ylabel, fname, label1=None, label2=None):
        plt.figure(figsize=(8,4))
        ax = plt.gca()
        # 色分け: theta1/tau1=red, theta2/tau2=green, 他はデフォルト
        if label2 is None:
            plt.plot(x, y, label=label1 if label1 else None)
        else:
            color1 = 'red' if label1 in ['theta1', 'tau1'] else None
            color2 = 'green' if label2 in ['theta2', 'tau2'] else None
            plt.plot(x, y[0], label=label1 if label1 else None, color=color1)
            plt.plot(x, y[1], label=label2 if label2 else None, color=color2)
        plt.xlabel("Time [s]")
        plt.ylabel(ylabel)
        plt.title(title)
        if label1 or label2:
            plt.legend()
        # グリッド線なし、目盛り内向き
        plt.grid(False)
        ax.tick_params(axis='both', direction='in', length=6, width=1.2)
        plt.tight_layout()
        out = save_dir / fname
        plt.savefig(out, dpi=200, bbox_inches="tight")
        plt.close()
        return out

    p1 = plot_and_save(t, [df['theta_ref'], df['theta']], "Output Angle Tracking", "Theta [turn]",
                       "fig_theta.png", "theta_ref", "theta")
    p2 = plot_and_save(t, df['tau_out'], "Output Torque Command", "Torque [Nm]",
                       "fig_tau_out.png", "tau_out*")
    p3 = plot_and_save(t, [df['tau1'], df['tau2']], "Joint Torques", "Torque [Nm]",
                       "fig_joint_torque.png", "tau1", "tau2")
    p4 = plot_and_save(t, [df['q1'], df['q2']], "Joint Positions", "Joint Angle [turn]",
                       "fig_joint_pos.png", "theta1", "theta2")
    p5 = plot_and_save(t, df['z'], "Nullspace Behavior", "Nullspace Coord [turn]",
                       "fig_nullspace.png", "z")

    return dict(csv=str(csv_path), figs=[str(p1), str(p2), str(p3), str(p4), str(p5)])

def main():
    p = Params()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = Path("sim_outputs")/timestamp
    results = simulate(p, outdir)
    print("Saved CSV:", results['csv'])
    print("Saved FIGs:")
    for f in results['figs']:
        print(" -", f)

if __name__ == "__main__":
    main()
