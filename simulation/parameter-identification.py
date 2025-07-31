import numpy as np
import matplotlib.pyplot as plt

def simulate_force_control(Kv, steps=1000):
    # Constants and initial settings
    Fcmd = 0.0  # Force command
    Ke = 1000.0  # Environment stiffness
    De = 50.0  # Environment damping
    Xe = 0.01  # Environment position
    Mv = 1.9  # Manipulator mass
    sample = 0.001  # Sampling time
    mu = 0.99  # Forgetting factor

    # Initial values
    Fres = 0.0
    Xp = 0.01
    Xv = 0.0
    Xa = 0.0
    alpha = np.array([20000.0, 400.0, 5.0])  # Parameter initial values
    Q = np.diag([1.0e6, 1.0e6, 1.0e6])  # Initial Q matrix

    # Data storage
    graph_Fres = []
    graph_Fcmd = []

    # Simulation loop
    for step in range(steps):
        # Force control
        if step >= 200: # Change the force command after 200 steps
            Fcmd = 1
        Xa = (Fcmd - Fres - Kv * Xv) / Mv
        Xv += Xa * sample
        Xp += Xv * sample
        if Xp >= Xe:
            Fres = Ke * (Xp - Xe) + De * Xv

        # Parameter identification
        m = np.array([Xp, Xv, 1])
        E = Fres - np.dot(alpha, m)
        deno = mu + np.dot(m.T, np.dot(Q, m))
        K = np.dot(Q, m) / deno
        alpha += K * E
        Q = (Q - np.outer(K, np.dot(m.T, Q))) / mu

        # Data storage
        graph_Fres.append(Fres)
        graph_Fcmd.append(Fcmd)

    return graph_Fres, graph_Fcmd

# Run simulations for Kv=10 and Kv=50
Fres_Kv10, Fcmd = simulate_force_control(Kv=10)
Fres_Kv50, Fcmd = simulate_force_control(Kv=50)

time_steps = np.arange(len(Fres_Kv10)) * 0.001

# Set font to Times New Roman
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["font.size"] = 14

# Plotting the results
plt.figure(figsize=(12, 6))
plt.plot(time_steps, Fres_Kv10, label="F_res : Kv=10", linestyle='-')
plt.plot(time_steps, Fres_Kv50, label="F_res : Kv=50", linestyle='-')
plt.plot(time_steps, Fcmd, label="F_cmd", linestyle='--', alpha=0.5)
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.xlim(0, 1)
plt.show()
