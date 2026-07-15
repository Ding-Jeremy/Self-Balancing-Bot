import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def save_values(
    df,
    x_col,
    y_col,
    filename="plot.pdf",
    label=None,
    title=None,
    ylabel=None,
):
    plt.figure(figsize=(6, 4))

    plt.plot(
        df.iloc[:, x_col],
        df.iloc[:, y_col],
        label=label if label else df.columns[y_col],
    )

    plt.xlabel(df.columns[x_col])
    plt.ylabel(ylabel if ylabel else df.columns[y_col])
    plt.title(title if title else f"{df.columns[y_col]} vs {df.columns[x_col]}")
    plt.grid(True)
    plt.legend()

    plt.savefig(filename, dpi=400, bbox_inches="tight")
    plt.close()


df = pd.read_csv("side_hit_neg.csv")

# Plot column 4 vs column 0
save_values(
    df,
    x_col=0,
    y_col=4,
    filename="graphs/side_hit_neg.pdf",
    label=r"$\theta$",
    title=r"$\theta = f(t)$",
    ylabel=r"$\theta[°]$",
)

df = pd.read_csv("side_hit_pos.csv")

# Plot column 4 vs column 0
save_values(
    df,
    x_col=0,
    y_col=4,
    filename="graphs/side_hit_pos.pdf",
    label=r"$\theta$",
    title=r"$\theta = f(t)$",
    ylabel=r"$\theta[°]$",
)

df = pd.read_csv("trapezoidal_move.csv")

# Plot column 4 vs column 0
save_values(
    df,
    x_col=0,
    y_col=[1, 2],
    filename="graphs/trapezoidal_move_speed.pdf",
    title=r"Consigne de vitesse: Trapèze",
    ylabel=r"$[m/s]$",
)


df = pd.read_csv("maintient_choc.csv")

plt.figure(figsize=(6, 4))
plt.xlabel(df.columns[0])
plt.ylabel("vitesse")
plt.plot(
    df.iloc[:, 0],
    df.iloc[:, [1, 2]],
    label=df.columns[[1, 2]],
)
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(6, 4))
plt.xlabel(df.columns[0])
plt.ylabel("vitesse")
plt.plot(
    df.iloc[:, 0],
    df.iloc[:, [3, 4]],
    label=df.columns[[3, 4]],
)
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(6, 4))
plt.xlabel(df.columns[0])
plt.ylabel("vitesse")
plt.plot(
    df.iloc[:, 0],
    df.iloc[:, 5],
    label=df.columns[5],
)
plt.legend()
plt.grid(True)
plt.show()
"""
time = df["Time [ms]"].to_numpy()

Ts = np.mean(np.diff(time)) / 1000
angle = df["Theta [deg]"].to_numpy()
angle = angle - np.mean(angle)

N = len(angle)

freq = np.fft.fftshift(np.fft.fftfreq(N, Ts))

X = np.fft.fftshift(np.fft.fft(angle))

plt.figure()
plt.plot(freq, np.abs(X))
plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude")
plt.grid(True)
plt.show()
"""
