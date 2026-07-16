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


df = pd.read_csv("hold_nofilt.csv")

# Plot column 4 vs column 0
save_values(
    df,
    x_col=0,
    y_col=[2, 3],
    filename="graphs/hold_nofilt.pdf",
    title="Régulation de maintient",
    ylabel=r"$v [m/s]$",
)

df = pd.read_csv("trapezoidal_move_pos_nofilt.csv")

# Plot column 4 vs column 0
save_values(
    df,
    x_col=0,
    y_col=[2, 3],
    filename="graphs/trapezoidal_move_pos_nofilt_speeds.pdf",
    title=r"Consigne de vitesse: Trapèze",
    ylabel=r"$[m/s]$",
)

save_values(
    df,
    x_col=0,
    y_col=6,
    filename="graphs/trapezoidal_move_pos_nofilt_position.pdf",
    title=r"Consigne de vitesse: Trapèze",
    ylabel=r"$x [m]$",
)

df = pd.read_csv("trapezoidal_move_neg_nofilt.csv")

# Plot column 4 vs column 0
save_values(
    df,
    x_col=0,
    y_col=[2, 3],
    filename="graphs/trapezoidal_move_neg_nofilt_speeds.pdf",
    title=r"Consigne de vitesse: Trapèze",
    ylabel=r"$[m/s]$",
)

save_values(
    df,
    x_col=0,
    y_col=6,
    filename="graphs/trapezoidal_move_neg_nofilt_position.pdf",
    title=r"Consigne de vitesse: Trapèze",
    ylabel=r"$x [m]$",
)
