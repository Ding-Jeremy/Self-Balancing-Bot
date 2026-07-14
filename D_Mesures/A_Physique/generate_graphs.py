from matplotlib import pyplot as plt
import pandas as pd

# Open CSV file
df = pd.read_csv("trapezoidal_move.csv")

# See the column names
print(df.columns)

# Plot columns (example: first column as x, second column as y)
plt.plot(df.iloc[:, 0], df.iloc[:, 1], label="speed_target")
plt.plot(df.iloc[:, 0], df.iloc[:, 2], label="speed")
# Add labels
plt.xlabel(df.columns[0])
plt.ylabel(df.columns[1])
plt.title("CSV Data Plot")

# Show grid
plt.grid(True)
plt.legend()
plt.savefig("trapezoidal_move.pdf", dpi=400)

# Display plot
plt.show()
