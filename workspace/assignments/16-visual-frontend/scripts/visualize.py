import pandas as pd

import numpy as np

import matplotlib
import matplotlib.pyplot as plt

# load evaluation results:
data = pd.read_csv("../doc/01-triangulation/evaluation-results.csv")

# get sliding window sizes:
sliding_window_sizes = map(lambda x: str(x), data['sliding_window_size'].unique())
# get measurement noises:
measurement_noises = map(lambda x: str(x), data['measurement_noise_stddev'].unique())[1:]

# get ratios:
ratios = np.zeros(
    (len(measurement_noises), len(sliding_window_sizes))
)
for (j, m) in enumerate(measurement_noises):
    for (i, s) in enumerate(sliding_window_sizes):
        ratios[j, i] = np.log(
            data[
                (data['measurement_noise_stddev'] == float(m)) & (data['sliding_window_size'] == int(s))
            ].quality_ratio.iloc[0]
        )

# visualize ratios:
fig, ax = plt.subplots()
im = ax.imshow(ratios)

# show all ticks and label them with the respective list entries:
ax.set_xticks(np.arange(len(sliding_window_sizes)))
ax.set_yticks(np.arange(len(measurement_noises)))
ax.set_xticklabels(sliding_window_sizes)
ax.set_yticklabels(measurement_noises)

# rotate the tick labels and set their alignment:
plt.setp(
    ax.get_xticklabels(), 
    rotation=45, ha="right",
    rotation_mode="anchor"
)

# loop over ratios and create text annotations.
for j in range(len(measurement_noises)):
    for i in range(len(sliding_window_sizes)):
        text = ax.text(
            i, j, "{:.2f}".format(ratios[j, i]),
            ha="center", va="center", color="w"
        )

ax.set_title("Triangulation Quality Ratio")
fig.tight_layout()
plt.show()