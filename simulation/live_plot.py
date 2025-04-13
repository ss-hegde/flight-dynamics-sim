import matplotlib.pyplot as plt
import numpy as np

class LivePlotter:
    def __init__(self, labels):
        self.labels = labels
        self.n = len(labels)
        self.fig, self.axs = plt.subplots(self.n, 1, figsize=(8, 2 * self.n), sharex=True)
        self.data = [[] for _ in range(self.n)]
        self.time = []

    def update(self, t, state_vector):
        self.time.append(t)
        for i in range(self.n):
            self.data[i].append(state_vector[i])

        for i in range(self.n):
            self.axs[i].clear()
            self.axs[i].plot(self.time, self.data[i])
            self.axs[i].set_ylabel(self.labels[i])
            self.axs[i].grid(True)
        self.axs[-1].set_xlabel("Time (s)")
        plt.pause(0.001)
