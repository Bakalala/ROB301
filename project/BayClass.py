import numpy as np
import matplotlib.pyplot as plt


class BayClass:

    def __init__(self):
        # State Model
        # Back 0, Stay 1, Forward 2
        self.State = np.array(
            [[0.85, 0.1, 0.05], [0.05, 0.90, 0.05], [0.05, 0.10, 0.85]])

        # Measurmenet Model
        self.Blue = 0
        self.Green = 1
        self.Yellow = 2
        self.Orange = 3
        self.Nothing = 4
        self.Measurmenet = np.array([[0.60, 0.20, 0.05, 0.05], [0.20, 0.60, 0.05, 0.05], [
            0.05, 0.05, 0.65, 0.20], [0.05, 0.05, 0.15, 0.60], [0.10, 0.10, 0.10, 0.10]])

        # Map
        self.Map = np.array([self.Green, self.Orange, self.Green, self.Yellow,
                             self.Blue, self.Green, self.Orange, self.Yellow, self.Blue, self.Blue, self.Orange, self.Yellow])

        # Distribution of locatio
        self.Distribution = np.ones(12) / 12

    def update(self, input, zk):

        self.Distribution = np.roll(self.Distribution, -1) * self.State[input + 1][0] + self.Distribution * \
            self.State[input + 1][1] + \
            np.roll(self.Distribution, 1) * self.State[input + 1][2]

        for k in range(0, self.Distribution.size):
            xk = self.Map[k]
            self.Distribution[k] = self.Distribution[k] * \
                self.Measurmenet[zk][xk]

        self.Distribution = self.Distribution / np.sum(self.Distribution)
        index = np.argmax(self.Distribution)
        print(self.Distribution)
        if self.Distribution[index] > .5:
            return index + 1 #account for location since index starts at 0
        else:
            return -1
