import numpy as np

class Rotor2D:
    def __init__(self, theta):
        self.c = np.cos(theta / 2.0)
        self.s = np.sin(theta / 2.0)

    def rotate(self, v):
        x, y = v
        return np.array([
            (self.c**2 - self.s**2)*x - 2*self.c*self.s*y,
            2*self.c*self.s*x + (self.c**2 - self.s**2)*y
        ])
