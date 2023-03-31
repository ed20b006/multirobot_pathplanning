import numpy as np
from scipy.optimize import minimize

dt = 0.25

class Optimizer:

    def __init__(self, X, Y, theta, V, W):
        self.depth = 2
        self.noBot = 2
        self.x = np.zeros(2 * self.depth * self.noBot)
        self.X = X
        self.Y = Y
        self.theta = theta
        self.V = V
        self.W = W
        self.obsD = 7.0
        self.alpha = 50.0
        self.beta = 0.01

    def heading(self, v, w):
        head = 0
        for i in range(self.depth):
            for j in range(self.noBot):
                head += abs(self.V[j] - v[i][j]) + abs(self.W[j] - w[i][j])

        return head

    @staticmethod
    def cal_distance(x1, y1, x2, y2):
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    @staticmethod
    def cal_next_pos(x0, y0, theta0, v, w, dt):
        theta = theta0 - w * dt
        x = x0 + v * np.cos(theta) * dt
        y = y0 + v * np.sin(theta) * dt
        return x, y, theta

    def dis(self, v, w):
        global dt
        dis = 0

        X1 = self.X
        Y1 = self.Y
        THETA1 = self.theta

        for i in range(self.depth):
            for j in range(self.noBot):
                X1[j], Y1[j], THETA1[j] = self.cal_next_pos(X1[j], Y1[j], THETA1[j], v[i][j], w[i][j], dt)

            for j in range(self.noBot):
                for k in range(j+1, self.noBot):
                    dis += 1 / (self.cal_distance(X1[j], Y1[j], X1[k], Y1[k]) ** 2)

        return dis

    def objective(self, x):
        v = [x[i:i+self.noBot] for i in range(len(x)) if i % 4 == 0]
        w = [x[i:i+self.noBot] for i in range(len(x)) if i % 4 == 2]
        G = self.alpha * self.heading(v, w) + self.beta * self.dis(v, w)
        return G

    def optimize(self):
        bv = ((0.5, 3),) * self.noBot
        bw = ((-0.5, 0.5),) * self.noBot
        bnds = (bv + bw) * self.depth
        solution = minimize(self.objective, self.x, method='SLSQP', bounds=bnds)
        #print(f"{solution.x[0]:.2f} : {solution.x[1]:.2f}")
        #print(self.obstacles)
        return solution.x[0:self.noBot], solution.x[self.noBot:self.noBot * 2]