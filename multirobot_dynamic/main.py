import time

import matplotlib.pyplot as plt
from bot import Bot
from optimizer import Optimizer

def globalOptimizer():
    X = []
    Y = []
    THETA = []
    V = []
    W = []

    for i in range(n):
        X.append(mybot[i].x)
        Y.append(mybot[i].y)
        THETA.append(mybot[i].theta)
        V.append(mybot[i].v)
        W.append(mybot[i].w)

    opt = Optimizer(X, Y, THETA, V, W)
    optV, optW = opt.optimize()
    for i in range(n):
        mybot[i].v = optV[i]
        mybot[i].w = optW[i]

n = 2

mybot = [None for i in range(n)]

mybot[0] = Bot(25, 25, 0.7, 0, 0)
mybot[0].set_goal([75, 75])

mybot[1] = Bot(75, 25, 0.7, 0, 0)
mybot[1].set_goal([25, 75])

plt.ion()

while True:
    plt.clf()
    plt.xlim([0, 100])
    plt.ylim([0, 100])

    plt.plot(mybot[0].x, mybot[0].y, 'bo', markersize = 3)
    plt.plot(mybot[0].goal[0], mybot[0].goal[1], 'go', markersize = 3)

    plt.plot(mybot[1].x, mybot[1].y, 'bo', markersize=3)
    plt.plot(mybot[1].goal[0], mybot[1].goal[1], 'go', markersize=3)

    plt.draw()

    mybot[0].PID()
    mybot[1].PID()

    globalOptimizer()

    mybot[0].move()
    mybot[1].move()

    plt.pause(0.01)

