# Write a function 'kalman_filter' that implements a multi-
# dimensional Kalman Filter for the example given

from math import *

"""
- Prediction is not necessarily to happen when there's a observation, because the observation could miss for some 
  time steps in the real world actually, but the prediction should always triggered when the time elapsed.
- KF always assumes there's already a prediction for current time step t (it could be a initial guess if we know
  nothing about the state). If we got a measurement at time step t, we'll update the state x and P according to the 
  newly observed data (at this moment, x should be close to the measurement). At the same time, the error of current
  measurement and the prediction we were done at time step t - 1 is taken into account to measure the uncertainty
  and the hidden state (e.g. velocity). OK, the next thing we're going to do is to give a prediction for 
  time step t + 1. And this rule goes over and over.
- In the course example, we have a state vector containing position and velocity. We do observe position but nothing 
  about the velocity. But the interesting thing is we can still maintain a quite accurate velocity value using 
  position observations and our transition matrix F. The velocity here is so called hidden state, which could be the
  observable variables' driven power.
- Question: why the uncertainty P is quite different before and after the prediction-step?

"""


class Matrix:
    """implements basic operations of a matrix class"""
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def zero(self, dimx, dimy):
        if dimx < 1 or dimy < 1:  # check if valid dimensions
            raise ValueError("Invalid size of matrix")
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        if dim < 1:  # check if valid dimension
            raise ValueError("Invalid size of matrix")
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def show(self):
        for i in range(self.dimx):
            print(self.value[i])
        print(' ')

    def __add__(self, other):
        if self.dimx != other.dimx or self.dimy != other.dimy:  # check if correct dimensions
            raise ValueError("Matrices must be of equal dimensions to add")
        else:
            # add if correct dimensions
            res = Matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError("Matrices must be of equal dimensions to subtract")
        else:
            # subtract if correct dimensions
            res = Matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError("Matrices must be m*n and n*p to multiply")
        else:
            # multiply if correct dimensions
            res = Matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res

    def transpose(self):
        # compute transpose
        res = Matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    def cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = Matrix([[]])
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i]) ** 2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError("Matrix not positive-definite")
                res.value[i][i] = sqrt(d)
            for j in range(i + 1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                try:
                    res.value[i][j] = (self.value[i][j] - S) / res.value[i][i]
                except:
                    raise ValueError("Zero diagonal")
        return res

    def cholesky_inverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = Matrix([[]])
        res.zero(self.dimx, self.dimx)

        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k] * res.value[j][k] for k in range(j + 1, self.dimx)])
            res.value[j][j] = 1.0 / tjj ** 2 - S / tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum(
                    [self.value[i][k] * res.value[k][j] for k in range(i + 1, self.dimx)]) / self.value[i][i]
        return res

    def inverse(self):
        aux = self.cholesky()
        res = aux.cholesky_inverse()
        return res

    def __repr__(self):
        return repr(self.value)


########################################

# Implement the filter function below

def kalman_filter(x, P):
    for n in range(len(measurements)):
        pass
    # measurement update

    # prediction

    return x, P


############################################
### use the code below to test your filter!
############################################

measurements = [1, 2, 3]

x = Matrix([[0.], [0.]])  # initial state (location and velocity)
P = Matrix([[1000., 0.], [0., 1000.]])  # initial uncertainty
u = Matrix([[0.], [0.]])  # external motion
F = Matrix([[1., 1.], [0, 1.]])  # next state function
H = Matrix([[1., 0.]])  # measurement function
R = Matrix([[1.]])  # measurement uncertainty
I = Matrix([[1., 0.], [0., 1.]])  # identity matrix

for z in measurements:
    print('-' * 80)
    # measurement update
    y = Matrix([[z]]) - H * x
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + K * y
    P = (I - K * H) * P

    print('x=')
    x.show()
    print('P=')
    P.show()

    # prediction
    x = F * x + u
    P = F * P * F.transpose()

    print('x=')
    x.show()
    print('P=')
    P.show()

# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]
