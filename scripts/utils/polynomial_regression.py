#!/usr/bin/python

"""
********************************************************************************
* Filename      : Polynomial Regression
* Author        : Susung Park
* Description   : Polynomial regression calculator.
* Version       : Version 1; 11 AUG 2019
********************************************************************************
"""

import numpy as np
import copy

class Polynomial:
    def __init__(self, args):
        self.coef = args
    
    def compute(self, x):
        if type(x) == np.ndarray:
            ans = np.zeros(x.shape)
            x_p = np.ones(x.shape)
        else:
            ans = 0
            x_p = 1

        for c in self.coef:
            ans += c * x_p
            x_p *= x
        return np.sum(ans)

def polynomial_regression_2d(order, dataset):
    power_sums = [0] * (2 * order  + 1)
    domain = dataset[:, 0]
    domain_power = np.ones(domain.shape)

    for x in range(len(power_sums)):
        power_sums[x] = np.sum(domain_power)
        domain_power *= domain
    
    regression_A = np.zeros([order+1, order+1])
    regression_b = np.zeros([order+1, 1])

    for r in range(order + 1):
        for c in range(order + 1):
            regression_A[r, c] = power_sums[r + c]
    
    for r in range(order + 1):
        regression_b[r, 0] = np.sum(dataset[:, 1] * dataset[:, 0]**r)
    
    coefs = np.matmul(np.linalg.inv(regression_A), regression_b)[:,0].tolist()
    return Polynomial(coefs)