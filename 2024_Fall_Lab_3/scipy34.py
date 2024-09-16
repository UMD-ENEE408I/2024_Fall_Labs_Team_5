import numpy as np
from scipy.linalg import solve
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
 
#linear system of equations 

A = np.array([[3, 1], [1, 2]])
b = np.array([9, 8])

ans = solve(A,b)

print("Solution to system of equations is \n", ans)

#finding the minimum of the function with optimization module

def func(x):
	return x**2 + 2*x

init = 0.0

result = minimize(func, init)

print("min value of x: \n", result.x)
print("function output at the  minimum: \n", result.fun)


 # fourier transforms

def f(x):
	return np.sin(100 * np.pi * x) + 0.5 * np.sin(160 * np.pi * x) 

x = np.linspace(0, 1, 1000)
y = f(x) 

Y = fft(y)

frequencies = fftfreq(len(x), (x[1] - x[0]))

plt.plot(frequencies, np.abs(Y))
plt.title("Frequency Response of f(x)")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Amplitude")

plt.grid(True)
plt.show()

