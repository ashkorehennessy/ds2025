import matplotlib.pyplot as plt
import numpy as np
x = np.linspace(-180, 180, 1000)
y = np.log(np.abs(x))
y[x == 0] = 0  # Avoid log(0) which is undefined
plt.plot(x, y)
plt.show()