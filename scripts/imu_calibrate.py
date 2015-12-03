#!/usr/bin/python2

# Calibrate IMU based on a steady state data set

import sys
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt


data = np.loadtxt(sys.argv[1], dtype=np.float)
data = data[1:-1, 1:] # Strip time and start/stop markers


rng = [x for x in range(50, data.shape[0]-50)]
stds = np.zeros((data.shape[1],))

for i in range(0, len(rng)):
    s = rng[i]
    d = data[s-50:s+50, :]
    #d = d - np.mean(data)
    stds = stds + np.std(d, axis=0)

stds = stds / len(rng)

print(stds)

print(np.std(data, axis=0))

print(np.mean(data, axis=0))

for i in range(0, data.shape[1]):
    #mu, sigma = 100, 15
    #x = mu + sigma*np.random.randn(10000)

    # the histogram of the data
    n, bins, patches = plt.hist(data[:, i], 50, normed=1, facecolor='green', alpha=0.75)

    # add a 'best fit' line
    #y = mlab.normpdf( bins, mu, sigma)
    #l = plt.plot(bins, y, 'r--', linewidth=1)

    #plt.xlabel('Smarts')
    #plt.ylabel('Probability')
    #plt.title(r'$\mathrm{Histogram\ of\ IQ:}\ \mu=100,\ \sigma=15$')
    #plt.axis([40, 160, 0, 0.03])
    plt.grid(True)

    plt.show()





means = np.empty((len(rng), data.shape[1]))
for i in range(0, len(rng)):
    s = rng[i]
    means[i, :] = np.mean(data[s-50:s+50, :])


#plt.plot(means[:, 4])

#plt.show()
