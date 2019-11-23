import struct
import matplotlib.pyplot as plt
import numpy as np

path = "ramdump.dat"

# Cycles in ISR
n_isr = []

# Cycles time at entry to ISR
ns = []

with open(path, mode='rb') as file:
    for entry in struct.iter_unpack("II", file.read()[:32000]):
        n = entry[0]
        ns.append(n)
        dn = entry[1]-entry[0]
        n_isr.append(dn)

t_cycle = 1/400e6

t_isr = t_cycle * np.array(n_isr)
dts = t_cycle * np.diff(ns)

fig, axs = plt.subplots(2,1)
axs[0].plot(1e6 * t_isr)
axs[0].set_title("ISR execution time")
axs[0].set_ylabel("t /us")
axs[0].grid()
axs[1].plot(1e6 * dts)
axs[1].set_title("Time between ISR start")
axs[1].set_ylabel("t /us")
axs[1].grid()

plt.tight_layout()
plt.show()
