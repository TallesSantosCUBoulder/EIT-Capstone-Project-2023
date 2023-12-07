import nidaqmx
import matplotlib.pyplot as plt
import numpy as np

with nidaqmx.Task() as task:
    task.ai_channels.add_ai_voltage_chan("Dev2/ai0", terminal_config=nidaqmx.constants.TerminalConfiguration.RSE)
    data = task.read(number_of_samples_per_channel=int(10e3))

ax = plt.subplot()
x = np.linspace(0, 1/10, int(10e3))
ax.plot(x, data, linewidth = 2.0)
plt.show()
