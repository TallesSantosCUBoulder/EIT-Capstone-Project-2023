import nidaqmx as daq
import nidaqmx.stream_readers as streamRead
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType, TerminalConfiguration, TaskMode
import matplotlib.pyplot as plt
import numpy as np

numSamples = 1024
data = np.zeros(numSamples)
time = 1.0/125e3*numSamples
ax = plt.subplot()

readTask = daq.Task('readTask')
readTask.ai_channels.add_ai_voltage_chan('Dev2/ai0', terminal_config=TerminalConfiguration.RSE)
readTask.timing.cfg_samp_clk_timing(125e3, sample_mode=AcquisitionType.FINITE, samps_per_chan=numSamples)
readTask.in_stream.input_buf_size = 1024
reader = streamRead.AnalogSingleChannelReader(readTask.in_stream)
samps = reader.read_many_sample(data, number_of_samples_per_channel=numSamples)

x = np.linspace(0, time, samps)
ax.plot(x, data, linewidth = 2.0)

plt.show()

ax.clear()

samps = reader.read_many_sample(data, number_of_samples_per_channel=numSamples)

ax.plot(x, data, linewidth = 2.0)

plt.show()

readTask.close()