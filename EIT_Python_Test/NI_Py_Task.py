import nidaqmx as daq
import nidaqmx.stream_readers as streamRead
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType, TerminalConfiguration, TaskMode
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

numSamples = 1024
numCollected = 0
data = np.zeros((numSamples, 2))
time = 1.0/125e3*numSamples
fig, ax = plt.subplots()

def collectData(dataOut):
    readTask = daq.Task('readTask')
    readTask.ai_channels.add_ai_voltage_chan('Dev2/ai0', terminal_config=TerminalConfiguration.RSE)
    readTask.ai_channels.add_ai_voltage_chan('Dev1/ai0', terminal_config=TerminalConfiguration.RSE)
    readTask.timing.cfg_samp_clk_timing(125e3, sample_mode=AcquisitionType.CONTINUOUS, samps_per_chan=numSamples)
    readTask.in_stream.input_buf_size = 1024
    reader = streamRead.AnalogSingleChannelReader(readTask.in_stream)
    samps = reader.read_many_sample(dataOut, number_of_samples_per_channel=numSamples)
    readTask.close()
    return dataOut, samps

def animate(i, data, numCollected, time):
    data, numCollected = collectData(dataOut=data)
    x = np.linspace(0, time, numCollected)

    ax.clear()
    ax.plot(x, data, linewidth=2.0)
    ax.set_title('Plot of NI Data')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Voltage (V)')

ani = animation.FuncAnimation(fig, animate, fargs=(data, numCollected, time), interval=1, cache_frame_data=False)

plt.show()