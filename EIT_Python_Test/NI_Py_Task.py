import nidaqmx as daq
import nidaqmx.stream_readers as streamRead
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType, TerminalConfiguration, TaskMode
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from time import sleep

numSamples = 1024
numCollected = 0
data = np.zeros(numSamples)
time = 1.0/125e3*numSamples
fig, ax = plt.subplots()

readTask = daq.Task('readTask')
readTask.ai_channels.add_ai_voltage_chan('Dev2/ai0', terminal_config=TerminalConfiguration.RSE)
#readTask.ai_channels.add_ai_voltage_chan('Dev1/ai0', terminal_config=TerminalConfiguration.RSE)
readTask.timing.cfg_samp_clk_timing(125e3, sample_mode=AcquisitionType.CONTINUOUS)
#readTask.in_stream.input_buf_size = 1024
reader = readTask.in_stream
readTask.start()



def collectData(dataOut, reader, numSamples):
    
    dataOut = reader.read(number_of_samples_per_channel=numSamples)
    
    return dataOut


def animate(i, data, numCollected, time):
    data = collectData(dataOut=data, reader=reader, numSamples=numSamples)
    x = np.linspace(0, time, numCollected)

    ax.clear()
    ax.plot(x, data, linewidth=2.0)
    ax.set_title('Plot of NI Data')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Voltage (V)')

    sleep(0.1)
    

ani = animation.FuncAnimation(fig, animate, fargs=(data, numCollected, time), interval=1, cache_frame_data=False)

plt.show()
readTask.close()