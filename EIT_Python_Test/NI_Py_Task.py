import nidaqmx as daq
import nidaqmx.stream_readers as streamRead
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType, TerminalConfiguration, TaskMode
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from time import sleep




N = 512
sampleRate = 110e3
readTaskA = daq.Task('readTaskA') # Create analog read task
#daq.system.System().connect_terms( # Connect the clocks together
#    source_terminal='Dev1/ao/SampleClock',
#    destination_terminal='Dev2/RTSI2')
#daq.system.System().connect_terms( # Connect the triggers together
#    source_terminal='Dev1/ao/StartTrigger',
#    destination_terminal='Dev2/RTSI3')
readTaskA.ai_channels.add_ai_voltage_chan('Dev1/ai0', terminal_config=TerminalConfiguration.RSE)
#readTask.ai_channels.add_ai_voltage_chan('Dev2/ai0', terminal_config=TerminalConfiguration.RSE)
#readTask.triggers.reference_trigger.dig_edge_src('Dev1/ai/SampleClock')
#readTask.triggers.start_trigger.dig_edge_src('Dev1/ai/StartTrigger')
readTaskA.timing.cfg_samp_clk_timing(sampleRate, sample_mode=AcquisitionType.CONTINUOUS)
readerA = readTaskA.in_stream

readTaskB = daq.Task('readTaskB')
readTaskB.ai_channels.add_ai_voltage_chan('Dev2/ai0', terminal_config=TerminalConfiguration.RSE)
readTaskB.timing.cfg_samp_clk_timing(sampleRate, sample_mode=AcquisitionType.CONTINUOUS)
readerB = readTaskB.in_stream

readTaskA.start()
readTaskB.start()



def collectData(dataOut, readerA, readerB, numSamples):
    dataOut[0,:] = readerA.read(number_of_samples_per_channel=numSamples)
    dataOut[1,:] = readerB.read(number_of_samples_per_channel=numSamples)
    return dataOut


# def animate(i, data, reader, N, dT):
#     data = collectData(dataOut=data, reader=reader, numSamples=N)
#     x = np.linspace(0, dT, N)

#     ax.clear()
#     ax.plot(x, data, linewidth=2.0)
#     ax.set_title('Plot of NI Data')
#     ax.set_xlabel('Time (s)')
#     ax.set_ylabel('Voltage (V)')


data = np.zeros([2,N])
dT = 1.0/sampleRate*N
x = np.linspace(0, dT, N)
fig, ax = plt.subplots()
data = collectData(data, readerA, readerB, N)
data = np.rot90(data,1)
#ani = animation.FuncAnimation(fig, animate, fargs=(data, reader, N, dT), interval=150, cache_frame_data=False)
ax.plot(x, data, linewidth=2.0)
plt.show()

readTaskA.stop()
readTaskB.stop()
readTaskA.close()
readTaskB.close()

