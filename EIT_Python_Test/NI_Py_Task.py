import nidaqmx as daq
import nidaqmx.stream_readers as streamRead
from nidaqmx.constants import AcquisitionType, Edge, Signal, TerminalConfiguration, TaskMode
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time

## Initialize Variables
sampleRate = 110e3
N = 128
data = np.zeros([2,N])
dT = 1.0/sampleRate*N
x = np.linspace(0, dT, N)

def collectData(dataOut, numSamples, sampleRate):

    readTaskA = daq.Task('readTaskA') # Create analog read task
    readTaskA.ai_channels.add_ai_voltage_chan('Dev1/ai0', terminal_config=TerminalConfiguration.RSE)
    readTaskA.timing.cfg_samp_clk_timing(sampleRate, samps_per_chan=N)
    readerA = readTaskA.in_stream
    readerA.auto_start = True

    readTaskB = daq.Task('readTaskB')
    readTaskB.ai_channels.add_ai_voltage_chan('Dev2/ai0', terminal_config=TerminalConfiguration.RSE)
    readTaskB.timing.cfg_samp_clk_timing(sampleRate, '/Dev1/ai/SampleClock', sample_mode=AcquisitionType.FINITE, samps_per_chan=N)
    readTaskB.triggers.start_trigger.cfg_dig_edge_start_trig(readTaskA.triggers.start_trigger.term)
    readerB = readTaskB.in_stream
    readTaskB.start()

    readTaskA.wait_until_done(daq.constants.WAIT_INFINITELY)
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



#ani = animation.FuncAnimation(fig, animate, fargs=(data, reader, N, dT), interval=150, cache_frame_data=False)



plt.ion()
i = 0
while True:
    print(i)
    i += 1
    data = collectData(data, readTaskA, readTaskB, readerA, readerB, N, sampleRate)
    dataShow = np.rot90(data,1)
    plt.plot(x, dataShow, linewidth=2.0)

    plt.draw()
    if not plt.fignum_exists(1):
        print('Loop stopped by user')
        break
    time.sleep(0.1)

readTaskA.stop()
readTaskB.stop()
readTaskA.close()
readTaskB.close()


