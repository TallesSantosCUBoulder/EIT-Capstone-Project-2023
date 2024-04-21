import nidaqmx as daq
import nidaqmx.stream_readers as streamRead
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType, TerminalConfiguration, TaskMode
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from time import sleep
from scipy.io import loadmat

def set_mux(device, current_injection, skip_num, num_channels):
    mux_set = [1] + list(reversed([int(x) for x in f'{current_injection % num_channels:04b}'])) + \
              [1] + list(reversed([int(x) for x in f'{(current_injection + skip_num + 1) % num_channels:04b}']))
    device.write(mux_set)

def set_electrode(device, current_injection, skip_num, num_channels):
    electrode_set = [int(x) for x in f'{current_injection:03b}']
    electrode_set.extend([int(x) for x in f'{(current_injection + skip_num + 1) % num_channels:03b}'])
    device.write(electrode_set)

def compute_epiv(frq, N, sample_frq):
    tk = np.arange(N) / sample_frq
    w = 2 * np.pi * frq * tk[:, np.newaxis]
    Etot = np.hstack((np.sin(w), np.cos(w), np.ones((N, len(frq)))))
    return np.linalg.pinv(Etot)

def multi_freq_demod(signal, Epiv, inject):
    phi_tot = np.dot(Epiv, signal.T)
    amp = np.sqrt(phi_tot[:, 0] ** 2 + phi_tot[:, 1] ** 2)
    phase = np.arctan2(phi_tot[:, 1], phi_tot[:, 0])
    phase -= phase[inject]
    return np.real(amp * np.exp(1j * phase))

def gather_frame(MuxDigiOut, SwitchSelect, dDAQ, num_channels, skip_num, N, Epiv):
    volt_vec = np.zeros((num_channels ** 2,))
    for i in range(num_channels):
        set_mux(MuxDigiOut, i, skip_num, num_channels)
        set_electrode(SwitchSelect, i, skip_num, num_channels)
        time.sleep(300 / 110e3)
        sig = dDAQ.read(N)
        hold = multi_freq_demod(sig, Epiv, i)
        volt_vec[i * num_channels: (i + 1) * num_channels] = hold[:num_channels]
    return volt_vec

## Initialize Variables
numChannels = 8;                    # Number of electrodes in the system
SkipN = 0;                          # Number of electrodes to skip
adcRange = 1;                       # Sets the range of the adc
currR = [10.03; 10.2];              # Resistance current is being measured over
currG = [49.72; 53.05];             # Gain for current measurement
N = 512;                            # Number of samples taken
sampleRate = 110e3;                 # Sets the rate that the NI-Boards sample at
outputRate = 1e6;                   # Sets the rate that the NI-Board output at
bufferForPreLoad = 500e3;           # sets the length of buffer for preloading
pkpV = 3;                           # pk-pk voltage per frequency
frq = [20e3];                       # array of frequencies to make up multi-freq signal
phase = [0 pi];                     # phase of signal 1 versus signal 2
dFrq = -90.00548                    # Frequency Shift 

# Load Reconstruction Matrix
reconstruction = loadmat('reconstruction.mat')

MatrixA_int = reconstruction['MatrixA_int']
mask = reconstruction['mask']
n_pixels = reconstruction['n_pixels']

# Clear the loaded data (optional in Python)
del reconstruction


readTaskA = daq.Task('readTaskA') # Create analog read task
readTaskA.ai_channels.add_ai_voltage_chan('Dev1/ai0', terminal_config=TerminalConfiguration.RSE)
readTaskA.timing.cfg_samp_clk_timing(sampleRate, sample_mode=AcquisitionType.CONTINUOUS)
readerA.triggers.sync_type.MASTER = True
readerA = readTaskA.in_stream

readTaskB = daq.Task('readTaskB')
readTaskB.ai_channels.add_ai_voltage_chan('Dev2/ai0', terminal_config=TerminalConfiguration.RSE)
readTaskB.timing.cfg_samp_clk_timing(source="Dev1/ai/SampleClock", sample_mode=AcquisitionType.CONTINUOUS)
readTaskB.triggers.sync_type.SLAVE = True
readTaskB.triggers.start_trigger.cfg_dig_edge_start_trig('Dev1/ai/StartTrigger')
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



#ani = animation.FuncAnimation(fig, animate, fargs=(data, reader, N, dT), interval=150, cache_frame_data=False)
ax.plot(x, data, linewidth=2.0)
plt.show()

readTaskA.stop()
readTaskB.stop()
readTaskA.close()
readTaskB.close()


