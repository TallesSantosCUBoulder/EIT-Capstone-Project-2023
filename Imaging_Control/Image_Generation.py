import numpy as np                      # Used for vector and matrix math
from  scipy.io import loadmat           # Used to load the reconstruction.mat
import matplotlib.pyplot as plt         # Used for plotting image  
import nidaqmx                          # Used to connect python to the NI boards
import nidaqmx.constants as const       # Constants used to configure NI boards
import time                             # Used for sleeping for the RC curve 

def set_mux(deviceA, deviceB, current_injection, skip_num, num_channels):
    """
    Controls the mux channels being used for current injection. Device A controls
    the zero phase current and Device B controls the 180 phase current. 
    Args:
        deviceA (nidaqmx.Task): NIDAQMX Task to control Mux A
        deviceB (nidaqmx.Task): NIDAQMX Task to control Mux B
        current_injection (int): Electrode number for zero phase current
        skip_num (int): Number of electrodes to skip for 180 phase current
        num_channels (int): Total number of electrodes being used
    """
    mux_setA = 1|((current_injection % num_channels)<<1)
    mux_setB = 1|((current_injection + skip_num + 1)<<1)
    deviceA.write(mux_setA)
    deviceB.write(mux_setB)

def set_electrode(device, current_injection, skip_num, num_channels):
    """
    Controls the switches in the electrodes to enable/disable current injection
    Args:
        device (nidaqmx.Task): NIDAQMX Task to control switches
        current_injection (int): Electrode number for zero phase current
        skip_num (int): Number of electrodes to skip for 180 phase current
        num_channels (int): Total number of electrodes being used
    """    
    electrode_set = (1<<(current_injection % num_channels)) | (1<<((current_injection+skip_num+1)% num_channels))
    device.write(electrode_set)

def compute_epiv(frq, N, sample_frq):
    """
    Used to pre-compute the Epiv matrix for signal demodulation
    Args:
        frq (np.arry): An array containing the frequency of the impulsed signal
        N (int): Number of samples used
        sample_frq (int): Sample Frequency

    Returns:
        np.ndarray: Pre-computed Epiv Matrix for demodulation
    """    
    tk = np.arange(N) / sample_frq
    w = 2 * np.pi * frq * tk[:, np.newaxis]
    Etot = np.hstack((np.sin(w), np.cos(w), np.ones((N, len(frq)))))
    return np.linalg.pinv(Etot)

def multi_freq_demod(signal, Epiv, inject):
    """
    Demodulate signal for image reconstruction
    Args:
        signal (np.ndarray): num_channels x N Matrix of Electrode Samples
        Epiv (np.ndarray): Pre-computed Epiv Matrix Used for Demodulation
        inject (int): Electrode number the zero phase signal is being impulsed on

    Returns:
        np.array: Real component of impulsed signal for each electrode
    """    
    phi_tot = np.matmul(Epiv, signal)
    amp = np.sqrt(phi_tot[0, :] ** 2 + phi_tot[1, :] ** 2)
    phase = np.arctan2(phi_tot[1, :], phi_tot[0, :])
    phase -= phase[inject]
    return np.real(amp * np.exp(1j * phase))

def collectData(dataOut, numSamples, sampleRate):
    """
    Collect a vector of voltages to be demodulated
    Args:
        dataOut (np.ndarray): num_channels x N Matrix of Electrode Samples
        numSamples (int): Number of Samples to Collect
        sampleRate (int): Sample Frequency

    Returns:
        np.array: num_channels x N Matrix of Electrode Samples
    """    
    # Add ADC Inputs
    readTaskA = nidaqmx.Task('readTaskA') # Create analog read task
    readTaskA.ai_channels.add_ai_voltage_chan('Dev1/ai0:7', terminal_config=const.TerminalConfiguration.RSE, min_val=-1, max_val=1)
    readTaskA.timing.cfg_samp_clk_timing(sample_rate, sample_mode=const.AcquisitionType.FINITE, samps_per_chan=N)
    readerA = readTaskA.in_stream
    readerA.timeout = -1
    
    readTaskB = nidaqmx.Task('readTaskB')
    readTaskB.ai_channels.add_ai_voltage_chan('Dev2/ai0', terminal_config=const.TerminalConfiguration.RSE)
    readTaskB.timing.cfg_samp_clk_timing(sample_rate, '/Dev1/ai/SampleClock', sample_mode=const.AcquisitionType.FINITE, samps_per_chan=N)
    readTaskB.triggers.start_trigger.cfg_dig_edge_start_trig(readTaskA.triggers.start_trigger.term)
    readerB = readTaskB.in_stream
    readerB.timeout = -1
    s = time.time()
    dataOut = readerA.read(number_of_samples_per_channel=const.READ_ALL_AVAILABLE)
    #dataOut[1,:] = readerB.read(number_of_samples_per_channel=numSamples)
    e = time.time()
    print(str(e-s) + " Read")
    readTaskB.stop()
    readTaskA.stop()
    readTaskA.close()
    readTaskB.close()
    del readerA, readerB, readTaskA, readTaskB
    return dataOut
    
def gather_frame(MuxDigiOutA, MuxDigiOutB, SwitchSelect, data, num_channels, skip_num, N, Epiv):
    """
    demodulated (num_channels^2)x1 voltage vector for image generation
    Args:
        MuxDigiOutA (nidaqmx.Task): NIDAQMX Task to control Mux A
        MuxDigiOutB (nidaqmx.Task): NIDAQMX Task to control Mux B
        SwitchSelect (nidaqmx.Task): NIDAQMX Task to control switches
        data (np.ndarray): num_channels x N Matrix of Electrode Samples
        num_channels (int): Total number of electrodes being used
        skip_num (int): Number of electrodes to skip for 180 phase current
        N (int): Number of samples used
        Epiv (np.ndarray): Pre-computed Epiv matrix for demodulation

    Returns:
        np.ndarray: demodulated (num_channels^2)x1 voltage vector for image generation
    """    
    volt_vec = np.zeros((num_channels ** 2,))
    for i in range(num_channels):
        s = time.time()
        set_mux(MuxDigiOutA, MuxDigiOutB, i, skip_num, num_channels)
        e = time.time()
        print(str(e-s) + " MUX")
        s = time.time()
        set_electrode(SwitchSelect, i, skip_num, num_channels)
        e = time.time()
        print(str(e-s) + " SWITCH")
        time.sleep(600 / 110e3)
        data = collectData(data, N, num_channels).reshape((N,num_channels))
        hold = multi_freq_demod(data, Epiv, i)
        volt_vec[i * num_channels: (i + 1) * num_channels] = hold[:num_channels]
    return volt_vec

# Initialize Variables
num_channels = 8
skip_num = 0
adc_range = 1
curr_r = np.array([10.03, 10.2])
curr_g = np.array([49.72, 53.05])
N = 256
sample_rate = 110e3
data = np.zeros([num_channels,N])
output_rate = 1e6
buffer_for_preload = 500e3
pkpV = 3
frq = np.array([20e3])
phase = np.array([0, np.pi])
dFrq = -90.00548

# Load Reconstruction Matrix
reconstruction = loadmat('Imaging_Control/reconstruction.mat')

MatrixA_int = reconstruction['MatrixA_int']
mask = reconstruction['mask']
n_pixels = reconstruction['n_pixels']

# Clear the loaded data (optional in Python)
del reconstruction

# Add Digital Mux Select A
MuxDigiOutA = nidaqmx.Task()
MuxDigiOutA.do_channels.add_do_chan("Dev1/port1/line0:4")
MuxDigiOutA.stop()

# Add Digital Mux Select B
MuxDigiOutB = nidaqmx.Task()
MuxDigiOutB.do_channels.add_do_chan("Dev2/port1/line0:4")
MuxDigiOutB.stop()

# Add Digital Switch Select
SwitchSelect = nidaqmx.Task()
SwitchSelect.do_channels.add_do_chan("Dev1/port2/line0:7")
SwitchSelect.stop()

# Add Analog Output
dAOutA = nidaqmx.Task()
dAOutA.ao_channels.add_ao_voltage_chan("Dev1/ao0", min_val=-5, max_val=5)
dAOutA.timing.cfg_samp_clk_timing(output_rate, sample_mode=const.AcquisitionType.CONTINUOUS)
dAOutA.stop()

dAOutB = nidaqmx.Task()
dAOutB.ao_channels.add_ao_voltage_chan("Dev2/ao0", min_val=-5, max_val=5)
dAOutB.triggers.start_trigger.cfg_dig_edge_start_trig(dAOutA.triggers.start_trigger.term) 
dAOutB.timing.cfg_samp_clk_timing(output_rate, '/Dev1/ao/SampleClock', sample_mode=const.AcquisitionType.CONTINUOUS)
dAOutB.stop()

output_signal = loadmat('Imaging_Control/outputSignal.mat')
outputSignal = output_signal['outputSignal']

# Clear the loaded data (optional in Python)
del output_signal

# Main Loop
dAOutB.write(outputSignal[:,1], auto_start=True)
dAOutA.write(outputSignal[:,0], auto_start=True)

Epiv = compute_epiv(frq + dFrq, N, sample_rate)
empty_tank = gather_frame(MuxDigiOutA, MuxDigiOutB, SwitchSelect, data, num_channels, skip_num, N, Epiv)
plt.ion()
fig = plt.figure()
ax = fig.add_subplot()
line = None

while True:
    volt_vec = gather_frame(MuxDigiOutA, MuxDigiOutB, SwitchSelect, data, num_channels, skip_num, N, Epiv)
    voltage_vec_diff = (volt_vec - empty_tank)
    step1 = np.matmul(MatrixA_int, voltage_vec_diff)
    step2 = np.real(step1) * mask.T
    imagem = np.rot90(step2.reshape((64,64)),1)
    
    if(line == None):
        line = ax.imshow(imagem, cmap='jet', origin='lower', aspect='equal')
    else:
        line.set_data(imagem)
        line.set_clim([-5000, 5000])
        
    ax.redraw_in_frame
    if not plt.fignum_exists(1):
        print('Loop stopped by user')
        break
    
    fig.canvas.flush_events()

# Stop Outputs
MuxDigiOutA.stop()
MuxDigiOutB.stop()
SwitchSelect.stop()
dAOutA.stop()
dAOutB.stop()

MuxDigiOutA.close()
MuxDigiOutB.close()
SwitchSelect.close()
dAOutA.close()
dAOutB.close()