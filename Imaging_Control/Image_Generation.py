import numpy as np
import matplotlib.pyplot as plt
import nidaqmx
import time

def set_mux(deviceA, deviceB, current_injection, skip_num, num_channels):
    mux_setA = int(bin(1<<4)[:1:-1])|((current_injection % num_channels)<<1)
    mux_setB = int(bin(1<<4)[:1:-1])|((current_injection + skip_num + 1)<<1)
    deviceA.write(mux_setA)
    deviceB.write(mux_setB)

def set_electrode(device, current_injection, skip_num, num_channels):
    electrode_set = (1<<(current_injection % num_channels)) | (1<<((current_injection+skip_num+1)% num_channels))
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

# Initialize Variables
num_channels = 8
skip_num = 0
adc_range = 1
curr_r = np.array([10.03, 10.2])
curr_g = np.array([49.72, 53.05])
N = 512
sample_rate = 110e3
output_rate = 1e6
buffer_for_preload = 500e3
pkpV = 3
frq = np.array([20e3])
phase = np.array([0, np.pi])
dFrq = -90.00548

# Load Reconstruction Matrix
reconstruction = np.load('reconstruction.npy', allow_pickle=True).item()

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
dAOut = nidaqmx.Task()
AnalogCH1 = dAOut.ao_channels.add_ao_voltage_chan("Dev1/ao0", min_val=-10.0, max_val=10.0)
AnalogCH2 = dAOut.ao_channels.add_ao_voltage_chan("Dev2/ao0", min_val=-10.0, max_val=10.0)
dAOut.stop()

dAOut.triggers.start_trigger.cfg_dig_edge_start_trig(trigger_source="Dev1/RTSI0")
dAOut.timing.cfg_samp_clk_shared_src(src_terminal="Dev1/RTSI1")
dAOut.timing.cfg_samp_clk_timing(rate=output_rate)

output_signal = np.load("outputSignal.npy")

preload(dAOut, output_signal)

# Add ADC Inputs
dDAQ = nidaqmx.Task()
dDAQ.ai_channels.add_ai_voltage_chan("Dev1/ai0:7", min_val=-adc_range, max_val=adc_range)

dDAQ.timing.cfg_samp_clk_shared_src(src_terminal="Dev1/RTSI3")
dDAQ.timing.cfg_samp_clk_timing(rate=sample_rate)

# Main Loop
dAOut.start(task="repeatoutput")

Epiv = compute_epiv(frq + dFrq, N, sample_rate)
empty_tank = gather_frame(MuxDigiOut, SwitchSelect, dDAQ, num_channels, skip_num, N, Epiv)

while True:
    volt_vec = gather_frame(MuxDigiOut, SwitchSelect, dDAQ, num_channels, skip_num, N, Epiv)
    voltage_vec_diff = (volt_vec - empty_tank)
    
    imagem = np.real(np.dot(MatrixA_int, voltage_vec_diff).reshape(n_pixels, n_pixels) * mask)
    plt.imshow(imagem.T, cmap='jet', origin='lower', aspect='equal')
    plt.colorbar()
    plt.show()

    if not plt.fignum_exists(2):
        print('Loop stopped by user')
        break

# Stop Outputs
dAOut.stop()
MuxDigiOut.stop()
SwitchSelect.stop()
