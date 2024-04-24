import nidaqmx
import nidaqmx.constants as const
from  scipy.io import loadmat
from time import sleep

output_rate = 1e6

# Add Analog Output
dAOutA = nidaqmx.Task()
dAOutA.ao_channels.add_ao_voltage_chan("Dev1/ao0", min_val=-10.0, max_val=10.0)
dAOutA.timing.cfg_samp_clk_timing(output_rate, sample_mode=const.AcquisitionType.CONTINUOUS)
dAOutA.stop()

dAOutB = nidaqmx.Task()
dAOutB.ao_channels.add_ao_voltage_chan("Dev2/ao0", min_val=-10.0, max_val=10.0)
dAOutB.triggers.start_trigger.cfg_dig_edge_start_trig(dAOutA.triggers.start_trigger.term) 
dAOutB.timing.cfg_samp_clk_timing(output_rate, '/Dev1/ao/SampleClock', sample_mode=const.AcquisitionType.CONTINUOUS)
dAOutB.stop()

output_signal = loadmat('EIT_Python_Test/outputSignal.mat')
outputSignal = output_signal['outputSignal']
dAOutB.write(outputSignal[:,1], auto_start=True)
dAOutA.write(outputSignal[:,0], auto_start=True)



input()

dAOutA.stop()
dAOutB.stop()
dAOutA.close()
dAOutB.close()
