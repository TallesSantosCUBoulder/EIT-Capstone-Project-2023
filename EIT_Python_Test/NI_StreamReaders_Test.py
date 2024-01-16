import nidaqmx as daq
import nidaqmx.stream_readers as streamRead
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType, TerminalConfiguration, TaskMode
import matplotlib.pyplot as plt
import numpy as np
numSamples = 512
data = np.zeros(numSamples)
time = 1.0/125e3*numSamples
ax = plt.subplot()

with daq.Task(new_task_name='read') as read:
    read.ai_channels.add_ai_voltage_chan('Dev2/ai0', terminal_config=TerminalConfiguration.RSE)
    read.timing.cfg_samp_clk_timing(125e3, sample_mode=AcquisitionType.FINITE, samps_per_chan=numSamples)
    read.in_stream.input_buf_size = 512
    
    def reading_task_callback(task_idx, event_type, num_samples, callback_data=None):
        reader = streamRead.AnalogSingleChannelReader(read.in_stream)
        samps = reader.read_many_sample(data, number_of_samples_per_channel=numSamples)
        x = np.linspace(0, time, samps)
        ax.plot(x, data, linewidth = 2.0)

        return 0
    
    read.register_every_n_samples_acquired_into_buffer_event(numSamples, reading_task_callback)

    read.start()
    plt.show()