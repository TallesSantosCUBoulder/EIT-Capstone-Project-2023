import nidaqmx
import nidaqmx.constants as const
import sleep from time

def set_mux(device, current_injection, skip_num, num_channels):
    mux_set = [1] + list(reversed([int(x) for x in f'{current_injection % num_channels:04b}'])) + \
              [1] + list(reversed([int(x) for x in f'{(current_injection + skip_num + 1) % num_channels:04b}']))
    device.write(mux_set).

def set_electrode(device, current_injection, skip_num, num_channels):
    electrode_set = (1<<(current_injection % num_channels)) | (1<<((current_injection+skip_num+1)% num_channels))
    device.write(electrode_set)
    
num_channels = 8
skip_num = 0

# Add Digital Mux Select
MuxDigiOut = nidaqmx.Task()
MuxDigiOut.do_channels.add_do_chan("Dev1/port1/line0:4")
MuxDigiOut.do_channels.add_do_chan("Dev2/port1/line0:4")
MuxDigiOut.stop()

# Add Digital Switch Select
SwitchSelect = nidaqmx.Task()
SwitchSelect.do_channels.add_do_chan("Dev1/port2/line0:7")
SwitchSelect.stop()

for i in range(num_channels):
    set_mux(MuxDigiOut, i, skip_num, num_channels)
    set_electrode(SwitchSelect, i, skip_num, num_channels)
    input("Press Enter to continue...")

MuxDigiOut.stop()
MuxDigiOut.close()
