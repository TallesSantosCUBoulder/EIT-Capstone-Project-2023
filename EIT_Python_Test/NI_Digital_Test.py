import nidaqmx
import nidaqmx.constants as const
from time import sleep

def set_mux(deviceA, deviceB, current_injection, skip_num, num_channels):
    mux_setA = int(bin(1<<4)[:1:-1])|((current_injection % num_channels)<<1)
    mux_setB = int(bin(1<<4)[:1:-1])|((current_injection + skip_num + 1)<<1)
    deviceA.write(mux_setA)
    deviceB.write(mux_setB)

def set_electrode(device, current_injection, skip_num, num_channels):
    electrode_set = (1<<(current_injection % num_channels)) | (1<<((current_injection+skip_num+1)% num_channels))
    device.write(electrode_set)
    
num_channels = 8
skip_num = 0

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

MuxDigiOutA.start()
MuxDigiOutB.start()
for i in range(num_channels):
    set_mux(MuxDigiOutA, MuxDigiOutB, i, skip_num, num_channels)
    set_electrode(SwitchSelect, i, skip_num, num_channels)
    input("Press Enter to continue...")

MuxDigiOutA.stop()
MuxDigiOutB.stop()
SwitchSelect.stop()
MuxDigiOutA.close()
MuxDigiOutB.close()
SwitchSelect.close()