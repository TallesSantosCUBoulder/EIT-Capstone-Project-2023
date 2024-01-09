import nidaqmx
import nidaqmx.constants as const
import matplotlib.pyplot as plt
import numpy as np
import time
while True:
    with nidaqmx.Task() as task:
        task.do_channels.add_do_chan("Dev1/port0/line0:4")
        task.write(0b11)
        time.sleep(1)
        task.write(0b01)
        time.sleep(1)

