import nidaqmx
import nidaqmx.constants as const
import matplotlib.pyplot as plt
import numpy as np

with nidaqmx.Task() as task:
    task.do_channels.add_do_chan("Dev1/port0/line0:4")
    data = task.write(4)
