from pylablib.devices.NI import daq

dev = daq.NIDAQ(dev_name="Dev1", reset=True)
dev.add_voltage_input("input1", "ai0", terminal_cfg="rse")
