from osp import *

devs = find_osp_peripheral(OSP_DEV_OBP)

if len(devs) >0:
    dev = devs[0]
    print("SOC: "+str(dev.soc/100.0)+"%")
    print("V: "+str(dev.voltage)+"mV")
    print("I: "+str(dev.current)+"mA")
