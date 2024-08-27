import matplotlib.pyplot as plt
import numpy as np

MAX_DAC_OUT = 4096.0
MAX_GPIO_OUT = 3.3
AMPLIFICATOR_GAIN = 1.5
MAX_OPAMP_OUT = MAX_GPIO_OUT *AMPLIFICATOR_GAIN
MIN_OPAMP_OUT = 2.0

def dac_pump_set_duty_cycle(duty_cycle):
    voltage = 0
    analog_volt = 0
    if duty_cycle != 0:
        voltage = duty_cycle * (MAX_OPAMP_OUT - MIN_OPAMP_OUT) + MIN_OPAMP_OUT
    analog_volt = ((voltage * MAX_DAC_OUT) / (MAX_OPAMP_OUT))
    return voltage

xs = np.linspace(0, 1, 1000)
ys = [dac_pump_set_duty_cycle(x) for x in xs]
plt.plot(xs, ys)
plt.savefig("dac_pump.png")
