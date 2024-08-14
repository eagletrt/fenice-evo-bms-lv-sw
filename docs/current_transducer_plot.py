import numpy as np
import matplotlib.pyplot as plt

HO_50_SP33_1106_THEORETICAL_SENSITIVITY = 9.2
HO_50_SP33_1106_VREF_mV  = 1645
HO_50_SP33_1106_OCD_MULT = 2.92
S_HALL1_OFFSET_A = 0.3
S_HALL2_OFFSET_A = 1.5

def hall1(adc_value_mV):
    return (((adc_value_mV - HO_50_SP33_1106_VREF_mV) / HO_50_SP33_1106_THEORETICAL_SENSITIVITY)) - S_HALL1_OFFSET_A 

def hall2(adc_value_mV):
    return (((adc_value_mV - HO_50_SP33_1106_VREF_mV) / HO_50_SP33_1106_THEORETICAL_SENSITIVITY)) - S_HALL2_OFFSET_A

print("precision [mA] =", (hall1(1501) - hall1(1500)))
    
xs = np.linspace(1500, 1900, 1000)
hall1ys = [hall1(x) for x in xs]
hall2ys = [hall2(x) for x in xs]
plt.plot(xs, hall1ys, label="hall1")
plt.plot(xs, hall2ys, label="hall2")
plt.xlabel('adc measurement [mV]')
plt.ylabel('measured current [A]')
plt.legend()
plt.show()



