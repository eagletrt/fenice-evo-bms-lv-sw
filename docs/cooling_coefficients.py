
# 40 0
# 50 20
# 60 100

import matplotlib.pyplot as plt
import numpy as np

def export_coeffients(ps, rs):
    print (f"#define PUMPS_COEFFICIENTS {{{ps[0]}, {ps[1]},  {ps[2]}}}")
    print (f"#define RADIATOR_COEFFICIENTS {{{rs[0]}, {rs[1]}, {rs[2]}}}")

aggressiveness = 2

a1_pumps_xs = [30, 40, 50, 60, 75]
a1_pumps_ys = [0, 0, 20, 30, 100]
a1_radiators_xs = [30, 45, 60, 70, 80]
a1_radiators_ys = [0, 0, 15, 50, 100]
a2_pumps_xs = [30, 40, 50, 55, 65]
a2_pumps_ys = [0, 5, 20, 30, 100]
a2_radiators_xs = [30, 40, 50, 65, 70]
a2_radiators_ys = [0, 0, 10, 30, 100]
a3_pumps_xs = [30, 40, 45, 50, 55]
a3_pumps_ys = [0, 5, 20, 30, 100]
a3_radiators_xs = [30, 40, 45, 50, 60]
a3_radiators_ys = [0, 5, 10, 30, 100]

a1_pumps_coeff = np.polyfit(a1_pumps_xs, a1_pumps_ys, 2)
a1_radiators_coeff = np.polyfit(a1_radiators_xs, a1_radiators_ys, 2)
a2_pumps_coeff = np.polyfit(a2_pumps_xs, a2_pumps_ys, 2)
a2_radiators_coeff = np.polyfit(a2_radiators_xs, a2_radiators_ys, 2)
a3_pumps_coeff = np.polyfit(a3_pumps_xs, a3_pumps_ys, 2)
a3_radiators_coeff = np.polyfit(a3_radiators_xs, a3_radiators_ys, 2)

# print(pumps_coeff, radiators_coeff)
 
# Define the range of x values
x = np.linspace(30, 80, 500)

# def f(x):
    # return coeff[0] * x**2 + coeff[1] * x + coeff[2]

a1_pumps = np.polyval(a1_pumps_coeff, x)
a1_radiators = np.polyval(a1_radiators_coeff, x)
a2_pumps = np.polyval(a2_pumps_coeff, x)
a2_radiators = np.polyval(a2_radiators_coeff, x)
a3_pumps = np.polyval(a3_pumps_coeff, x)
a3_radiators = np.polyval(a3_radiators_coeff, x)

export_coeffients(a1_pumps_coeff, a1_radiators_coeff)
export_coeffients(a2_pumps_coeff, a2_radiators_coeff)
export_coeffients(a3_pumps_coeff, a3_radiators_coeff)

# y2 = f(x)
# Create the plot
plt.plot(x, a1_pumps, label='a1_pumps(x)', color='blue', linestyle='-')
plt.plot(x, a1_radiators, label='a1_rads(x)', color='blue', linestyle='--')
plt.plot(x, a2_pumps, label='a2_pumps(x)', color='yellow', linestyle='-')
plt.plot(x, a2_radiators, label='a2_rads(x)', color='yellow', linestyle='--')
plt.plot(x, a3_pumps, label='a3_pumps(x)', color='red', linestyle='-')
plt.plot(x, a3_radiators, label='a3_rads(x)', color='red', linestyle='--')
plt.ylim(0, 100)

# Add title and labels
plt.title('Plot of Cooling')
plt.xlabel('IGBT Temperature')
plt.ylabel('Cooling %')

# Add a legend
plt.legend()

# Add grid
plt.grid(True)

# Display the plot
plt.show()
