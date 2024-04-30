import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import glob


# find all files with name that contains 'measurements/opd_freq_*_hz.csv', and get their filenames into an array
filenames = glob.glob(f'measurements/opd_2024-04-30T09:44:44Z/opd_freq_*_hz.csv')

# initialise arrays to store Bode plot data
freqs = np.zeros(len(filenames))
gains = np.zeros(len(filenames), dtype=complex)

for i, filename in enumerate(filenames):
    # get the frequency as a float
    freq_str = filename.split('_')[-2]
    freq = float(freq_str)
    print(f'Frequency: {freq}')

    # read data from the file
    data = pd.read_csv(filename)

    t = data['Time (s)']
    setpoint = data['Setpoint (nm)']
    measurement = data['Measurement (nm)']
    control_signal = data['Control signal (nm)']
    dither_signal = data['Dither signal (nm)']

    # plot the measurement and dither signal
    # fig, ax = plt.subplots()
    # ax.plot(t, measurement, label='Measurement')
    # ax.plot(t, dither_signal, label='Dither signal')
    # ax.set_xlabel('Time (s)')
    # ax.set_ylabel('OPD (nm)')
    # ax.legend()
    # plt.show()


    # find the power of the dither signal and the measurement signal at frequency `freq`
    # using the single-point discrete Fourier transform
    power_dither = np.sum(dither_signal * np.exp(-2j * np.pi * freq * t)) / len(t)
    power_measurement = np.sum(measurement * np.exp(-2j * np.pi * freq * t)) / len(t)
    print(f'\tPower of dither signal: {np.abs(power_dither)}')
    print(f'\tPower of measurement: {np.abs(power_measurement)}')
    print(f'\tGain: {np.abs(power_dither/power_measurement)}, Phase: {np.angle(power_dither/power_measurement)}')

    freqs[i] = freq
    gains[i] = power_dither/power_measurement

# sort ascending frequency for plotting
order = np.argsort(freqs)
freqs = freqs[order]
gains = gains[order]

abs_gain = np.abs(gains)
phase = np.angle(gains)

# unwrap phase
for i in range(1, len(phase)):
    while phase[i] - phase[i-1] < -np.pi:
        phase[i] += 2*np.pi
    while phase[i] - phase[i-1] > np.pi:
        phase[i] -= 2*np.pi

# Bode plot
fig, ax = plt.subplots(2, 1, figsize=(6, 8), sharex=True)
ax[0].loglog(freqs, abs_gain)
ax[0].set_ylabel('Gain')
ax[1].semilogx(freqs, phase)
ax[1].set_ylabel('Phase')
ax[1].set_xlabel('Frequency (Hz)')
plt.show()


