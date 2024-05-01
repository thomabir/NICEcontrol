import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import glob


def single_point_dft(x, freq, t):
    return np.sum(x * np.exp(-2j * np.pi * freq * t)) / len(t)


def freq_analysis(t, y_in, y_out, freq):
    power_in = single_point_dft(y_in, freq, t)
    power_out = single_point_dft(y_out, freq, t)
    gain = power_out / power_in
    return gain


def unwrap_phase(phases):
    for i in range(1, len(phases)):
        while phases[i] - phases[i - 1] < -np.pi:
            phases[i] += 2 * np.pi
        while phases[i] - phases[i - 1] > np.pi:
            phases[i] -= 2 * np.pi
    return phases


def complex_gain_to_mag_phase(gains):
    return np.abs(gains), unwrap_phase(np.angle(gains))


def freq_analysis_full(filenames_arr, input_signal_str, output_signal_str):
    # initialise arrays to store Bode plot data
    freqs = np.zeros(len(filenames_arr))
    gains = np.zeros(len(filenames_arr), dtype=complex)

    # get freqs
    for i, filename in enumerate(filenames):
        # get name of file, not directory
        freqs_file = filename.split("/")[-1]
        freq_str = freqs_file.split("_")[-2]
        freqs[i] = float(freq_str)

    for i, filename in enumerate(filenames_arr):

        # read data from the file
        data = pd.read_csv(filename)

        # Time (s),Measurement (nm),Setpoint (nm),Dither signal (nm),Controller input (nm),Controller output (nm),Actuator command (nm)
        t = data["Time (s)"]
        y_in = data[input_signal_str]
        y_out = data[output_signal_str]

        gains[i] = freq_analysis(t, y_in, y_out, freqs[i])

    # sort ascending frequency for plotting
    order = np.argsort(freqs)
    freqs = freqs[order]
    gains = gains[order]

    abs_gain = np.abs(gains)
    phase = unwrap_phase(np.angle(gains))

    return freqs, gains


# # Controller frequency response analysis
# filenames = glob.glob("freq_controller/*_hz.csv")
# f_con, gg_con = freq_analysis_full(
#     filenames, "Controller input (nm)", "Controller output (nm)"
# )

# # Plant frequency response analysis
# filenames = glob.glob("freq_plant/control_*_hz.csv")
# f_plant, gg_plant = freq_analysis_full(
#     filenames, "Actuator command (nm)", "Measurement (nm)"
# )

# # Closed loop dither plant frequency response analysis
# filenames = glob.glob("freq_closed_loop_dither_plant/control_*_hz.csv")
# f_clp, gg_clp = freq_analysis_full(filenames, "Measurement (nm)", "Dither signal (nm)")

# # Closed loop dither setpoint frequency response analysis
# filenames = glob.glob("freq_closed_loop_dither_setpoint/control_*_hz.csv")
# f_cls, gg_cls = freq_analysis_full(filenames, "Dither signal (nm)", "Measurement (nm)")

# # save data to a .csv using pandas: save real and complex parts as separate columns for easier parsing
# df = pd.DataFrame(
#     {
#         "Frequency (Hz)": f_con,
#         "Controller gain real": np.real(gg_con),
#         "Controller gain imag": np.imag(gg_con),
#         "Plant gain real": np.real(gg_plant),
#         "Plant gain imag": np.imag(gg_plant),
#         "Closed loop dither plant gain real": np.real(gg_clp),
#         "Closed loop dither plant gain imag": np.imag(gg_clp),
#         "Closed loop dither setpoint gain real": np.real(gg_cls),
#         "Closed loop dither setpoint gain imag": np.imag(gg_cls),
#     }
# )
# df.to_csv("transfer_function.csv", index=False)

# exit()

# # read data from the file
# data = pd.read_csv("transfer_function.csv")
# f = data["Frequency (Hz)"]
# gg_con = data["Controller gain real"] + 1j * data["Controller gain imag"]
# gg_plant = data["Plant gain real"] + 1j * data["Plant gain imag"]
# gg_clp = data["Closed loop dither plant gain real"] + 1j * data["Closed loop dither plant gain imag"]
# gg_cls = data["Closed loop dither setpoint gain real"] + 1j * data["Closed loop dither setpoint gain imag"]

# # "simulated" closed loop gain based on plant and controller
# # gg_cl_sim = gg_con * gg_plant / (1 + gg_con * gg_plant)
# # g_cl_sim, ph_cl_sim = complex_gain_to_mag_phase(gg_cl_sim)

# # get magnitude and phase from complex gains
# g_con, ph_con = complex_gain_to_mag_phase(gg_con)
# g_plant, ph_plant = complex_gain_to_mag_phase(gg_plant)
# g_clp, ph_clp = complex_gain_to_mag_phase(gg_clp)
# g_cls, ph_cls = complex_gain_to_mag_phase(gg_cls)


# # Bode plot
# fig, ax = plt.subplots(2, 1, figsize=(6, 8), sharex=True)
# ax[0].loglog(f, g_con, label="Controller")
# ax[0].loglog(f, g_plant, label="Plant")
# ax[0].loglog(f, g_cls, label="Closed loop, dither setpoint")
# # ax[0].loglog(f, g_cl_sim, label="Closed loop (simulated)")
# ax[0].set_ylabel("Gain")
# ax[1].semilogx(f, ph_con, label="Controller")
# ax[1].semilogx(f, ph_plant, label="Plant")
# ax[1].semilogx(f, ph_cls, label="Closed loop, dither setpoint")
# # ax[1].semilogx(f, ph_cl_sim, label="Closed loop (simulated)")
# ax[1].set_ylabel("Phase")
# ax[1].set_xlabel("Frequency (Hz)")
# plt.legend()
# plt.show()

# # bode plot of plant dither
# fig, ax = plt.subplots(2, 1, figsize=(6, 8), sharex=True)
# ax[0].loglog(f, g_plant, label="Plant")
# ax[0].loglog(f, g_clp, label="Closed loop, dither plant")
# ax[0].set_ylabel("Gain")
# ax[1].semilogx(f, ph_plant, label="Plant")
# ax[1].semilogx(f, ph_clp, label="Closed loop, dither plant")
# ax[1].set_ylabel("Phase")
# ax[1].set_xlabel("Frequency (Hz)")
# plt.legend()
# plt.show()


# exit()


## Impulse response analysis
filename = f"impulse_response.csv"

# read data from the file
data = pd.read_csv(filename)
t = data["Time (s)"]
measurement = data["Measurement (nm)"]
setpoint = data["Setpoint (nm)"]

# plot
fig, ax = plt.subplots()
ax.plot(t, measurement, label='Measurement')
ax.plot(t, setpoint, label='Setpoint')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (nm)')
ax.legend()
plt.show()
