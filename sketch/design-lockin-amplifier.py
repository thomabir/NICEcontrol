import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as sig
from matplotlib import rcParams
from matplotlib.ticker import EngFormatter


def print_verilog_array(array):
    """Prints an array in Verilog format, e.g. {10, 3, 2}"""

    print("{", end="")
    for i, element in enumerate(array):
        print(f"{element}", end="")

        # unless it's the last element, print ", "
        if i != len(array) - 1:
            print(", ", end="")
    print("};")


def float_to_fixed(x, n_bits=16):
    """Converts a floating point number to a signed fixed point number with n_bits bits.

    The range [-1, 1] gets mapped to the range [-2^(n_bits-1)-1, 2^(n_bits-1)-1].
    Note that the most negative signed integer gets mapped to slightly less than -1, because of the assymmetry inherent to standard signed integers.
    """
    return int(x * (2 ** (n_bits - 1) - 1))


def float_to_fixed_arr(arr, n_bits=16):
    """Converts a floating point array to a signed fixed point array with n_bits bits.

    The range [-1, 1] gets mapped to the range [-2^(n_bits-1)-1, 2^(n_bits-1)-1].
    """
    return [int(x * (2 ** (n_bits - 1) - 1)) for x in arr]


rcParams.update({"figure.autolayout": True})
formatter_hz = EngFormatter(unit="Hz")


# Implement a discrete Hilbert transformer as an FIR filter


# Generate the filter coefficients of the Hilbert transformer
# Source: https://link.springer.com/chapter/10.1007/978-3-030-49256-4_11#Equ6

sampling_rate = 128000  # Hz
target_frequency = 10000  # Hz

N = 13  # number of filter stages
# (goal: gain of at least 0.99 at 500 Hz)

filter_coeffs = np.zeros((N))
filter_index = np.arange(N) - (N - 1) / 2
for n, index in enumerate(filter_index):
    if index == 0:
        filter_coeffs[n] = 0
    else:
        filter_coeffs[n] = 1 / (index * np.pi) * (1 - np.cos(index * np.pi))

# apply a blackman window to the filter coefficients
filter_coeffs = filter_coeffs * np.blackman(N)

# generate filter coefficients of a delay by (N-1)/2 samples
delay_coeffs = np.zeros((N))
delay_coeffs[int((N - 1) / 2)] = 1

# offset the filter index so that it starts at 0
filter_index = filter_index - filter_index[0]

# Plot the filter coefficients
fig, ax = plt.subplots()
ax.set_title("Filter coefficients of the Hilbert transformer")
ax.stem(filter_index, filter_coeffs)
ax.set_ylabel("Coefficient value")
ax.set_xlabel("Coefficient index")

# add the number of filter stages to the plot
ax.text(0.05, 0.95, f"Number of filter stages: {N}", transform=ax.transAxes, fontsize=10, verticalalignment="top")

plt.show()
# fig.savefig("fig/hilbert-coeffs.pdf", bbox_inches="tight")


N_plot = 4096


# Find the frequency response of the Hilbert transformer

f_w, h_causal = sig.freqz(filter_coeffs, fs=sampling_rate, worN=N_plot)

# Find the frequency response of the delay
f_w, h_delay = sig.freqz(delay_coeffs, fs=sampling_rate, worN=N_plot)


# a function to plot the Bode plot of a filter
def plot_bode(f, h, title, filename, annotate=False):
    fig, ax = plt.subplots()
    ax.set_title(title)

    # remove 0 Hz from the plot
    f = f[1:]
    h = h[1:]

    # First axis: magnitude
    magnitude = abs(h)
    ax.plot(f, magnitude)

    # Second axis: phase converted to degrees
    ax2 = ax.twinx()
    phase = np.angle(h, deg=True)
    ax2.plot(f, phase, color="C1")
    # ylim: -180 to 180 degrees
    ax2.set_ylim(-180, 180)

    # if annotate is True, add annotate the plot with gain and phase at 10 kHz
    if annotate:
        # evaluate which index in f is closest to target frequency
        index = np.argmin(np.abs(f - target_frequency))

        # add multi-line text to the center of the plot
        ax.text(
            0.5,
            0.5,
            f"At {f[index]:.2f} Hz:\nGain: {magnitude[index]:.4f}\nPhase: {phase[index]:.4f} deg",
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment="center",
            horizontalalignment="center",
        )

    # label the phase axis in multiples of 45 degrees
    ax2.set_yticks([-180, -135, -90, -45, 0, 45, 90, 135, 180])

    ax.xaxis.set_major_formatter(formatter_hz)
    ax.set_ylabel("Gain")
    ax2.set_ylabel("Phase (deg)")
    ax.set_xlabel("Frequency")
    # fig.savefig(filename + ".pdf", bbox_inches="tight")
    plt.show()


# Plot the Bode plot of the Hilbert transformer
plot_bode(f_w, h_causal, "Bode plot of the Hilbert transformer", "fig/hilbert-bode-causal")

# Bode plot of the delay line
plot_bode(f_w, h_delay, "Bode plot of the delay line", "fig/bode-delay")

# Plot the Bode plot of the Hilbert transformer minus the delay
h_differential = h_causal / h_delay
plot_bode(
    f_w,
    h_differential,
    "Bode plot of the Hilbert transformer minus the delay",
    "fig/hilbert-bode-differential",
    annotate=True,
)


# How many bits do we need to represent the weights?
# minNumber = np.min(np.abs(filter_coeffs))
# print(-np.log(minNumber)/np.log(2))

# Scale from normalized floats to fixed point
# signal_bits = 24
# coeff_scaling_factor = 2**(signal_bits-1)

print("number of filter stages: %i" % len(filter_coeffs))


# turn the above into a function and add a docstring
def print_coeffs(coeffs):
    """Print the filter coefficients in a format that can be copy-pasted into julia code.

    Format of the output: coeffs = [coeff0, coeff1, coeff2, ..., coeffN]
    """
    print("coeffs = [", end="")
    for i, coeff in enumerate(coeffs):
        print(f"{coeff}", end="")
        if i < len(coeffs) - 1:
            print(", ", end="")
    print("]")


# print coefficients of the Hilbert transformer and the delay line
# print_coeffs(filter_coeffs)
# print_coeffs(delay_coeffs)


# transform to 24 bit fixed point
signal_bits = 24
filter_coeffs = float_to_fixed_arr(filter_coeffs, signal_bits)
delay_coeffs = float_to_fixed_arr(delay_coeffs, signal_bits)

# print coefficients of the Hilbert transformer and the delay line
print(f"Number of coefficients: {len(filter_coeffs)}")

print("Hilbert transformer coefficients:")
print_verilog_array(filter_coeffs)

print("Delay line coefficients:")
print_verilog_array(delay_coeffs)
