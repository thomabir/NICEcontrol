"""Classes and convenience functions to deal with FIR filters."""

import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as sig
from matplotlib.ticker import EngFormatter


class FirFilter:
    """Base class for an FIR filter with a given set of coefficients.

    Attributes:
    filter_coeffs: array of filter coefficients
    sampling_rate: sampling rate of the filter
    signal_bits: number of bits for fixed point representation
    num_taps: number of filter coefficients
    """

    def __init__(self, filter_coeffs, sampling_rate, signal_bits):
        self.filter_coeffs = filter_coeffs
        self.sampling_rate = sampling_rate
        self.signal_bits = signal_bits
        self.num_taps = len(filter_coeffs)

    def get_transfer_function(self, n_points=4096):
        """Compute the frequency response of the filter at n_points frequencies."""
        f, h = sig.freqz(self.filter_coeffs, fs=self.sampling_rate, worN=n_points)
        return f, h

    def print_parameters(self, conv_to_fixed_point=True):
        """Print some parameters of the filter."""
        if conv_to_fixed_point:
            filter_coeffs = self.to_fixed_point()
            print(f"Signal bits: {self.signal_bits}")
        else:
            filter_coeffs = self.filter_coeffs
        print(f"Number of coefficients: {self.num_taps}")
        print(f"Sampling rate: {self.sampling_rate} Hz")
        print(f"Filter coefficients: {array_to_c_str(filter_coeffs)}")

    def to_fixed_point(self):
        """Return the filter coefficients in fixed point representation."""
        return float_to_fixed(self.filter_coeffs, self.signal_bits)


def array_to_c_str(array):
    """Convert an array into a string in C/C++/Verilog format, for convenient copy-pasting.

    Example:
    array = np.array([1, 2, 3])
    array_to_c_str(array)
    Output: "{1, 2, 3}"
    """
    coeffs_str = str.join(", ", [str(element) for element in array])
    return f"{{{coeffs_str}}}"


def float_to_fixed(x, n_bits=16):
    """Converts a floating point number in [-1, 1] to a signed fixed point number with n_bits bits.

    The range [-1, 1] gets mapped to the range [-2^(n_bits-1)-1, 2^(n_bits-1)-1].
    """
    x_fixed = x * (2 ** (n_bits - 1) - 1)

    # Convert to integer
    if isinstance(x_fixed, float):
        x_fixed = int(x_fixed)
    elif isinstance(x_fixed, np.ndarray):
        x_fixed = x_fixed.astype(int)
    else:
        raise ValueError("x must be a float or a numpy array")

    return x_fixed


def plot_bode(f, h, title):
    """Plot the Bode plot (magnitude and phase) of a complex transfer function."""
    formatter_hz = EngFormatter(unit="Hz")

    _, ax = plt.subplots()
    ax.set_title(title)

    # First axis: magnitude
    magnitude = abs(h)
    ax.plot(f, magnitude, color="C0")
    ax.set_ylabel("Gain", color="C0")
    ax.tick_params(axis="y", colors="C0")

    # Second axis: phase converted to degrees
    ax2 = ax.twinx()
    phase = np.angle(h, deg=True)
    ax2.plot(f, phase, color="C1")
    ax2.set_ylim(-180, 180)
    ax2.set_ylabel("Phase (deg)", color="C1")
    ax2.tick_params(axis="y", colors="C1")

    # phase labels in multiples of 45 degrees
    ax2.set_yticks([-180, -135, -90, -45, 0, 45, 90, 135, 180])

    ax.xaxis.set_major_formatter(formatter_hz)

    ax.set_xlabel("Frequency")
    plt.show()


class HilbertTransformer(FirFilter):
    """Class for an FIR Hilbert transformer.

    A Hilbert transformer shifts the phase of the input signal by 90 degrees.
    To be causal, the Hilbert transformer also adds a delay of (num_taps-1)/2 samples to the signal.
    """

    def __init__(self, num_taps, sampling_rate, signal_bits):
        assert num_taps % 2 == 1, "Hilbert transformer must have an odd number of taps"
        self.num_taps = num_taps
        coeffs = self.generate_coefficients()
        super().__init__(coeffs, sampling_rate, signal_bits)

    def generate_coefficients(self):
        """Generate the filter coefficients of the Hilbert transformer.

        Source for equation: https://link.springer.com/chapter/10.1007/978-3-030-49256-4_11#Equ6
        """
        N = self.num_taps
        coeffs = np.zeros((N))
        idxs = np.arange(N) - (N - 1) / 2  # indices are centered around 0

        non_zero = np.where(
            idxs
        )  # mask for non-zero indices, avoiding division by zero

        coeffs[non_zero] = (1 - np.cos(idxs[non_zero] * np.pi)) / (
            idxs[non_zero] * np.pi
        )

        # Reduce passband ripple by applying a Blackman window
        coeffs = coeffs * np.blackman(N)

        return coeffs


class HilbertDelayLine(FirFilter):
    """Class for the FIR delay line to match a Hilbert transformer with N taps."""

    def __init__(self, num_taps, sampling_rate, signal_bits):
        assert (
            num_taps % 2 == 1
        ), "The Hilbert delay line must have an odd number of taps"
        self.num_taps = num_taps
        coeffs = self.generate_coefficients()
        super().__init__(coeffs, sampling_rate, signal_bits)

    def generate_coefficients(self):
        """Generate the filter coefficients of a delay by (num_taps-1)/2 samples.

        Example: for num_taps=5, coeffs = [0, 0, 1, 0, 0].
        """
        coeffs = np.zeros((self.num_taps))
        coeffs[int((self.num_taps - 1) / 2)] = 1
        return coeffs
