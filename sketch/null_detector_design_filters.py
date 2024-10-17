"""Design the filters for the Null Detector."""

import numpy as np
from fir_filter import HilbertDelayLine, HilbertTransformer, plot_bode
from matplotlib import rcParams


def design_hilbert_transformer():
    """Design the Hilbert transformer and the delay line for the Lock-in Amplifier."""

    # Filter parameters
    sampling_rate = 128000  # Hz
    target_frequency = 20000  # Hz
    N = 15  # number of filter stages (must be odd)
    signal_bits = 24  # number of bits for fixed point representation

    # Create the Hilbert transformer and delay line
    h1 = HilbertTransformer(N, sampling_rate, signal_bits)
    d1 = HilbertDelayLine(N, sampling_rate, signal_bits)

    # Get the transfer functions of the filters
    f, h_hilbert = h1.get_transfer_function()
    f, h_delay = d1.get_transfer_function()
    h_diff = h_hilbert / h_delay

    # Generate the bode plots
    plot_bode(f, h_hilbert, "Bode plot of Hilbert transformer")
    plot_bode(f, h_delay, "Bode plot of the delay line")
    plot_bode(f, h_diff, "Bode plot of the h_hilbert / h_delay")

    # check gain at target frequency
    index = np.argmin(np.abs(f - target_frequency))
    magnitude = abs(h_diff)
    phase = np.angle(h_diff, deg=True)

    # print the filter coefficients
    print("Hilbert transformer:")
    print(
        f"At {f[index]:.2f} Hz: Gain = {magnitude[index]:.4f}, Phase = {phase[index]:.4f} deg"
    )
    h1.print_parameters(conv_to_fixed_point=False)
    print("\nDelay line:")
    d1.print_parameters(conv_to_fixed_point=False)


if __name__ == "__main__":
    # Plot settings
    rcParams.update({"figure.autolayout": True})

    # Design the filters
    design_hilbert_transformer()
