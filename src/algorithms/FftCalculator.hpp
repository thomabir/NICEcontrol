#pragma once

#include <fftw3.h>  // Fast Fourier Transform

#include "data/ScrollingBufferT.hpp"

template <typename T, typename U>
class FFT_calculator {
 public:
  int size;
  double sampling_rate;  // Hz
  ScrollingBufferT<T, U> *measurement_buffer;
  double *output_power;
  double *output_freq;
  fftw_complex *in, *out;
  fftw_plan fft_plan;

  FFT_calculator(int n_points, double sampling_rate_hz, ScrollingBufferT<T, U> *buffer, double *power, double *freq)
      : size(n_points),
        sampling_rate(sampling_rate_hz),
        measurement_buffer(buffer),
        output_power(power),
        output_freq(freq) {
    in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * size);
    out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * size);
    fft_plan = fftw_plan_dft_1d(size, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
  }

  void calculate() {
    // if there are at least size points in the measurement_buffer, copy the size latest points to in
    // Note: the most recent point is at measurement_buffer.Data[measurement_buffer.Offset].
    // While looping over the buffer, if you reach measurement_buffer[0], you need to continue at
    // measurement_buffer.Data[measurement_buffer.MaxSize-1]
    int offset = measurement_buffer->Offset;
    static int max_size = measurement_buffer->MaxSize;

    if (measurement_buffer->Data.size() == max_size) {
      for (int i = 0; i < size; i++) {
        in[i][0] = measurement_buffer->Data[(offset - size + i + max_size) % max_size].value;
        in[i][1] = 0.0f;
      }
    }

    // execute fft
    fftw_execute(fft_plan);

    // calculate power spectral density (only the real half)
    for (int i = 0; i < size / 2; i++) {
      output_power[i] = out[i][0] * out[i][0] + out[i][1] * out[i][1];
      output_power[i] = output_power[i] * 2 / size;             // normalize by size to get periodogram
      output_power[i] = sqrt(output_power[i] / sampling_rate);  // convert to amplitude (e.g. V/sqrt(Hz))
    }

    // set first element to 0 (DC)
    output_power[0] = 0.0f;

    // find the frequency axis
    for (int i = 0; i < size / 2; i++) {
      output_freq[i] = i * sampling_rate / size;
    }
  }
};
