#pragma once

#include <vector>

#include "core/Whiteboard.hpp"
#include "data/ScrollingBufferT.hpp"

// Plot history. The core owns the streams and this is the copy that ImPlot draws from.
class PlotHistory {
 public:
  explicit PlotHistory(Whiteboard &whiteboard)
      : wb(whiteboard),
        adc_reader(whiteboard.adc.subscribe()),
        plc_reader(whiteboard.plc.subscribe()),
        phot_reader(whiteboard.phot.subscribe()) {
    phot_buffers.resize(kMaxPhotRegions, ScrollingBufferT<double, double>(kPhotHistoryPoints));
  }

  // Every stream is drained in every frame, whether or not its panel is open. A stream that nobody drains overflows.
  void Drain(const Snapshot &snap) {
    Measurement<AdcSample> adc_measurement;
    while (wb.adc.try_pop(adc_reader, adc_measurement)) {
      const AdcSample &adc_sample = adc_measurement.value;
      adc_time = adc_sample.counter;
      for (int channel = 0; channel < 16; channel++) {
        adc_buffers[channel].AddPoint(adc_sample.counter, adc_sample.value[channel]);
      }
    }

    Measurement<PlcSample> plc_measurement;
    while (wb.plc.try_pop(plc_reader, plc_measurement)) {
      const PlcSample &plc_sample = plc_measurement.value;
      plc_time = 1e-9 * static_cast<double>(plc_measurement.time.t_DC);
      dl_pos_buffer.AddPoint(plc_time, plc_sample.dl_pos_um);
      dl_cmd_buffer.AddPoint(plc_time, plc_sample.dl_cmd_um);
      opd_buffer.AddPoint(plc_time, plc_sample.opd_um);
      // The PLC does not send the setpoint back, thus the plot takes the value that the core last sent.
      opd_setpoint_buffer.AddPoint(plc_time, snap.opd.setpoint_um);
      const QpdData &qpd1 = plc_sample.qpd1;
      const QpdData &qpd2 = plc_sample.qpd2;
      const double qpd_values[12] = {qpd1.x1, qpd1.y1, qpd1.i1, qpd1.x2, qpd1.y2, qpd1.i2,
                                     qpd2.x1, qpd2.y1, qpd2.i1, qpd2.x2, qpd2.y2, qpd2.i2};
      for (int i = 0; i < 12; i++) {
        qpd_buffers[i].AddPoint(plc_time, qpd_values[i]);
      }
    }

    Measurement<PhotSample> phot_measurement;
    while (wb.phot.try_pop(phot_reader, phot_measurement)) {
      const PhotSample &phot_sample = phot_measurement.value;
      phot_time = 1e-9 * static_cast<double>(phot_measurement.time.t_DC);
      for (int region = 0; region < kMaxPhotRegions; region++) {
        phot_buffers[region].AddPoint(phot_time, phot_sample.values[region]);
      }
    }
  }

  ScrollingBufferT<int, int> adc_buffers[16];
  int adc_time = 0;
  ScrollingBufferT<double, double> dl_pos_buffer;
  ScrollingBufferT<double, double> dl_cmd_buffer;
  ScrollingBufferT<double, double> opd_buffer;
  ScrollingBufferT<double, double> opd_setpoint_buffer;
  ScrollingBufferT<double, double> qpd_buffers[12];
  double plc_time = 0.0;
  std::vector<ScrollingBufferT<double, double>> phot_buffers;
  double phot_time = 0.0;

 private:
  static constexpr int kPhotHistoryPoints = 20000;

  Whiteboard &wb;
  Consumer *adc_reader;
  Consumer *plc_reader;
  Consumer *phot_reader;
};
