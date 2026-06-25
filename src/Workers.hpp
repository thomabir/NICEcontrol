#pragma once

#include "BeamController.hpp"
#include "EthercatAdsReader.hpp"
#include "MetrologyReader.hpp"
#include "SharedResources.hpp"

struct Workers {
  MetrologyReader metrology_reader;
  EthercatAdsReader ethercat_ads_reader;
  ControlManager beam_controller;

  Workers(SharedResources &resources)
      : metrology_reader(resources.metrology),
        ethercat_ads_reader(resources.ethercat_ads),
        beam_controller(resources) {}
};
