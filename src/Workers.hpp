#pragma once

#include "BeamController.hpp"
#include "EthercatTapReader.hpp"
#include "MetrologyReader.hpp"
#include "SharedResources.hpp"

struct Workers {
  MetrologyReader metrology_reader;
  EthercatTapReader ethercat_reader;
  BeamController beam_controller;

  Workers(SharedResources &resources)
      : metrology_reader(resources.metrology), ethercat_reader(resources.ethercat), beam_controller(resources.piezos) {}
};
