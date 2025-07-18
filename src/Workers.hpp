#pragma once

#include "EthercatTapReader.hpp"
#include "MetrologyReader.hpp"
#include "SharedResources.hpp"

struct Workers {
  MetrologyReader metrology_reader;
  EthercatTapReader ethercat_reader;

  Workers(SharedResources &resources) : metrology_reader(resources.metrology), ethercat_reader(resources.ethercat) {}
};
