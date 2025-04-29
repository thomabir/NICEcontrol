#pragma once

#include "MetrologyReader.hpp"

struct Workers {
  MetrologyReader metrology_reader;

  Workers(SharedResources &ressources) : metrology_reader(ressources.metrology) {}
};
