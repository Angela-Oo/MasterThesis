#pragma once

#include "mLibInclude.h"
#include <vector>

std::vector<double> nodeDistanceWeighting(const ml::vec3f & point, const std::vector<ml::vec3f>& node_positions);