#pragma once

#include "../mLibInclude.h"
#include <vector>

ml::mat4f iterative_closest_point(std::vector<ml::vec3f> & points_a, std::vector<ml::vec3f> & points_b);