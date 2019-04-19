#pragma once

#include "../mLibInclude.h"

std::vector<ml::vec3f> evaluate_error(std::vector<ml::vec3f> result, const ml::TriMeshf & reference_mesh);

std::vector<float> evaluate_distance_error(const std::vector<ml::vec3f>& points_a, const std::vector<ml::vec3f>& points_b);