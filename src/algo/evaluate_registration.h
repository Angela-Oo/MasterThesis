#pragma once

#include "../mLibInclude.h"

typedef ml::TriMeshf Mesh;
std::vector<ml::vec3f> evaluate_error(const Mesh & result, const Mesh & reference_mesh);

std::vector<float> evaluate_distance_error(const Mesh& points_a, const std::vector<ml::vec3f>& points_b);