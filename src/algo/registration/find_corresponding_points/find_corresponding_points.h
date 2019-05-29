#pragma once

#include "stdafx.h"

#include "algo/registration/deformation_graph/nearest_neighbor_search.h"


double angle_between_to_vectors_in_rad(Vector vector_a, Vector vector_b);

class FindCorrespondingPoints
{
	SurfaceMesh _mesh;
	std::unique_ptr<NearestNeighborSearch> _nn_search;
	double _max_normal_angle_deviation;
	double _median_distance;
	double _k;
public:
	double median();
	std::pair<bool, Point> correspondingPoint(Point point, Vector normal);
	FindCorrespondingPoints(const SurfaceMesh & mesh, double max_allowed_distance = 0.05, double max_normal_angle_deviation = 30.);
};