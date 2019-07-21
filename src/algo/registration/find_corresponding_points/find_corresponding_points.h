#pragma once

#include "stdafx.h"

#include "algo/surface_mesh/nearest_neighbor_search.h"


double angle_between_to_vectors_in_rad(Vector vector_a, Vector vector_b);

class FindCorrespondingPoints
{
	SurfaceMesh _mesh;
	std::unique_ptr<NearestNeighborSearch> _nn_search;
	double _max_normal_angle_deviation;
	double _median_distance;
	int _k;
	double _allowed_multiple_of_median_distance;
	double _min_allowed_distance;
public:
	double median();
	Point getPoint(vertex_descriptor);
	Vector getNormal(vertex_descriptor);
	std::pair<bool, vertex_descriptor> correspondingPoint(Point point, Vector normal);
	FindCorrespondingPoints(const SurfaceMesh & mesh,
							double initial_max_allowed_distance = 0.1,							
							double max_normal_angle_deviation = 45.,
							double allowed_multiple_of_median_distance = 10.,
							double min_allowed_distance = 0.0001);
};