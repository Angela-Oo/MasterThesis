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
public:
	double median();
	Point getPoint(vertex_descriptor);
	Vector getNormal(vertex_descriptor);
	std::pair<bool, vertex_descriptor> correspondingPoint(Point point, Vector normal);
	FindCorrespondingPoints(const SurfaceMesh & mesh, double max_allowed_distance = 0.05, double max_normal_angle_deviation = 30.);
};