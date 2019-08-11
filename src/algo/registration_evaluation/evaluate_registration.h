#pragma once

#include "mesh/mesh_definition.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"

std::vector<float> evaluate_distance_error(std::vector<std::pair<Point, Point>> nearest_points);


class ErrorEvaluation
{
	SurfaceMesh _reference_mesh;
	std::unique_ptr<NearestNeighborSearch> _nn_search;
	SurfaceMesh::Property_map<face_descriptor, Vector> _fnormal;
private:
	Point barycentricCoordinates(Point point_a, Point point_b, Point point_c, Point point_on_triangle);
	Point getNearestPointOnSurface(SurfaceMesh::Point & point);
public:
	std::vector<std::pair<Point, Point>> evaluate_error(const SurfaceMesh & mesh);
public:
	ErrorEvaluation(const SurfaceMesh & reference_mesh);
};