#pragma once

#include "mesh/mesh_definition.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"

std::vector<double> evaluate_distance_error(std::vector<std::pair<Point, Point>> nearest_points);

double calculateVariance(const std::vector<double> & values, double mean);

class RegistrationError
{
private:
	double _mean{ 0. };
	double _variance{0.};
	double _median{ 0. };
	double _min{INFINITY};
	double _max{0.};
	std::vector<vertex_descriptor> _v_ids;
	std::vector<double> _errors;
public:
	size_t size() const;
	vertex_descriptor v(size_t i) const;
	double error(size_t i) const;
	double mean() const;
	double variance() const;
	double median() const;
	double min() const;
	double max() const;	
public:	
	RegistrationError(std::vector<vertex_descriptor> v_ids, std::vector<double> errors);
};

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
	RegistrationError errorEvaluation(const SurfaceMesh & mesh);
public:
	ErrorEvaluation(const SurfaceMesh & reference_mesh);
};