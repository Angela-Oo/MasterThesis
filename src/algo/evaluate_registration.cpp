#include "stdafx.h"
#include "evaluate_registration.h"
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

class Plane
{
public:
	Point point;
	Vector normal;
public:
	Plane(Point p, Vector n)
		: point(p)
		, normal(n)
	{}
};

SurfaceMesh::Point pointOnPlane(Plane plane, SurfaceMesh::Point point)
{
	auto normal = plane.normal;
	Vector direction = point - plane.point;
	auto angle = CGAL::scalar_product(normal, direction);
	auto h = sin(angle) * std::sqrt(direction.squared_length());

	if (isnan(h))
		std::cout <<"fuu h" << std::endl;
	auto point_on_plane = point - normal * h;
	return point_on_plane;
}


std::vector<float> evaluate_distance_error(std::vector<std::pair<Point, Point>> nearest_points)
{
	std::vector<float> distances;
	for (auto p : nearest_points) {
		auto vector = p.first - p.second;
		float distance = sqrt(vector.squared_length());
		if (isnan(distance) || isinf(distance))
			std::cout << "bad" << std::endl;
		distances.push_back(distance);
	}
	return distances;
}



float area(Point point_a, Point point_b, Point point_c)
{
	auto c = point_b - point_a;
	auto b = point_c - point_a;
	assert(c.squared_length() != 0.f && b.squared_length() != 0.f);

	float angle = CGAL::scalar_product(c, b);
	float b_length = sqrt(b.squared_length());
	float hb = sin(angle) * b_length;
	float area = (hb * b_length) / 2.f;
	return area;
}

Point ErrorEvaluation::barycentricCoordinates(Point point_a, Point point_b, Point point_c, Point point_on_triangle)
{
	auto area_u = area(point_on_triangle, point_a, point_b);
	auto area_v = area(point_on_triangle, point_b, point_c);
	auto area_w = area(point_on_triangle, point_c, point_a);
	auto area_face = area(point_a, point_b, point_c);

	assert(area_face != 0.);
	float u = area_u / area_face;
	float v = area_v / area_face;
	float w = area_w / area_face;
	return Point(u, v, w);
}


SurfaceMesh::Point ErrorEvaluation::getNearestPointOnSurface(Point & point)
{
	auto nn = _nn_search->search(point);
	for (Neighbor_search::iterator it = nn.begin(); it != nn.end(); ++it) {
		auto distance = std::sqrt(it->second);
		auto vertex_handle = it->first;
		auto vertex = _reference_mesh.point(vertex_handle);
		auto he_handle = _reference_mesh.halfedge(vertex_handle);

		for (auto face : _reference_mesh.faces_around_target(he_handle)) {
			if (_reference_mesh.has_valid_index(face)) {
				auto normal = _fnormal[face];

				auto point_on_plane = pointOnPlane(Plane(vertex, normal), point);

				std::vector<Point> face_points;
				for (auto fv : _reference_mesh.vertices_around_face(_reference_mesh.halfedge(face)))
				{
					face_points.push_back(_reference_mesh.point(fv));
				}
				if (face_points.size() == 3) {
					auto barycentric_coordinates = barycentricCoordinates(face_points[0], face_points[1], face_points[2], point_on_plane);
					float relative_area = barycentric_coordinates.x() + barycentric_coordinates.y() + barycentric_coordinates.z();
					if (relative_area <= 1) {
						return point_on_plane;
					}
				}
			}
		}
		return vertex;
	}
	return point;
}


std::vector<std::pair<Point, Point>> ErrorEvaluation::evaluate_error(const SurfaceMesh & mesh)
{
	std::vector<std::pair<Point, Point>> nearest_points;
	for(auto p : mesh.vertices()) {
		auto point = mesh.point(p);
		auto nearest_point = getNearestPointOnSurface(point);
		if ((nearest_point - CGAL::ORIGIN).squared_length() == 0.f || isnan(nearest_point[0]) || isnan(nearest_point[1]))
			std::cout << "help" << std::endl;
		nearest_points.push_back(std::make_pair(point, nearest_point));
	}
	return nearest_points;
}

ErrorEvaluation::ErrorEvaluation(const SurfaceMesh & reference_mesh)
	: _reference_mesh(reference_mesh)
{
	_nn_search = std::make_unique<NearestNeighborSearch>(_reference_mesh);
	auto fnormals = _reference_mesh.add_property_map<face_descriptor, Vector>("f:normals", Vector(0, 0, 0));	
	CGAL::Polygon_mesh_processing::compute_face_normals(_reference_mesh, fnormals.first);
	_fnormal = fnormals.first;
}