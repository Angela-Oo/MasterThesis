#include "error_evaluation.h"
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include "barycentric_coordinates.h"

namespace Registration {


std::vector<double> evaluate_distance_error(std::vector<std::pair<Point, Point>> nearest_points)
{
	std::vector<double> distances;
	for (auto p : nearest_points) {
		auto vector = p.first - p.second;
		double distance = sqrt(vector.squared_length());
		if (isnan(distance) || isinf(distance))
			std::cout << "bad" << std::endl;
		distances.push_back(distance);
	}
	return distances;
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
					double relative_area = barycentric_coordinates.x() + barycentric_coordinates.y() + barycentric_coordinates.z();
					if (relative_area <= 1.) {
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
	for (auto p : mesh.vertices()) {
		auto point = mesh.point(p);
		auto nearest_point = getNearestPointOnSurface(point);
		if ((nearest_point - CGAL::ORIGIN).squared_length() == 0.f || isnan(nearest_point[0]) || isnan(nearest_point[1]))
			std::cout << "help" << std::endl;
		nearest_points.push_back(std::make_pair(point, nearest_point));
	}
	return nearest_points;
}

RegistrationError ErrorEvaluation::errorEvaluation(const SurfaceMesh & mesh)
{
	std::vector<vertex_descriptor> v_ids;
	std::vector<double> distances;
	//std::vector<std::pair<Point, Point>> nearest_points;
	for (auto p : mesh.vertices()) {
		auto point = mesh.point(p);
		auto nearest_point = getNearestPointOnSurface(point);
		auto vector = point - nearest_point;
		double distance = sqrt(vector.squared_length());

		if (isnan(distance) || isinf(distance))
			std::cout << "bad" << std::endl;

		v_ids.push_back(p);
		distances.push_back(distance);
	}
	return RegistrationError(v_ids, distances);
}


ErrorEvaluation::ErrorEvaluation(const SurfaceMesh & reference_mesh)
	: _reference_mesh(reference_mesh)
{
	_nn_search = std::make_unique<NearestNeighborSearch>(_reference_mesh);
	auto fnormals = _reference_mesh.add_property_map<face_descriptor, Vector>("f:normals", Vector(0, 0, 0));
	CGAL::Polygon_mesh_processing::compute_face_normals(_reference_mesh, fnormals.first);
	_fnormal = fnormals.first;
}

}