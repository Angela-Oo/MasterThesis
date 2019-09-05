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


double calculateVariance(const std::vector<double> & values, double mean)
{
	std::vector<double> diff(values.size());
	std::transform(values.begin(), values.end(), diff.begin(), [&mean](double x) { return x - mean; });
	double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	return std::sqrt(sq_sum / values.size());
}

RegistrationError::RegistrationError(std::vector<vertex_descriptor> v_ids, std::vector<double> errors)
	: _v_ids(v_ids)
	, _errors(errors)
{
	double sum = 0.;
	for(auto e : _errors)
	{
		if (e < _min)
			_min = e;
		if (e > _max)
			_max = e;
		sum += e;
	}
	if (!_errors.empty()) {
		_mean = sum / _errors.size();
		_variance = calculateVariance(_errors, _mean);
		_median = _errors[floor(_errors.size() / 2.)];
	}
}


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



size_t RegistrationError::size() const
{
	return _errors.size();	
}

vertex_descriptor RegistrationError::v(size_t i) const
{
	return _v_ids[i];
}

double RegistrationError::error(size_t i) const
{
	return _errors[i];
}

double RegistrationError::mean() const
{
	return _mean;
}

double RegistrationError::variance() const
{
	return _variance;
}

double RegistrationError::median() const
{
	return _median;
}

double RegistrationError::min() const
{
	return _min;
}

double RegistrationError::max() const
{
	return _max;
}


double area(Point point_a, Point point_b, Point point_c)
{
	auto c = point_b - point_a;
	auto b = point_c - point_a;
	assert(c.squared_length() != 0.f && b.squared_length() != 0.f);

	double angle = CGAL::scalar_product(c, b);
	double b_length = sqrt(b.squared_length());
	double hb = sin(angle) * b_length;
	double area = (hb * b_length) / 2.f;
	return area;
}

Point ErrorEvaluation::barycentricCoordinates(Point point_a, Point point_b, Point point_c, Point point_on_triangle)
{
	auto area_u = area(point_on_triangle, point_a, point_b);
	auto area_v = area(point_on_triangle, point_b, point_c);
	auto area_w = area(point_on_triangle, point_c, point_a);
	auto area_face = area(point_a, point_b, point_c);

	assert(area_face != 0.);
	double u = area_u / area_face;
	double v = area_v / area_face;
	double w = area_w / area_face;
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
	for(auto p : mesh.vertices()) {
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