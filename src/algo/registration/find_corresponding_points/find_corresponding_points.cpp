#include "stdafx.h"
#include "find_corresponding_points.h"


double angle_between_to_vectors_in_rad(Vector vector_a, Vector vector_b)
{
	auto dot_product = CGAL::scalar_product(vector_a, vector_b);
	auto angle = acos(dot_product);
	return angle;
}

double FindCorrespondingPoints::median()
{
	return _median_distance;
}


std::pair<bool, Point> FindCorrespondingPoints::correspondingPoint(Point point, Vector normal)
{
	auto s = _nn_search.search(point, _k);

	std::vector<std::pair<vertex_descriptor, std::pair<double, double>>> valid_point_with_angle_and_distance;
	auto vertex_normals = _mesh.property_map<vertex_descriptor, Direction>("v:normal").first;

	for (Neighbor_search::iterator it = s.begin(); it != s.end(); ++it) {
		auto distance = std::sqrt(it->second);
		auto v = it->first;

		//auto vertex = _mesh.point(v);
		auto angle = angle_between_to_vectors_in_rad(normal, vertex_normals[v].vector());

		if (distance < 100. * median()) {
			_median_distance = (_median_distance * 10000. + distance) / 10001.;
		}
		float k_median = 10. * median();
		bool valid_angle = angle < _max_normal_angle_deviation;
		bool valid_distance = distance < k_median;

		if (valid_angle && valid_distance) {
			valid_point_with_angle_and_distance.push_back(std::make_pair(v, std::make_pair(angle, distance)));
		}
	}

	if (valid_point_with_angle_and_distance.empty()) {
		return std::make_pair(false, point);
	}
	else {
		auto sort_by_angle = [](const std::pair<vertex_descriptor, std::pair<double, double>> & rhs, const std::pair<vertex_descriptor, std::pair<double, double>> & lhs) { return rhs.second.second < lhs.second.second; };
		std::sort(valid_point_with_angle_and_distance.begin(), valid_point_with_angle_and_distance.end(), sort_by_angle);
		return std::make_pair(true, _mesh.point(valid_point_with_angle_and_distance[0].first));
	}
}

FindCorrespondingPoints::FindCorrespondingPoints(SurfaceMesh mesh, double max_allowed_distance, double max_normal_angle_deviation)
	: _mesh(mesh)
	, _k(10)
	, _nn_search(mesh)
	, _max_normal_angle_deviation(max_normal_angle_deviation)
	, _median_distance(max_allowed_distance)
{
	_max_normal_angle_deviation = ml::math::degreesToRadians(max_normal_angle_deviation);
	std::cout << "allowed angle " << _max_normal_angle_deviation << " allowed distance " << _median_distance << std::endl;
}