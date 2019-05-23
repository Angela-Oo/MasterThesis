#include "stdafx.h"
#include "find_correspondece_point.h"


double angle_between_to_vectors_in_rad(ml::vec3f vector_a, ml::vec3f vector_b)
{
	auto dot_product = ml::vec3f::dot(vector_a.getNormalized(), vector_b.getNormalized());
	auto angle = acos(dot_product);
	return angle;
}

double FindCorrespondecePoint::median()
{
	return _median;
}

std::pair<bool, ml::vec3f> FindCorrespondecePoint::correspondingPoint(ml::vec3f point, ml::vec3f normal)
{
	auto k_nearest_indices = _nn_search.k_nearest_indices(point, _k);

	std::vector<std::pair<ml::vec3f, std::pair<double, double>>> valid_point_with_angle_and_distance;
	for (auto i : k_nearest_indices) {
		unsigned int i = _nn_search.nearest_index(point);
		auto vertex = _mesh.getVertices()[i];
		
		auto angle = angle_between_to_vectors_in_rad(normal, vertex.normal);
		auto distance = dist(point, vertex.position);

		_median = (_median * 10000. + distance) / 10001.;
		float k_median = 10. * median();

		bool valid_angle = angle < _max_normal_angle_deviation;
		bool valid_distance = distance < k_median;// _max_allowed_distance;


		if (valid_angle && valid_distance) {
			valid_point_with_angle_and_distance.push_back(std::make_pair(vertex.position, std::make_pair(angle, distance)));
		}
	}


	if (valid_point_with_angle_and_distance.empty()) {
		return std::make_pair(false, point);
	}
	else {
		auto sort_by_angle = [](const std::pair<ml::vec3f, std::pair<double, double>> & rhs, const std::pair<ml::vec3f, std::pair<double, double>> & lhs) { return rhs.second.second < lhs.second.second; };
		std::sort(valid_point_with_angle_and_distance.begin(), valid_point_with_angle_and_distance.end(), sort_by_angle);
		return std::make_pair(true, valid_point_with_angle_and_distance[0].first);
	}
}

FindCorrespondecePoint::FindCorrespondecePoint(Mesh mesh, double max_allowed_distance, double max_normal_angle_deviation)
	: _mesh(mesh)
	, _k(10)
	, _nn_search(mesh, 10)
	, _max_allowed_distance(max_allowed_distance)
	, _max_normal_angle_deviation(max_normal_angle_deviation)
	, _median(0.05)
{
	_max_normal_angle_deviation = ml::math::degreesToRadians(45.);
	std::cout << "allowed angle " << _max_normal_angle_deviation << " allowed distance " << _max_allowed_distance << std::endl;
}