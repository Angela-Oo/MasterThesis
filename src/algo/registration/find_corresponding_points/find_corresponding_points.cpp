#include "find_corresponding_points.h"
#include "mLibCore.h"


double angle_between_to_vectors_in_rad(Vector vector_a, Vector vector_b)
{
	Vector normalized_vector_a = vector_a / std::sqrt(vector_a.squared_length());
	Vector normalized_vector_b = vector_b / std::sqrt(vector_b.squared_length());
	auto dot_product = CGAL::scalar_product(normalized_vector_a, normalized_vector_b);
	auto angle = acos(dot_product);
	return angle;
}

double FindCorrespondingPoints::median()
{
	return _median_distance;
}


Point FindCorrespondingPoints::getPoint(vertex_descriptor v)
{
	return _mesh.point(v);
}

Vector FindCorrespondingPoints::getNormal(vertex_descriptor v)
{
	auto vertex_normals = _mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	return vertex_normals[v];
}

std::pair<bool, vertex_descriptor> FindCorrespondingPoints::correspondingPoint(Point point, Vector normal)
{
	auto s = _nn_search->search(point, _k);

	std::vector<std::pair<vertex_descriptor, std::pair<double, double>>> valid_point_with_angle_and_distance;
	auto vertex_normals = _mesh.property_map<vertex_descriptor, Vector>("v:normal").first;

	for (Neighbor_search::iterator it = s.begin(); it != s.end(); ++it) {
		auto distance = std::sqrt(it->second);
		auto v = it->first;

		auto angle = angle_between_to_vectors_in_rad(normal, vertex_normals[v]);

		//if (distance < 100. * median()) {
		_median_distance = (_median_distance * 10000. + distance) / 10001.;
		//}
		double k_median = _allowed_multiple_of_median_distance * median();
		k_median = std::max(_min_allowed_distance, k_median);
		bool valid_angle = angle < _max_normal_angle_deviation;
		bool valid_distance = distance < k_median;

		if (valid_angle && valid_distance) {
			valid_point_with_angle_and_distance.push_back(std::make_pair(v, std::make_pair(angle, distance)));
		}
		//else if(!valid_angle){
		//	std::cout << normal.x() << " " << normal.y() << " " << normal.z() << std::endl;
		//	std::cout << vertex_normals[v].x() << " " << vertex_normals[v].y() << " " << vertex_normals[v].z() << std::endl << std::endl;
		//}
	}

	if (valid_point_with_angle_and_distance.empty()) {
		return std::make_pair(false, vertex_descriptor(0));
	}
	else {
		auto sort_by_angle = [](const std::pair<vertex_descriptor, std::pair<double, double>> & rhs, const std::pair<vertex_descriptor, std::pair<double, double>> & lhs) { return rhs.second.second < lhs.second.second; };
		std::sort(valid_point_with_angle_and_distance.begin(), valid_point_with_angle_and_distance.end(), sort_by_angle);
		return std::make_pair(true, valid_point_with_angle_and_distance[0].first);
	}
}

FindCorrespondingPoints::FindCorrespondingPoints(const SurfaceMesh & mesh,
												 double initial_max_allowed_distance,												 
												 double max_normal_angle_deviation,
												 double allowed_multiple_of_median_distance,
												 double min_allowed_distance)
	: _mesh(mesh)
	, _k(10)	
	, _max_normal_angle_deviation(max_normal_angle_deviation)
	, _median_distance(initial_max_allowed_distance)
	, _allowed_multiple_of_median_distance(allowed_multiple_of_median_distance)
	, _min_allowed_distance(min_allowed_distance)
{
	_nn_search = std::make_unique<NearestNeighborSearch>(_mesh);
	std::cout << "allowed angle " << _max_normal_angle_deviation << " allowed distance " << _median_distance << std::endl;
	_max_normal_angle_deviation = ml::math::degreesToRadians(max_normal_angle_deviation);	
}