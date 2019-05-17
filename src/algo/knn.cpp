#include "stdafx.h"

#include "knn.h"


KNNBruteForce::KNNBruteForce(const std::vector<ml::vec3f> & points)
	: _points(points)
{
	std::vector<const float*> nn_points;
	for (auto & p : points) {
		float * point = new float[3];
		point[0] = p[0];
		point[1] = p[1];
		point[2] = p[2];
		nn_points.push_back(point);
	}
	_neares_neighbor_search.init(nn_points, 3, 3);

	for (auto & p : nn_points)
	{
		delete[] p;
	}
	nn_points.clear();
}

ml::vec3f KNNBruteForce::nearest_f(const float * point)
{
	unsigned int pos = _neares_neighbor_search.nearest(point);
	return _points[pos];
}

ml::vec3f KNNBruteForce::nearest(const ml::vec3f & p)
{
	float * point = new float[3];
	point[0] = p[0];
	point[1] = p[1];
	point[2] = p[2];
	unsigned int pos = _neares_neighbor_search.nearest(point);
	return _points[pos];
}


KNN::KNN(const std::vector<ml::vec3f> & points, unsigned int max_k)
	: _points(points)
	, _neares_neighbor_search(50, 12)
{
	std::vector<const float*> nn_points;
	for (auto & p : points) {
		float * point = new float[3];
		point[0] = p[0];
		point[1] = p[1];
		point[2] = p[2];
		nn_points.push_back(point);
	}
	_neares_neighbor_search.init(nn_points, 3, max_k);

	for (auto & p : nn_points)
	{
		delete[] p;
	}
	nn_points.clear();

}

ml::vec3f KNN::nearest(const ml::vec3f & p)
{
	float * point = new float[3];
	point[0] = p[0];
	point[1] = p[1];
	point[2] = p[2];
	unsigned int pos = _neares_neighbor_search.nearest(point);
	return _points[pos];
}

unsigned int KNN::nearest_index(const ml::vec3f & p)
{
	float * point = new float[3];
	point[0] = p[0];
	point[1] = p[1];
	point[2] = p[2];
	return _neares_neighbor_search.nearest(point);
}

std::vector<unsigned int> KNN::k_nearest_indices(const ml::vec3f & point, unsigned int k)
{
	return _neares_neighbor_search.kNearest(point.array, k, 0.000001);
}

std::vector<ml::vec3f> KNN::k_nearest(const ml::vec3f & point, unsigned int k)
{
	std::vector<unsigned int> indices = k_nearest_indices(point, k);
	std::vector<ml::vec3f> points;
	for (auto i : indices) {
		points.push_back(_points[i]);
	}
	return points;
}

ml::vec3f KNN::nearest_f(const float * point)
{
	unsigned int pos = _neares_neighbor_search.nearest(point);
	return _points[pos];
}