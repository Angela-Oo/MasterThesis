#pragma once
#include <vector>
#include "mLibInclude.h"
#include "mLibFLANN.h"

class KNNBruteForce {
	const std::vector<ml::vec3f> & _points;
	ml::NearestNeighborSearchBruteForce<float> _neares_neighbor_search;
public:
	KNNBruteForce(const std::vector<ml::vec3f> & points);
	ml::vec3f nearest(const ml::vec3f & point);
	ml::vec3f nearest_f(const float * point);
};



class KNN {
	const std::vector<ml::vec3f> & _points;
	ml::NearestNeighborSearchFLANN<float> _neares_neighbor_search;
public:
	KNN(const std::vector<ml::vec3f> & points, unsigned int max_k = 1);
	ml::vec3f nearest(const ml::vec3f & point);
	unsigned int nearest_index(const ml::vec3f & point);
	std::vector<unsigned int> k_nearest_indices(const ml::vec3f & point, unsigned int k);
	std::vector<ml::vec3f> k_nearest(const ml::vec3f & point, unsigned int k);
	ml::vec3f nearest_f(const float * point);
};


