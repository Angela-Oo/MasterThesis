#pragma once
#include <vector>
#include "mLibInclude.h"
#include "ext-flann/nearestNeighborSearchFLANN.h"

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
	KNN(const std::vector<ml::vec3f> & points);
	ml::vec3f nearest(const ml::vec3f & point);
	unsigned int nearest_index(const ml::vec3f & point);
	ml::vec3f nearest_f(const float * point);
};


