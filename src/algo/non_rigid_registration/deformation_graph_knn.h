#pragma once
#include <vector>
#include "mLibInclude.h"
#include "ext-flann/nearestNeighborSearchFLANN.h"
#include "node.h"


class DeformationGraphKNN {
	const Graph & _graph;
	std::vector<vertex_index> _node_graph_indices;
	ml::NearestNeighborSearchFLANN<float> _neares_neighbor_search;
public:
	DeformationGraphKNN(const Graph & graph, unsigned int max_k = 1);
	vertex_index nearest_index(const ml::vec3f & point);
	Node nearest(const ml::vec3f & point);	
	std::vector<vertex_index> k_nearest_indices(const ml::vec3f & point, unsigned int k);
	std::vector<Node> k_nearest(const ml::vec3f & point, unsigned int k);
};


