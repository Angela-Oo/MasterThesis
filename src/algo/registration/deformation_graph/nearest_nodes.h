#pragma once

#include "mLibInclude.h"
#include <vector>

namespace DG
{

class NearestNodes
{
public:
	Point point;
	std::vector<std::pair<vertex_descriptor, double>> node_weight_vector;
public:
	NearestNodes() {}
	NearestNodes(Point p, const std::vector<std::pair<vertex_descriptor, double>> & n_w_vector)
		: point(p)
		, node_weight_vector(n_w_vector)
	{}
	NearestNodes(const NearestNodes & other) 
		: point(other.point)
		, node_weight_vector(other.node_weight_vector)
	{
	}
	NearestNodes & operator=(const NearestNodes & other)
	{
		if (&other == this)
			return *this;
		point = other.point;
		node_weight_vector = other.node_weight_vector;
		return *this;
	}
};


}