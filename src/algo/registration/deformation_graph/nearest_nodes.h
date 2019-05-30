#pragma once

#include "mLibInclude.h"
#include <vector>

namespace DG
{

class NearestNodes
{
public:
	Point point;
	std::vector<vertex_descriptor> nodes;
	std::vector<double> weights;
public:
	NearestNodes() {}
	NearestNodes(Point p, const std::vector<vertex_descriptor> & n, const std::vector<double> & w)
		: point(p)
		, nodes(n)
		, weights(w)
	{}
};


}