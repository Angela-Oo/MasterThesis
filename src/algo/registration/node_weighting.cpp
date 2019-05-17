#include "stdafx.h"
#include "node_weighting.h"

std::vector<double> nodeDistanceWeighting(const ml::vec3f & point, const std::vector<ml::vec3f>& node_positions)
{
	auto last_node = node_positions[node_positions.size() - 1];
	double d_max = ml::dist(point, last_node);

	std::vector<double> weights;
	for (size_t i = 0; i < node_positions.size() - 1; ++i)
	{
		double normed_distance = ml::dist(point, node_positions[i]);
		double weight = std::pow(1. - (normed_distance / d_max), 2);
		weights.push_back(weight);
	}

	double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
	std::for_each(weights.begin(), weights.end(), [sum](double & w) { w = w / sum; });
	return weights;
}
