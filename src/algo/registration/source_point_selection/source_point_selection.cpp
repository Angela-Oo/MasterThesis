#include "source_point_selection.h"
#include <random>

namespace Registration {
	
std::vector<vertex_descriptor> selectRandomSubset(const SurfaceMesh & mesh, double random_probability)
{
	std::knuth_b rand_engine;
	const auto randomBoolWithProb = [&rand_engine](double prob) {
		std::bernoulli_distribution d(prob);
		return d(rand_engine);
	};

	std::vector<vertex_descriptor> selected_vertices;
	
	// comment out for random at in each iteration step
	if (random_probability < 1.) {
		for (auto & v : mesh.vertices()) {
			const bool use_vertex = randomBoolWithProb(random_probability);
			if (use_vertex) {
				selected_vertices.push_back(v);
			}
		}
	}
	// select all points
	else
	{
		for (auto & v : mesh.vertices()) {
			selected_vertices.push_back(v);
		}
	}
	return selected_vertices;
}
	
}
