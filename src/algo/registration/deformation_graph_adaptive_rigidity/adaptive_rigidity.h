#pragma once

#include "algo/registration/deformation_graph_refinement/refinement_deformation_graph.h"
#include <memory>

namespace Registration {

template<typename Deformation>
size_t adaptRigidity(Deformation & deformation, double rigidity_smooth_cost_threshold, double minimal_rigidity)
{
	auto & edge_rigidity = deformation._mesh.property_map<edge_descriptor, double>("e:rigidity").first;
	auto edges = getEdgesToRefine(deformation._mesh, rigidity_smooth_cost_threshold, 0.001);
	int i = 0;
	for (auto & e : edges) {
		if (edge_rigidity[e] > minimal_rigidity) {
			edge_rigidity[e] = 0.5 * edge_rigidity[e];
			i++;
		}
	}
	return i;// edges.size();
}

}

