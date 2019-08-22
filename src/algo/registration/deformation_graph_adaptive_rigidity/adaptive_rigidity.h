#pragma once

#include "algo/registration/deformation_graph_refinement/refinement_deformation_graph.h"
#include <memory>

namespace Registration {

template<typename Deformation>
size_t adaptRigidity(Deformation & deformation)
{
	auto & edge_rigidity = deformation._mesh.property_map<edge_descriptor, double>("e:rigidity").first;
	auto edges = getEdgesToRefine(deformation._mesh, 0.5, 0.001);
	int i = 0;
	for (auto & e : edges) {
		if (edge_rigidity[e] > 0.1) {
			edge_rigidity[e] = 0.25 * edge_rigidity[e];
			i++;
		}
	}
	return i;// edges.size();
}

}

