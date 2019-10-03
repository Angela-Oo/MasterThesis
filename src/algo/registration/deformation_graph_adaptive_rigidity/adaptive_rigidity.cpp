#include "adaptive_rigidity.h"
#include "algo/registration/deformation_graph_refinement/refinement_deformation_graph.h"
#include <memory>

namespace Registration {

size_t adaptRigidity(SurfaceMesh & deformation, double rigidity_smooth_cost_threshold, double minimal_rigidity)
{
	auto & edge_rigidity = deformation.property_map<edge_descriptor, double>("e:rigidity").first;
	auto edges = getEdgesToRefine(deformation, rigidity_smooth_cost_threshold, 0.1);
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

