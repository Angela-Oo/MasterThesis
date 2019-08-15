#pragma once

#include "mesh/mesh_definition.h"
#include "refine_deformation_graph_deformation.h"
#include "algo/registration/deformation_graph/refine_deformation_graph.h"

namespace Registration {

template <typename PositionDeformation>
DeformationGraph<PositionDeformation> refineDeformationGraph(const RefineDeformationGraphDeformation<DeformationGraph<PositionDeformation>> & deformation)
{
	auto h_mesh = deformation.hierarchical_mesh;
	h_mesh._mesh = deformation.non_rigid_deformation._mesh;
	auto edges = getEdgesToRefine(h_mesh._mesh);

	std::vector<vertex_descriptor> new_vertices;
	for (auto & e : edges) {
		auto vs = h_mesh.refineEdge(e);
		new_vertices.insert(new_vertices.end(), vs.begin(), vs.end());
	}
	h_mesh.triangulate();

	auto & deformation_property_map = h_mesh._mesh.property_map<vertex_descriptor, PositionDeformation>("v:node_deformation").first; // TODO
	for (auto v : new_vertices) {
		int k = 4; // todo
		NearestNodes kNN = createNearestNodes<DeformationGraph<PositionDeformation>>(deformation.non_rigid_deformation, h_mesh._mesh.point(v), k);

		std::vector<std::pair<PositionDeformation, double>> deformation_weights_vector;
		for (auto n_w : kNN.node_weight_vector)
		{
			auto node = deformation.non_rigid_deformation.getDeformation(n_w.first);
			deformation_weights_vector.push_back(std::make_pair(node, n_w.second));
		}
		PositionDeformation deformed_position = interpolateDeformations(h_mesh._mesh.point(v), deformation_weights_vector);
		deformation_property_map[v] = deformed_position;
	}

	return DeformationGraph<PositionDeformation>(h_mesh._mesh, deformation.non_rigid_deformation._global);
}

}