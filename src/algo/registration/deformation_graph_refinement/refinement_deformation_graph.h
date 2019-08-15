#pragma once

#include "mesh/mesh_definition.h"
#include "refinement_deformation.h"

namespace Registration {


std::vector<edge_descriptor> getEdgesToRefine(SurfaceMesh & refined_mesh);


template <typename PositionDeformation>
DeformationGraph<PositionDeformation> refineHierarchicalMesh(const RefineDeformationGraphDeformation<DeformationGraph<PositionDeformation>> & deformation)
{
	auto refined_mesh = deformation.non_rigid_deformation._mesh;
	auto edges = getEdgesToRefine(refined_mesh);

	HierarchicalMeshRefinement mesh_refiner(deformation.hierarchical_mesh);
	auto new_vertices = mesh_refiner.refine(edges, refined_mesh);


	auto & deformation_property_map = refined_mesh.property_map<vertex_descriptor, PositionDeformation>("v:node_deformation").first; // TODO
	for (auto v : new_vertices) {
		if (refined_mesh.is_valid(v)) {
			int k = 4; // todo
			NearestNodes kNN = createNearestNodes<DeformationGraph<PositionDeformation>>(deformation.non_rigid_deformation, refined_mesh.point(v), k);

			std::vector<std::pair<PositionDeformation, double>> deformation_weights_vector;
			for (auto n_w : kNN.node_weight_vector)
			{
				auto node = deformation.non_rigid_deformation.getDeformation(n_w.first);
					deformation_weights_vector.push_back(std::make_pair(node, n_w.second));
			}
			PositionDeformation deformed_position = interpolateDeformations(refined_mesh.point(v), deformation_weights_vector);
			deformation_property_map[v] = deformed_position;
		}
	}

	return DeformationGraph<PositionDeformation>(refined_mesh, deformation.non_rigid_deformation._global);
}

}