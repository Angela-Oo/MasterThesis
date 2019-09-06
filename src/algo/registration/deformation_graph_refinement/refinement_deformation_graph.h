#pragma once

#include "refinement_deformation.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "mesh/mesh_definition.h"


namespace Registration {


std::vector<edge_descriptor> getEdgesToRefine(SurfaceMesh & refined_mesh, double min_smooth_cost = 0.01, double percentage_max_smooth_cost = 0.9);

std::vector<vertex_descriptor> getVerticesToRefine(SurfaceMesh & refined_mesh, double min_smooth_cost, double percentage_max_smooth_cost);


template <typename PositionDeformation>
void interpolateNewNodeDeformations(RefineDeformationGraphDeformation<DeformationGraph<PositionDeformation>>& deformation, 
									const std::vector<vertex_descriptor> & new_vertices,
									SurfaceMesh & refined_mesh)
{
	auto & deformation_property_map = refined_mesh.property_map<vertex_descriptor, PositionDeformation>("v:node_deformation").first;
	for (auto v : new_vertices) {
		if (refined_mesh.is_valid(v)) {
			//NearestNodes kNN = createNearestNodes<DeformationGraph<PositionDeformation>>(deformation.non_rigid_deformation, refined_mesh.point(v));
			NearestNodes kNN = createNearestNodesRadiusOfInfluence<DeformationGraph<PositionDeformation>>(deformation.non_rigid_deformation,
			                                                                                              refined_mesh.point(v));

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
}

template <typename PositionDeformation>
size_t refineHierarchicalMeshAtEdges(RefineDeformationGraphDeformation<DeformationGraph<PositionDeformation>> & deformation)
{
	SurfaceMesh refined_mesh = deformation.non_rigid_deformation._mesh;
	HierarchicalMeshRefinement mesh_refiner(deformation.hierarchical_mesh);

	auto edges = getEdgesToRefine(refined_mesh, 0.1, 0.9);
	auto new_vertices = mesh_refiner.refine(edges, refined_mesh);

	interpolateNewNodeDeformations<PositionDeformation>(deformation, new_vertices, refined_mesh);
	deformation.non_rigid_deformation = DeformationGraph<PositionDeformation>(refined_mesh,
																			  deformation.non_rigid_deformation._global, 
																			  deformation.non_rigid_deformation.getNumberOfInterpolationNeighbors());

	return edges.size();
}

template <typename PositionDeformation>
size_t refineHierarchicalMeshAtVertices(RefineDeformationGraphDeformation<DeformationGraph<PositionDeformation>> & deformation)
{
	SurfaceMesh refined_mesh = deformation.non_rigid_deformation._mesh;
	HierarchicalMeshRefinement mesh_refiner(deformation.hierarchical_mesh);

	auto vertices = getVerticesToRefine(refined_mesh, 0.01, 0.8);
	auto new_vertices = mesh_refiner.refine(vertices, refined_mesh);

	interpolateNewNodeDeformations<PositionDeformation>(deformation, new_vertices, refined_mesh);

	// test by resetting deformation graph
	//for (auto v : refined_mesh.vertices()) {
	//	deformation_property_map[v] = PositionDeformation(refined_mesh.point(v));
	//}
	deformation.non_rigid_deformation = DeformationGraph<PositionDeformation>(refined_mesh,
																			  deformation.non_rigid_deformation._global,
																			  deformation.non_rigid_deformation.getNumberOfInterpolationNeighbors());
	return vertices.size();
}
	
}