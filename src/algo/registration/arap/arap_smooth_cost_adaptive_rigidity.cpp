#pragma once

#include "arap_smooth_cost_adaptive_rigidity.h"
#include "arap_cost_functions.h"

namespace Registration
{

AsRigidAsPossibleSmoothCostAdaptiveRigidity::EdgeResidualIds
AsRigidAsPossibleSmoothCostAdaptiveRigidity::asRigidAsPossibleCost(ceres::Problem &problem,
																   double loss_weighting,
																   DeformationGraph<ARAPDeformation> & deformation_graph)
{
	std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> residual_ids;
	auto & mesh = deformation_graph._mesh;

	auto & edge_rigidity = mesh.property_map<edge_descriptor, double>("e:rigidity");
	for (auto e : mesh.halfedges())
	{
		double smooth = loss_weighting;
		if (edge_rigidity.second) {
			smooth = edge_rigidity.first[mesh.edge(e)];
		}

		auto source = mesh.source(e);
		auto target = mesh.target(e);

		auto deformations = deformation_graph._mesh.property_map<vertex_descriptor, ARAPDeformation>("v:node_deformation").first;

		ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(deformation_graph._mesh.point(mesh.source(e)),
																				   deformation_graph._mesh.point(mesh.target(e)));
		auto loss_function = new ceres::ScaledLoss(NULL, loss_weighting, ceres::TAKE_OWNERSHIP);
		auto residual_id = problem.AddResidualBlock(cost_function, loss_function,
										deformations[source].d(), deformations[target].d());

		auto edge = deformation_graph._mesh.edge(e);
		residual_ids[edge].push_back(residual_id);
	}

	return residual_ids;
}



}