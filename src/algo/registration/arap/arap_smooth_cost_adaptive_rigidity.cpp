#pragma once

#include "arap_smooth_cost_adaptive_rigidity.h"
#include "arap_cost.h"

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

		auto residual_id = Registration::asRigidAsPossibleCost(problem, smooth, deformation_graph, mesh.source(e), mesh.target(e));

		auto edge = deformation_graph._mesh.edge(e);
		residual_ids[edge].push_back(residual_id);
	}

	return residual_ids;
}



}