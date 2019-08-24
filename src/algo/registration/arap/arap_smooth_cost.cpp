#pragma once

#include "arap_smooth_cost.h"
#include "arap_cost.h"
#include "algo/registration/util/ceres_residual_evaluation.h"

namespace Registration
{

void AsRigidAsPossibleSmoothCost::evaluateResiduals(ceres::Problem &problem, SurfaceMesh & mesh, CeresIterationLoggerGuard & logger)
{
	auto smooth_cost = mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	if (smooth_cost.second) {
		auto max_and_mean_cost = ::evaluateResiduals(mesh, problem, _arap_residual_ids, smooth_cost.first, _smooth_factor);
		logger.write(" max smooth cost: " + std::to_string(max_and_mean_cost.first) + " reference smooth " + std::to_string(max_and_mean_cost.second * 10.), false);
	}
}

AsRigidAsPossibleSmoothCost::EdgeResidualIds
AsRigidAsPossibleSmoothCost::asRigidAsPossibleCost(ceres::Problem &problem,
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

AsRigidAsPossibleSmoothCost::AsRigidAsPossibleSmoothCost(double smooth_factor)
	: _smooth_factor(smooth_factor)
{ }


}