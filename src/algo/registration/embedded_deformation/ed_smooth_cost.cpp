#pragma once

#include "ed_smooth_cost.h"
#include "algo/registration/util/ceres_residual_evaluation.h"
#include "ed_cost_functions.h"

namespace Registration
{

void EmbeddedDeformationSmoothCost::evaluateResiduals(ceres::Problem &problem, SurfaceMesh & mesh, CeresIterationLoggerGuard & logger)
{
	auto smooth_cost = mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	if (smooth_cost.second) {
		auto max_and_mean_cost = ::evaluateResiduals(mesh, problem, _smooth_residual_ids, smooth_cost.first, _smooth_factor);
		logger.write(" max_smooth_cost: " + std::to_string(max_and_mean_cost.first) + " mean_smooth_cost " + std::to_string(max_and_mean_cost.second), false);
	}
}

EmbeddedDeformationSmoothCost::EdgeResidualIds
EmbeddedDeformationSmoothCost::smoothCost(ceres::Problem &problem,
										  double loss_weighting,
										  DeformationGraph<EDDeformation> & deformation_graph)
{
	_smooth_residual_ids.clear();
	auto & mesh = deformation_graph._mesh;

	auto deformations = mesh.property_map<vertex_descriptor, EDDeformation>("v:node_deformation").first;
	for (auto e : mesh.halfedges()) {
		auto target = mesh.target(e);
		auto source = mesh.source(e);

		ceres::CostFunction* cost_function = ED::SmoothCostFunction::Create(mesh.point(source), mesh.point(target));
		auto loss_function = new ceres::ScaledLoss(NULL, loss_weighting, ceres::TAKE_OWNERSHIP);
		auto residual_id = problem.AddResidualBlock(cost_function, loss_function,
													deformations[source].r(), deformations[source].t(), deformations[target].t());

		auto edge = deformation_graph._mesh.edge(e);
		_smooth_residual_ids[edge].push_back(residual_id);
	}

	return _smooth_residual_ids;
}

EmbeddedDeformationSmoothCost::VertexResidualIds
EmbeddedDeformationSmoothCost::rotationCost(ceres::Problem &problem,
											double loss_weighting,
											DeformationGraph<EDDeformation> & deformation_graph)
{
	VertexResidualIds residual_ids;

	auto deformations = deformation_graph._mesh.property_map<vertex_descriptor, EDDeformation>("v:node_deformation").first;
	for (auto & v : deformation_graph._mesh.vertices()) {
		ceres::CostFunction* cost_function = ED::RotationCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(new ceres::SoftLOneLoss(0.001), loss_weighting, ceres::TAKE_OWNERSHIP);
		residual_ids[v].push_back(problem.AddResidualBlock(cost_function, loss_function, deformations[v].r()));
	}

	// add global rotation cost
	ceres::CostFunction* cost_function = ED::RotationCostFunction::Create();
	auto loss_function = new ceres::ScaledLoss(new ceres::SoftLOneLoss(0.001), loss_weighting, ceres::TAKE_OWNERSHIP);
	problem.AddResidualBlock(cost_function, loss_function, deformation_graph._global.r());
	return residual_ids;	
}

EmbeddedDeformationSmoothCost::EmbeddedDeformationSmoothCost(double smooth_factor)
	: _smooth_factor(smooth_factor)
{ }


}
