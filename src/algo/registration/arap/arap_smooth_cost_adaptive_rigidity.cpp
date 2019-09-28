#pragma once

#include "arap_smooth_cost_adaptive_rigidity.h"
#include "arap_smooth_cost_functions.h"
#include "algo/registration/util/ceres_residual_evaluation.h"

namespace Registration
{


void AsRigidAsPossibleSmoothCostAdaptiveRigidity::evaluateResiduals(ceres::Problem &problem, SurfaceMesh & mesh, CeresIterationLoggerGuard & logger)
{
	auto smooth_cost = mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	if (!_arap_residual_ids.empty() && smooth_cost.second) {
		auto max_and_mean_cost = ::evaluateResiduals(mesh, problem, _arap_residual_ids, smooth_cost.first, _smooth_factor);
		logger.write(" max_smooth_cost: " + std::to_string(max_and_mean_cost.first) + " mean_smooth_cost: " + std::to_string(max_and_mean_cost.second), false);
	}

	if (!_rigidity_residual_ids.empty()) {
		auto rigidity_cost = mesh.add_property_map<edge_descriptor, double>("e:rigidity_cost", 0.);
		auto max_and_mean_cost = ::evaluateResiduals(mesh, problem, _rigidity_residual_ids, rigidity_cost.first, _rigidity_factor * _smooth_factor);
		logger.write(" max_rigidity_cost: " + std::to_string(max_and_mean_cost.first) + " mean_rigidity_cost: " + std::to_string(max_and_mean_cost.second), false);

	}
	if(mesh.property_map<edge_descriptor, double>("e:rigidity").second)
	{
		auto rigidity_value = mesh.property_map<edge_descriptor, double>("e:rigidity").first;
		double max_rigidity = *std::max_element(rigidity_value.begin(), rigidity_value.end());
		double min_rigidity = *std::min_element(rigidity_value.begin(), rigidity_value.end());
		double mean_rigidity = std::accumulate(rigidity_value.begin(), rigidity_value.end(), 0.) / mesh.number_of_edges();

		logger.write(" min_rigidity_value: " + std::to_string(min_rigidity) + " max_rigidity_value: " + std::to_string(max_rigidity) + " mean_rigidity_value: " + std::to_string(mean_rigidity), false);
	}
}

ceres::ResidualBlockId
AsRigidAsPossibleSmoothCostAdaptiveRigidity::asRigidAsPossibleCostEdge(ceres::Problem &problem,
																	   halfedge_descriptor he,
																	   DeformationGraph<ARAPDeformation> & deformation_graph)
{
	auto edge = deformation_graph._mesh.edge(he);
	auto source = deformation_graph._mesh.source(he);
	auto target = deformation_graph._mesh.target(he);

	auto edge_rigidity = deformation_graph._mesh.property_map<edge_descriptor, double>("e:rigidity").first;
	auto deformations = deformation_graph._mesh.property_map<vertex_descriptor, ARAPDeformation>("v:node_deformation").first;
	ceres::CostFunction* cost_function = AsRigidAsPossibleAdaptableRigidityCostFunction::Create(deformation_graph._mesh.point(source),
																								deformation_graph._mesh.point(target),
																								_minimal_rigidity_weight);
	auto loss_function = new ceres::ScaledLoss(NULL, _smooth_factor, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									deformations[source].d(),
									deformations[target].d(),
									&edge_rigidity[edge]);
}

ceres::ResidualBlockId
AsRigidAsPossibleSmoothCostAdaptiveRigidity::adaptiveRigidityCostEdge(ceres::Problem &problem,
																	  edge_descriptor edge,
																	  DeformationGraph<ARAPDeformation> & deformation_graph)
{
	auto edge_rigidity = deformation_graph._mesh.property_map<edge_descriptor, double>("e:rigidity").first;
	ceres::CostFunction* cost_function;
	if(_use_quadratic_rigid_weight)
		cost_function = AdaptableRigidityWeightCostFunction::Create(1.);
	else
		cost_function = RigidityWeightRegularizationCostFunction::Create(1.);
	auto loss_function = new ceres::ScaledLoss(NULL, _rigidity_factor * _smooth_factor, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, &edge_rigidity[edge]);
}





AsRigidAsPossibleSmoothCostAdaptiveRigidity::EdgeResidualIds
AsRigidAsPossibleSmoothCostAdaptiveRigidity::asRigidAsPossibleCost(ceres::Problem &problem,
																   double loss_weighting,
																   DeformationGraph<ARAPDeformation> & deformation_graph)
{
	if (_smooth_factor != loss_weighting) {
		_smooth_factor = loss_weighting;
		std::cout << " smooth factor " << _smooth_factor << ", rigidity factor " << _rigidity_factor * _smooth_factor << std::endl;
	}
	_arap_residual_ids.clear();
	_rigidity_residual_ids.clear();
	auto & mesh = deformation_graph._mesh;

	if (!mesh.property_map<edge_descriptor, double>("e:rigidity").second)
	{
		mesh.add_property_map<edge_descriptor, double>("e:rigidity", 1.);
	}
	for (auto he : mesh.halfedges())
	{
		auto edge = mesh.edge(he);
		_arap_residual_ids[edge].push_back(asRigidAsPossibleCostEdge(problem, he, deformation_graph));
		_rigidity_residual_ids[edge].push_back(adaptiveRigidityCostEdge(problem, edge, deformation_graph));
	}

	return _arap_residual_ids;
}

AsRigidAsPossibleSmoothCostAdaptiveRigidity::AsRigidAsPossibleSmoothCostAdaptiveRigidity(RegistrationOptions options)
	: _smooth_factor(options.smooth)
	, _rigidity_factor(options.adaptive_rigidity.rigidity_cost_coefficient)
	, _minimal_rigidity_weight(options.adaptive_rigidity.minimal_rigidity_weight)
	, _use_quadratic_rigid_weight(options.adaptive_rigidity.regular == AdaptiveRigidityRegularizer::SQUARED)
{
	std::cout << std::endl << " smooth factor " << _smooth_factor
		<< ", rigidity factor " << _rigidity_factor * _smooth_factor
		<< ", minimal rigidity weight" << _minimal_rigidity_weight << std::endl;
}

}