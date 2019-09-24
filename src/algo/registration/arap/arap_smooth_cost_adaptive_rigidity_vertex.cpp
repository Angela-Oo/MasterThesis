#pragma once

#include "arap_smooth_cost_adaptive_rigidity_vertex.h"
#include "arap_cost_functions.h"
#include "algo/registration/util/ceres_residual_evaluation.h"

namespace Registration {

void AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex::evaluateResiduals(ceres::Problem &problem, SurfaceMesh & mesh, CeresIterationLoggerGuard & logger)
{
	auto smooth_cost = mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	if (!_arap_residual_ids.empty() && smooth_cost.second) {
		auto max_and_mean_cost = ::evaluateResiduals(mesh, problem, _arap_residual_ids, smooth_cost.first, _smooth_factor);
		logger.write(" max_smooth_cost: " + std::to_string(max_and_mean_cost.first) + " mean_smooth_cost: " + std::to_string(max_and_mean_cost.second), false);
	}

	if (!_rigidity_residual_ids.empty()) {
		auto rigidity_cost = mesh.add_property_map<edge_descriptor, double>("e:rigidity_cost", 0.);
		auto max_and_mean_cost = ::evaluateResiduals(mesh, problem, _rigidity_residual_ids, rigidity_cost.first, _rigidity_factor);
		logger.write(" max_rigidity_cost: " + std::to_string(max_and_mean_cost.first) + " mean_rigidity_cost: " + std::to_string(max_and_mean_cost.second), false);

		auto rigidity_value = mesh.add_property_map<vertex_descriptor, double>("v:rigidity", 0.).first;
		double max_rigidity = *std::max_element(rigidity_value.begin(), rigidity_value.end());
		double min_rigidity = *std::min_element(rigidity_value.begin(), rigidity_value.end());
		double mean_rigidity = std::accumulate(rigidity_value.begin(), rigidity_value.end(), 0.) / mesh.number_of_vertices();

		logger.write(" min_rigidity_value: " + std::to_string(min_rigidity) + " max_rigidity_value: " + std::to_string(max_rigidity) + " mean_rigidity_value: " + std::to_string(mean_rigidity), false);
	}
}

ceres::ResidualBlockId
AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex::asRigidAsPossibleCostEdge(ceres::Problem &problem,
																			 vertex_descriptor source,
																			 vertex_descriptor target,
																			 DeformationGraph<ARAPDeformation> & deformation_graph)
{
	auto vertex_rigidity = deformation_graph._mesh.property_map<vertex_descriptor, double>("v:rigidity").first;
	auto deformations = deformation_graph._mesh.property_map<vertex_descriptor, ARAPDeformation>("v:node_deformation").first;
	ceres::CostFunction* cost_function = ARAPAdaptiveRigidityVertexCostFunction::Create(deformation_graph._mesh.point(source),
																						deformation_graph._mesh.point(target));
	auto loss_function = new ceres::ScaledLoss(NULL, _smooth_factor, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									deformations[source].d(),
									deformations[target].d(),
									&vertex_rigidity[source],
									&vertex_rigidity[target]);	
}

ceres::ResidualBlockId
AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex::adaptiveRigidityCostEdge(ceres::Problem &problem,
																			vertex_descriptor vertex,
																			DeformationGraph<ARAPDeformation> & deformation_graph)
{
	auto vertex_rigidity = deformation_graph._mesh.property_map<vertex_descriptor, double>("v:rigidity").first;
	ceres::CostFunction* cost_function;
	if(_use_quadratic_rigid_weight)
		cost_function = AdaptableRigidityWeightCostFunction::Create(1.);
	else
		cost_function = RigidityWeightRegularizationCostFunction::Create(1.);
	auto loss_function = new ceres::ScaledLoss(NULL, _rigidity_factor, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, &vertex_rigidity[vertex]);
}



AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex::EdgeResidualIds
AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex::asRigidAsPossibleCost(ceres::Problem &problem,
																		 double loss_weighting,
																		 DeformationGraph<ARAPDeformation> & deformation_graph)
{
	if (_smooth_factor != loss_weighting) {
		_smooth_factor = loss_weighting;
		std::cout << " smooth factor " << _smooth_factor << std::endl;
	}
	_arap_residual_ids.clear();
	_rigidity_residual_ids.clear();
	auto & mesh = deformation_graph._mesh;

	if (!mesh.property_map<vertex_descriptor, double>("v:rigidity").second) {
		mesh.add_property_map<vertex_descriptor, double>("v:rigidity", 1.);
	}

	for (auto he : mesh.halfedges()) {
		auto edge = mesh.edge(he);

		auto source = deformation_graph._mesh.source(he);
		auto target = deformation_graph._mesh.target(he);

		_arap_residual_ids[edge].push_back(asRigidAsPossibleCostEdge(problem, source, target, deformation_graph));

		_rigidity_residual_ids[edge].push_back(adaptiveRigidityCostEdge(problem, source, deformation_graph));
		_rigidity_residual_ids[edge].push_back(adaptiveRigidityCostEdge(problem, target, deformation_graph));
	}	
	return _arap_residual_ids;
}

AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex::AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex(double smooth_factor, double rigidity_factor, bool use_quadratic_rigid_weight)
	: _smooth_factor(smooth_factor)
	, _rigidity_factor(rigidity_factor)
	, _use_quadratic_rigid_weight(use_quadratic_rigid_weight)
{
	std::cout << std::endl << " smooth factor " << _smooth_factor
		<< ", rigidity factor " << _rigidity_factor << std::endl;
}

}