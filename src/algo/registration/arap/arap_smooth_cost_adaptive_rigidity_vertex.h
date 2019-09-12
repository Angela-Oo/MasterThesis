#pragma once

#include "i_arap_smooth_cost.h"
#include "arap_deformation.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "util/ceres_include.h"

namespace Registration
{

class AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex : public IAsRigidAsPossibleSmoothCost
{
private:
	std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> _arap_residual_ids;
	std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> _rigidity_residual_ids;
	double _smooth_factor{ 1. };
	double _rigidity_factor{ 0.1 };
private:
	ceres::ResidualBlockId asRigidAsPossibleCostEdge(ceres::Problem &problem,
													 vertex_descriptor source,
													 vertex_descriptor target,
													 DeformationGraph<ARAPDeformation> & deformation_graph);
	ceres::ResidualBlockId adaptiveRigidityCostEdge(ceres::Problem &problem,													
													vertex_descriptor vertex,
													DeformationGraph<ARAPDeformation> & deformation_graph);
public:
	void evaluateResiduals(ceres::Problem &problem,
						   SurfaceMesh & mesh,
						   CeresIterationLoggerGuard & logger) override;
	EdgeResidualIds asRigidAsPossibleCost(ceres::Problem &problem,
										  double loss_weighting,
										  DeformationGraph<ARAPDeformation> & deformation_graph) override;
	AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex(double smooth_factor, double rigidity_factor);
};


}