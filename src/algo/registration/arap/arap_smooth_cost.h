#pragma once

#include "i_arap_smooth_cost.h"
#include "algo/registration/util/ceres_iteration_logger.h"

namespace Registration
{

class AsRigidAsPossibleSmoothCost : public IAsRigidAsPossibleSmoothCost
{
private:
	std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> _arap_residual_ids;
	double _smooth_factor{ 1. };
public:
	void evaluateResiduals(ceres::Problem &problem,
						   SurfaceMesh & mesh,
						   CeresIterationLoggerGuard & logger) override;
	EdgeResidualIds asRigidAsPossibleCost(ceres::Problem &problem,
										  double loss_weighting,
										  DeformationGraph<ARAPDeformation> & deformation_graph) override;
public:
	AsRigidAsPossibleSmoothCost(double smooth_factor);
};



}