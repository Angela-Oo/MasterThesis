#pragma once

#include "i_ed_smooth_cost.h"
#include "algo/registration/util/ceres_iteration_logger.h"

namespace Registration
{

class EmbeddedDeformationSmoothCost : public IEmbeddedDeformationSmoothCost
{
private:
	std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> _smooth_residual_ids;
	double _smooth_factor{ 1. };
public:
	void evaluateResiduals(ceres::Problem &problem,
						   SurfaceMesh & mesh,
						   CeresIterationLoggerGuard & logger) override;
	EdgeResidualIds smoothCost(ceres::Problem &problem,
							   double loss_weighting,
							   DeformationGraph<EDDeformation> & deformation_graph) override;
	VertexResidualIds rotationCost(ceres::Problem &problem,
								 double loss_weighting,
								 DeformationGraph<EDDeformation> & deformation_graph) override;
public:
	EmbeddedDeformationSmoothCost(double smooth_factor);
};



}