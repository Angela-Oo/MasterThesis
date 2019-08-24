#pragma once

#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif // !GLOG_NO_ABBREVIATED_SEVERITIES

#include "glog/logging.h"

#include "arap_deformation.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include <ceres/problem.h>

namespace Registration
{

class IAsRigidAsPossibleSmoothCost
{
protected:
	using ResidualIds = std::vector<ceres::ResidualBlockId>;
	using EdgeResidualIds = std::map<edge_descriptor, ResidualIds>;
public:
	virtual void evaluateResiduals(ceres::Problem &problem,
								   SurfaceMesh & mesh,
								   CeresIterationLoggerGuard & logger) = 0;
	virtual EdgeResidualIds asRigidAsPossibleCost(ceres::Problem &problem,
											      double loss_weighting,
												  DeformationGraph<ARAPDeformation> & deformation_graph) = 0;
	virtual ~IAsRigidAsPossibleSmoothCost() {};
};


}