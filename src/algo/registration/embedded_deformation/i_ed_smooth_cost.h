#pragma once

#include "ed_deformation.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include "util/ceres_include.h"

namespace Registration
{

class IEmbeddedDeformationSmoothCost
{
protected:
	using ResidualIds = std::vector<ceres::ResidualBlockId>;
	using EdgeResidualIds = std::map<edge_descriptor, ResidualIds>;
	using VertexResidualIds = std::map<vertex_descriptor, ResidualIds>;
public:
	virtual void evaluateResiduals(ceres::Problem &problem,
								   SurfaceMesh & mesh,
								   CeresIterationLoggerGuard & logger) = 0;
	virtual EdgeResidualIds smoothCost(ceres::Problem &problem,
									   double loss_weighting,
									   DeformationGraph<EDDeformation> & deformation_graph) = 0;
	virtual VertexResidualIds rotationCost(ceres::Problem &problem,
										 double loss_weighting,
										 DeformationGraph<EDDeformation> & deformation_graph) = 0;
	virtual ~IEmbeddedDeformationSmoothCost() {};
};


}