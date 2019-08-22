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

class IAsRigidAsPossibleFitCost
{
protected:
	using ResidualIds = std::vector<ceres::ResidualBlockId>;
	using VertexResidualIds = std::map<vertex_descriptor, ResidualIds>;
public:
	virtual std::vector<Point> getFixedPostions() {	return std::vector<Point>(); }

	virtual VertexResidualIds addFitCost(ceres::Problem &problem,
										 DeformationGraph<ARAPDeformation> & deformation_graph,
										 DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
										 std::unique_ptr<CeresIterationLoggerGuard>& logger) = 0;
	virtual ~IAsRigidAsPossibleFitCost() {};
};


}