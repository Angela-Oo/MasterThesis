#pragma once

#include "i_arap_smooth_cost.h"
#include "arap_deformation.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include <ceres/problem.h>

namespace Registration
{

class AsRigidAsPossibleSmoothCostAdaptiveRigidity : public IAsRigidAsPossibleSmoothCost
{
public:
	EdgeResidualIds asRigidAsPossibleCost(ceres::Problem &problem,
										  double loss_weighting,
										  DeformationGraph<ARAPDeformation> & deformation_graph) override;
};



}