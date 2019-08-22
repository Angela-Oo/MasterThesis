#pragma once

#include "i_arap_smooth_cost.h"

namespace Registration
{

class AsRigidAsPossibleSmoothCost : public IAsRigidAsPossibleSmoothCost
{
public:
	EdgeResidualIds asRigidAsPossibleCost(ceres::Problem &problem,
										  double loss_weighting,
										  DeformationGraph<ARAPDeformation> & deformation_graph) override;
};



}