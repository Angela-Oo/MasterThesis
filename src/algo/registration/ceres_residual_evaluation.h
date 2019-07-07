#pragma once

#include "algo/surface_mesh/mesh_definition.h"
#include <ceres/ceres.h>
#include <vector>

using ResidualIds = std::vector<ceres::ResidualBlockId>;

std::vector<double> evaluateResiduals(ceres::Problem & problem,
									  std::vector<std::vector<ceres::ResidualBlockId>> & residual_block_ids);


std::vector<double> evaluateResiduals(ceres::Problem & problem,
									  std::map<vertex_descriptor, std::vector<ceres::ResidualBlockId>> & residual_block_ids);

std::vector<double> evaluateResiduals(ceres::Problem & problem,
									  std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> & residual_block_ids);


template<typename Descriptor>
void evaluateResiduals(SurfaceMesh & mesh,
					   ceres::Problem & problem,
					   std::map<Descriptor, ResidualIds> & residual_block_ids,
					   SurfaceMesh::Property_map<Descriptor, double> cost,
					   double coefficient)
{
	auto residuals = evaluateResiduals(problem, residual_block_ids);
	assert(residuals.size() == residual_block_ids.size());
	int i = 0;
	for (auto & r : residual_block_ids) {
		cost[r.first] = residuals[i] * coefficient;
		++i;
	}
}
