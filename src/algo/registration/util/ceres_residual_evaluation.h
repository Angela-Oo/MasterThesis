#pragma once

#include "mesh/mesh_definition.h"
#include <ceres/ceres.h>
#include <vector>

using ResidualIds = std::vector<ceres::ResidualBlockId>;

std::vector<double> evaluateResiduals(ceres::Problem & problem,
									  std::vector<std::vector<ceres::ResidualBlockId>> & residual_block_ids);


std::vector<double> evaluateResiduals(ceres::Problem & problem,
									  std::map<vertex_descriptor, std::vector<ceres::ResidualBlockId>> & residual_block_ids);

std::vector<double> evaluateResiduals(ceres::Problem & problem,
									  std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> & residual_block_ids);


// returns max and mean cost
template<typename Descriptor>
std::pair<double, double> evaluateResiduals(SurfaceMesh & mesh,
					                        ceres::Problem & problem,
					                        std::map<Descriptor, ResidualIds> & residual_block_ids,
					                        SurfaceMesh::Property_map<Descriptor, double> cost,
					                        double coefficient)
{
	auto residuals = evaluateResiduals(problem, residual_block_ids);
	assert(residuals.size() == residual_block_ids.size());
	int i = 0;
	double mean_cost = 0.;
	double max_cost = 0.0;
	for (auto & r : residual_block_ids) {
		cost[r.first] = residuals[i] * coefficient;
		if (cost[r.first] > max_cost)
			max_cost = cost[r.first];
		mean_cost += cost[r.first];
		++i;
	}
	mean_cost /= residual_block_ids.size();
	return std::make_pair(max_cost, mean_cost);
}
