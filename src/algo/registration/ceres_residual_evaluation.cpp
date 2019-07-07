#include "ceres_residual_evaluation.h"

std::vector<double> evaluateResiduals(ceres::Problem & problem,
									  std::vector<std::vector<ceres::ResidualBlockId>> & residual_block_ids)
{
	std::vector<ceres::ResidualBlockId> residual_ids;
	for (auto & r : residual_block_ids) {
		residual_ids.insert(residual_ids.end(), r.begin(), r.end());
	}

	ceres::Problem::EvaluateOptions evaluate_options;
	evaluate_options.residual_blocks = residual_ids;
	double total_cost = 0.0;
	std::vector<double> residuals;
	problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);

	std::vector<double> cost(residual_block_ids.size(), 0.);
	int residuals_per_edge = residuals.size() / residual_block_ids.size();
	for (int i = 0; i < residuals.size(); ++i) {
		size_t index = i / residuals_per_edge;
		cost[index] += abs(residuals[i]);
	}
	return cost;
}


std::vector<double> evaluateResiduals(ceres::Problem & problem,
									  std::map<vertex_descriptor, std::vector<ceres::ResidualBlockId>> & residual_block_ids)
{
	std::vector<std::vector<ceres::ResidualBlockId>> ids;
	for (auto & r : residual_block_ids) {
		ids.push_back(r.second);
	}
	return evaluateResiduals(problem, ids);
}

std::vector<double> evaluateResiduals(ceres::Problem & problem,
									  std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> & residual_block_ids)
{
	std::vector<std::vector<ceres::ResidualBlockId>> ids;
	for (auto & r : residual_block_ids) {
		ids.push_back(r.second);
	}
	return evaluateResiduals(problem, ids);
}
