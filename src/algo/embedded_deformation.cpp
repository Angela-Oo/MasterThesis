#include "embedded_deformation.h"
#include "embedded_deformation_cost_function.h"
#include "se3.h"

std::vector<size_t> EmbeddedDeformation::getNeighborIndices(size_t i, size_t size)
{
	std::vector<size_t> indices;
	int j = static_cast<int>(i) - 1;
	if (j >= 0)
		indices.push_back(static_cast<size_t>(j));
	j = static_cast<int>(i) + 1;
	if (j < size)
		indices.push_back(static_cast<size_t>(j));
	return indices;
}

std::vector<ml::vec3f> EmbeddedDeformation::solve()
{
	ceres::Solver::Summary summary;
	ceres::Problem problem;
	for (int i = 0; i < _src.size(); ++i) {
		auto neighbors = getNeighborIndices(i, _src.size());
		for (auto & j : neighbors) {
			ceres::CostFunction* cost_function = EmbeddedDeformationCostFunction::Create(_solved_points[i], _solved_points[j], _src[i], _src[j]);
			problem.AddResidualBlock(cost_function, NULL, (&_matrix[i])->getData());
		}
	}
	for (int i = 0; i < _src.size() - 1; ++i)
	{
		ceres::CostFunction* cost_function = RotationCostFunction::Create();
		problem.AddResidualBlock(cost_function, NULL, (&_matrix[i])->getData());
	}
	for (int i = 0; i < _src.size() - 1; ++i) {
		ml::vec3d * dst_i = &_solved_points[i];
		auto neighbors = getNeighborIndices(i, _src.size());
		for (auto & j : neighbors) {
			ceres::CostFunction* cost_function = EmbeddedDeformationPointsCostFunction::Create(_src[i], _src[j], _matrix[i], _solved_points[j]);
			problem.AddResidualBlock(cost_function, NULL, dst_i->array);
		}
	}
	ceres::Solve(_options, &problem, &summary);

	std::vector<ml::vec3f> solved_points;
	for (int i = 0; i < _solved_points.size(); ++i) {
		solved_points.push_back(ml::vec3d(_solved_points[i].x, _solved_points[i].y, _solved_points[i].z));
	}
	return solved_points;
}

EmbeddedDeformation::EmbeddedDeformation(const std::vector<ml::vec3f>& src,
										 const std::vector<ml::vec3f>& dst,
										 ceres::Solver::Options option)
	: _src(src)
	, _dst(dst)
	, _options(option)
{
	for (int i = 0; i < _src.size(); ++i) {
		_matrix.push_back(ml::mat3d::identity());
	}
	for (int i = 0; i < _dst.size(); ++i) {
		_solved_points.push_back(ml::vec3d(_dst[i].x, _dst[i].y, _dst[i].z));
	}

	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}