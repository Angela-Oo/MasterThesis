#include "non_rigid_deformation.h"
#include "as_rigid_as_possible_cost_function.h"
#include "embedded_deformation_cost_function.h"
#include "se3.h"



std::vector<size_t> AsRigidAsPossible::getNeighborIndices(size_t i, size_t size)
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

void AsRigidAsPossible::solveRotation()
{
	ceres::Solver::Summary summary;
	ceres::Problem problem;
	for (int i = 0; i < _src.size(); ++i) {
		auto neighbors = getNeighborIndices(i, _src.size());
		ml::vec3d * r_i = &_rotations[i];
		for (auto & j : neighbors) {
			ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(_solved_points[i], _solved_points[j], _src[i], _src[j]);
			problem.AddResidualBlock(cost_function, NULL, r_i->array);
		}
	}
	ceres::Solve(_options, &problem, &summary);
}

void AsRigidAsPossible::solvePositions()
{
	ceres::Solver::Summary summary;
	ceres::Problem problem;
	for (int i = 0; i < _src.size() - 1; ++i) {
		ml::vec3d * dst_i = &_solved_points[i];
		auto neighbors = getNeighborIndices(i, _src.size());
		for (auto & j : neighbors) {
			ceres::CostFunction* cost_function = AsRigidAsPossiblePointCostFunction::Create(_src[i], _src[j], _rotations[i], _solved_points[j]);
			problem.AddResidualBlock(cost_function, NULL, dst_i->array);
		}
	}
	for (int i = 0; i < _src.size() - 1; ++i)
	{
		ceres::CostFunction* cost_function = FitCostFunction::Create(_src[i]);
		problem.AddResidualBlock(cost_function, NULL, (&_solved_points[i])->array);
	}
	ceres::Solve(_options, &problem, &summary);
}


std::vector<ml::vec3f> AsRigidAsPossible::solve()
{
	for (int i = 0; i < _rotations.size(); ++i) {
		_rotations[i] = ml::vec3d(0., 0., 0.);
	}

	solveRotation();
	solvePositions();

	std::vector<ml::vec3f> solved_points;
	for (int i = 0; i < _solved_points.size(); ++i) {
		solved_points.push_back(ml::vec3d(_solved_points[i].x, _solved_points[i].y, _solved_points[i].z));
	}
	return solved_points;
}

AsRigidAsPossible::AsRigidAsPossible(const std::vector<ml::vec3f>& src,
									 const std::vector<ml::vec3f>& dst,
									 ceres::Solver::Options option)
	: _src(src)
	, _dst(dst)
	, _options(option)
{
	for (int i = 0; i < _src.size(); ++i) {
		_rotations.push_back(ml::vec3d(0., 0., 0.));
	}
	for (int i = 0; i < _dst.size(); ++i) {
		_solved_points.push_back(ml::vec3d(_dst[i].x, _dst[i].y, _dst[i].z));
	}

	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}
