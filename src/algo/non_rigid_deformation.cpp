#include "non_rigid_deformation.h"
#include "as_rigid_as_possible_cost_function.h"
#include "se3.h"

AsRigidAsPossible::AsRigidAsPossible(const std::vector<ml::vec3f>& src,
									 const std::vector<ml::vec3f>& dst,
									 ceres::Solver::Options option)
	: _src(src)
	, _dst(dst)
	, _options(option)
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}


ml::mat4f AsRigidAsPossible::solve()
{
	ceres::Solver::Summary summary;

	ml::vec6d rotation_translation(0., 0., 0., 0., 0., 0.);
	ceres::Problem problem;
	for (int i = 0; i < _src.size() - 1; ++i) {
		ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(_dst[i], _dst[i+1], _src[i], _src[i+1]);
		problem.AddResidualBlock(cost_function, NULL, rotation_translation.array);		
	}
	ml::vec3d error(0., 0., 0.);
	for (int i = 0; i < _src.size(); ++i)
	{
		ceres::CostFunction* cost_function = FitCostFunction::Create(_dst[i], _src[i]);
		problem.AddResidualBlock(cost_function, NULL, error.array);
	}
	ceres::Solve(_options, &problem, &summary);
	return rigid_transformation_from_se3(rotation_translation);
}

