#include "non_rigid_deformation.h"
#include "ceres_iteration_logger.h"
#include "se3.h"
//
//NonRigidICP::NonRigidICP(const std::vector<ml::vec3f>& src,
//						 const std::vector<ml::vec3f>& dst,
//						 ceres::Solver::Options options)
//	: _src(src)
//	, _dst(dst)
//	, _options(options)
//	, _nn_search(dst)
//{
//	std::cout << "\nCeres Solver" << std::endl;
//	std::cout << "Ceres preconditioner type: " << options.preconditioner_type << std::endl;
//	std::cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << std::endl;
//	std::cout << "Ceres linear solver type: " << options.linear_solver_type << std::endl;
//}
//
//ml::mat4f NonRigidICP::solve()
//{
//	ml::mat4f transformation = ml::mat4f::identity();
//	while (!finished()) {
//		transformation = solveIteration();
//	}
//	return transformation;
//}
//
//
//ml::mat4f NonRigidICP::solveIteration() // working
//{
//	if (!finished()) {
//		ceres::Solver::Summary summary;
//		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration);
//
//		_solve_iteration++;
//
//		ceres::Problem problem;
//		for (int i = 0; i < _src.size(); i++) {
//			ml::mat4f transform_matrix = rigid_transformation_from_se3(_transformation_se3);
//			unsigned int index = _nn_search.nearest_index((transform_matrix * _src[i]));
//			ceres::CostFunction* cost_function = PointToPointErrorSE3::Create(_dst[index], _src[i]);
//			problem.AddResidualBlock(cost_function, NULL, _transformation_se3.array);
//		}
//		ceres::Solve(_options, &problem, &summary);
//
//		_current_tol = abs(_current_cost - summary.final_cost);
//		_current_cost = summary.final_cost;
//
//		_total_time_in_ms += logger.get_time_in_ms();
//	}
//
//	return rigid_transformation_from_se3(_transformation_se3);
//}
//
//
//bool NonRigidICP::finished()
//{
//	double tol = 0.001;
//	return (_solve_iteration >= _max_iterations || _current_tol < tol);
//}
